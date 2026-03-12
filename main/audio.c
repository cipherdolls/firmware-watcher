#include "audio.h"
#include "board.h"
#include "config.h"
#include "events.h"
#include "display.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "driver/i2s_std.h"
#include "driver/i2c.h"
#include "es8311.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include <string.h>
#include <stdio.h>

#define ES8311_ADDR         0x18  // ADDR pin low on SenseCAP Watcher
#define I2S_MCLK_MULTIPLE   256   // matches I2S_STD_CLK_DEFAULT_CONFIG

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#include "minimp3.h"

static const char *TAG = "audio";

static i2s_chan_handle_t   s_tx_chan    = NULL;
static QueueHandle_t      s_queue     = NULL;
static SemaphoreHandle_t  s_play_mutex = NULL;
static volatile bool      s_stop      = false;

// PSRAM-allocated decode buffers — keeps internal SRAM free for TLS/WiFi/LVGL heap
static mp3dec_t  *s_dec    = NULL;
static int16_t   *s_pcm    = NULL;  // MINIMP3_MAX_SAMPLES_PER_FRAME*2 shorts
static int16_t   *s_stereo = NULL;  // mono→stereo expansion buffer (same size)

// Static task descriptor must be in DRAM; stack goes in PSRAM
static StaticTask_t s_audio_tcb;

#define AUDIO_MSG_ID_MAX  80
#define STREAM_BUF_SIZE   8192   // MP3 accumulation buffer (enough for several frames)

typedef struct {
    char message_id[AUDIO_MSG_ID_MAX];
} play_req_t;

// ── I2S ──────────────────────────────────────────────────────────────────────

static es8311_handle_t s_codec = NULL;

static void codec_init(int sample_rate)
{
    // I2C is already initialized by display.c (lcd_power_on uses AUDIO_I2C_PORT)
    s_codec = es8311_create(AUDIO_I2C_PORT, ES8311_ADDR);
    if (!s_codec) {
        ESP_LOGE(TAG, "ES8311 create failed");
        return;
    }

    es8311_clock_config_t clk = {
        .mclk_inverted      = false,
        .sclk_inverted      = false,
        .mclk_from_mclk_pin = true,
        .sample_frequency   = sample_rate,
    };
    ESP_ERROR_CHECK(es8311_init(s_codec, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    ESP_ERROR_CHECK(es8311_sample_frequency_config(s_codec, I2S_MCLK_MULTIPLE * sample_rate, sample_rate));
    ESP_ERROR_CHECK(es8311_voice_volume_set(s_codec, 70, NULL));
    ESP_ERROR_CHECK(es8311_microphone_config(s_codec, false));
    ESP_LOGI(TAG, "ES8311 initialized at %d Hz, volume 70", sample_rate);
}

static void i2s_start(int sample_rate)
{
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk         = I2S_MCLK,
            .bclk         = I2S_BCLK,
            .ws           = I2S_WS,
            .dout         = I2S_DOUT,
            .din          = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    // Configure codec sample rate now that MCLK is running from I2S
    if (s_codec) {
        ESP_ERROR_CHECK(es8311_sample_frequency_config(s_codec, I2S_MCLK_MULTIPLE * sample_rate, sample_rate));
        es8311_voice_mute(s_codec, false);
        ESP_ERROR_CHECK(es8311_voice_volume_set(s_codec, 70, NULL));
    } else {
        codec_init(sample_rate);
    }

    ESP_LOGI(TAG, "I2S started: %d Hz stereo (Philips)", sample_rate);
}

static void i2s_stop_ch(void)
{
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }
}

// ── Stream-decode: download MP3 + decode + play simultaneously ────────────────
// Opens HTTP GET, reads chunks into a small buffer, decodes MP3 frames as they
// arrive, and plays them via I2S immediately.  No waiting for the full download.

static void stream_play_mp3(const char *message_id)
{
    if (xSemaphoreTake(s_play_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Audio busy, skipping HTTP play for %s", message_id);
        return;
    }

    uint8_t *sbuf = heap_caps_malloc(STREAM_BUF_SIZE,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!sbuf) {
        ESP_LOGE(TAG, "No memory for stream buffer");
        xSemaphoreGive(s_play_mutex);
        return;
    }

    char url[256], auth[128];
    snprintf(url,  sizeof(url),  "%s/messages/%s/audio",
             g_config.server_url, message_id);
    snprintf(auth, sizeof(auth), "Bearer %s", g_config.apikey);

    esp_http_client_config_t cfg = {
        .url               = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size       = 4096,
        .timeout_ms        = 20000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Authorization", auth);

    bool i2s_started = false;

    if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed for %s", message_id);
        goto cleanup;
    }

    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP %d for %s", status, message_id);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Streaming MP3 for msg %s", message_id);

    mp3dec_init(s_dec);
    s_stop = false;
    xEventGroupSetBits(g_events, EVT_AUDIO_PLAYING);
    display_set_state(DISPLAY_STATE_PLAYING, "Playing...");

    size_t buf_fill = 0;
    bool   http_done = false;

    while (!s_stop) {
        // ── Fill buffer from HTTP ────────────────────────────────────────────
        if (!http_done && buf_fill < STREAM_BUF_SIZE) {
            int rd = esp_http_client_read(client,
                        (char *)(sbuf + buf_fill),
                        STREAM_BUF_SIZE - buf_fill);
            if (rd > 0) {
                buf_fill += rd;
            } else {
                http_done = true;
            }
        }

        if (buf_fill == 0) break;

        // ── Decode one MP3 frame ─────────────────────────────────────────────
        mp3dec_frame_info_t info = {};
        int samples = mp3dec_decode_frame(s_dec, sbuf, (int)buf_fill,
                                           s_pcm, &info);

        if (info.frame_bytes == 0) {
            if (http_done) break;   // no more data, no more frames
            continue;               // need more data from network
        }

        // Consume decoded bytes
        buf_fill -= info.frame_bytes;
        if (buf_fill > 0)
            memmove(sbuf, sbuf + info.frame_bytes, buf_fill);

        if (samples <= 0) continue;  // ID3 / padding frame

        // Init I2S once we know the sample rate from the first decoded frame
        if (!i2s_started) {
            i2s_start(info.hz);
            i2s_started = true;
        }

        // ── Play decoded PCM ─────────────────────────────────────────────────
        int16_t *out   = s_pcm;
        size_t   bytes = (size_t)samples * info.channels * sizeof(int16_t);

        if (info.channels == 1) {
            for (int i = 0; i < samples; i++) {
                s_stereo[i * 2]     = s_pcm[i];
                s_stereo[i * 2 + 1] = s_pcm[i];
            }
            out   = s_stereo;
            bytes = (size_t)samples * 2 * sizeof(int16_t);
        }

        size_t written = 0;
        i2s_channel_write(s_tx_chan, out, bytes, &written, pdMS_TO_TICKS(2000));
    }

    if (i2s_started) {
        // Flush DMA with silence to prevent echo/artifact at end
        memset(s_pcm, 0, MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int16_t));
        size_t zw = 0;
        i2s_channel_write(s_tx_chan, s_pcm, MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int16_t), &zw, pdMS_TO_TICKS(500));
        vTaskDelay(pdMS_TO_TICKS(50));
        i2s_stop_ch();
    }

    xEventGroupClearBits(g_events, EVT_AUDIO_PLAYING);

    // Restore display
    const char *msg = strlen(g_config.chat_id) > 0 ? "" : "No chat linked";
    display_set_state(DISPLAY_STATE_WIFI_OK, msg);

cleanup:
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(sbuf);
    xSemaphoreGive(s_play_mutex);
}

// ── Play task ─────────────────────────────────────────────────────────────────

static void audio_play_task(void *arg)
{
    play_req_t req;
    while (1) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE) continue;
        stream_play_mp3(req.message_id);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void audio_init(void)
{
    // Allocate decode buffers from PSRAM so internal SRAM stays free for TLS/WiFi heap
    s_dec    = heap_caps_malloc(sizeof(mp3dec_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_pcm    = heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int16_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_stereo = heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int16_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_dec && s_pcm && s_stereo);

    // Allocate task stack from PSRAM — minimp3 decode uses ~10 KB of stack
    // (float grbuf[2][576] + call chain), keeping DRAM free for LVGL/TLS
    StackType_t *audio_stack = heap_caps_malloc(
        32768, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(audio_stack);

    s_play_mutex = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(4, sizeof(play_req_t));
    xTaskCreateStaticPinnedToCore(audio_play_task, "audio_play",
        32768 / sizeof(StackType_t), NULL, 5, audio_stack, &s_audio_tcb, 0);

    // Early-init ES8311 codec so speaker mute works before first playback.
    // Without this, s_codec is NULL and mute calls during recording are no-ops,
    // causing speaker clicking/noise from the uninitialised DAC.
    // Only do basic I2C init + mute — skip sample_frequency_config because
    // MCLK is generated by I2S TX and isn't running yet.
    s_codec = es8311_create(AUDIO_I2C_PORT, ES8311_ADDR);
    if (s_codec) {
        es8311_clock_config_t clk = {
            .mclk_inverted      = false,
            .sclk_inverted      = false,
            .mclk_from_mclk_pin = true,
            .sample_frequency   = 44100,
        };
        es8311_init(s_codec, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
        es8311_voice_volume_set(s_codec, 0, NULL);
        es8311_voice_mute(s_codec, true);
        es8311_microphone_config(s_codec, false);
        ESP_LOGI(TAG, "ES8311 early init — muted, awaiting I2S for sample rate config");
    }

    ESP_LOGI(TAG, "Audio subsystem ready (PSRAM stack + decode buffers)");
}

void audio_play_message(const char *message_id)
{
    play_req_t req = {};
    strlcpy(req.message_id, message_id, sizeof(req.message_id));
    if (xQueueSend(s_queue, &req, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Play queue full, dropping %s", message_id);
    }
}

void audio_stop(void)
{
    s_stop = true;
}

void audio_speaker_mute(void)
{
    if (s_codec) {
        es8311_voice_volume_set(s_codec, 0, NULL);
        es8311_voice_mute(s_codec, true);
    }
    // Disable I2S TX to stop bus noise from reaching the speaker amp
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
    }
}

void audio_speaker_unmute(void)
{
    // Re-enable I2S TX before unmuting codec
    if (s_tx_chan) {
        i2s_channel_enable(s_tx_chan);
    }
    if (s_codec) {
        es8311_voice_mute(s_codec, false);
        es8311_voice_volume_set(s_codec, 70, NULL);
    }
}

// ── Stream-fed playback (from stream_player WebSocket) ──────────────────────

void audio_stream_play(StreamBufferHandle_t stream,
                       volatile bool *stream_active,
                       volatile bool *stream_error,
                       uint8_t first_byte)
{
    if (xSemaphoreTake(s_play_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Audio busy, skipping stream play");
        return;
    }

    uint8_t *sbuf = heap_caps_malloc(STREAM_BUF_SIZE,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!sbuf) {
        ESP_LOGE(TAG, "No memory for stream buffer");
        xSemaphoreGive(s_play_mutex);
        return;
    }

    bool i2s_started = false;
    mp3dec_init(s_dec);
    s_stop = false;
    xEventGroupSetBits(g_events, EVT_AUDIO_PLAYING);
    display_set_state(DISPLAY_STATE_PLAYING, "Playing...");

    // Seed buffer with the first peeked byte
    sbuf[0] = first_byte;
    size_t buf_fill = 1;

    while (!s_stop) {
        // Fill buffer from StreamBuffer
        // Don't block when stream has ended — avoids I2S DMA replaying stale audio
        if (buf_fill < STREAM_BUF_SIZE) {
            TickType_t wait = *stream_active ? pdMS_TO_TICKS(200) : 0;
            size_t rd = xStreamBufferReceive(stream,
                            sbuf + buf_fill,
                            STREAM_BUF_SIZE - buf_fill,
                            wait);
            buf_fill += rd;
        }

        // Check for error with empty buffer
        if (*stream_error && buf_fill == 0) {
            ESP_LOGW(TAG, "Stream error, aborting playback");
            break;
        }

        if (buf_fill == 0) {
            if (!(*stream_active) && xStreamBufferBytesAvailable(stream) == 0)
                break;  // stream ended, no more data
            continue;    // wait for more data
        }

        // Decode one MP3 frame
        mp3dec_frame_info_t info = {};
        int samples = mp3dec_decode_frame(s_dec, sbuf, (int)buf_fill,
                                           s_pcm, &info);

        if (info.frame_bytes == 0) {
            // Need more data
            if (!(*stream_active) && xStreamBufferBytesAvailable(stream) == 0)
                break;
            continue;
        }

        buf_fill -= info.frame_bytes;
        if (buf_fill > 0)
            memmove(sbuf, sbuf + info.frame_bytes, buf_fill);

        if (samples <= 0) continue;  // ID3 / padding frame

        // Init I2S once we know the sample rate
        if (!i2s_started) {
            i2s_start(info.hz);
            i2s_started = true;
        }

        // Play decoded PCM
        int16_t *out   = s_pcm;
        size_t   bytes = (size_t)samples * info.channels * sizeof(int16_t);

        if (info.channels == 1) {
            for (int i = 0; i < samples; i++) {
                s_stereo[i * 2]     = s_pcm[i];
                s_stereo[i * 2 + 1] = s_pcm[i];
            }
            out   = s_stereo;
            bytes = (size_t)samples * 2 * sizeof(int16_t);
        }

        size_t written = 0;
        i2s_channel_write(s_tx_chan, out, bytes, &written, pdMS_TO_TICKS(2000));
    }

    if (i2s_started) {
        // Flush DMA with silence to prevent echo/artifact at end
        memset(s_pcm, 0, MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int16_t));
        size_t zw = 0;
        i2s_channel_write(s_tx_chan, s_pcm, MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int16_t), &zw, pdMS_TO_TICKS(500));
        vTaskDelay(pdMS_TO_TICKS(50));
        i2s_stop_ch();
    }

    xEventGroupClearBits(g_events, EVT_AUDIO_PLAYING);

    const char *msg = strlen(g_config.chat_id) > 0 ? "" : "No chat linked";
    display_set_state(DISPLAY_STATE_WIFI_OK, msg);

    free(sbuf);
    xSemaphoreGive(s_play_mutex);
}
