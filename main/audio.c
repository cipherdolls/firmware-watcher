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
#include "record.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define ES8311_ADDR         0x18  // ADDR pin low on SenseCAP Watcher
#define I2S_MCLK_MULTIPLE   256   // matches I2S_STD_CLK_DEFAULT_CONFIG

// PCM format: 16-bit signed LE, 24 kHz, mono — matches Kokoro/ElevenLabs streaming
#define PCM_SAMPLE_RATE     24000
#define PCM_BUF_SIZE        4096   // raw PCM byte buffer (2048 mono samples)
#define STEREO_BUF_SAMPLES  2048   // max mono samples per chunk
#define STEREO_BUF_SIZE     (STEREO_BUF_SAMPLES * 2 * sizeof(int16_t))  // 8192 bytes

static const char *TAG = "audio";

static void update_play_rms(const int16_t *pcm, int samples)
{
    if (samples <= 0) { g_audio_rms = 0; return; }
    uint64_t sum_sq = 0;
    for (int i = 0; i < samples; i++) {
        int32_t v = pcm[i];
        sum_sq += (uint64_t)(v * v);
    }
    g_audio_rms = (uint16_t)sqrtf((float)sum_sq / samples);
}

static i2s_chan_handle_t   s_tx_chan    = NULL;
static QueueHandle_t      s_queue     = NULL;
static SemaphoreHandle_t  s_play_mutex = NULL;
static volatile bool      s_stop      = false;

// PSRAM-allocated buffers
static uint8_t  *s_pcm_buf = NULL;  // raw PCM byte input buffer
static int16_t  *s_stereo  = NULL;  // mono→stereo expansion buffer

// Static task descriptor must be in DRAM; stack goes in PSRAM
static StaticTask_t s_audio_tcb;

#define AUDIO_MSG_ID_MAX  80

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

// ── Helper: play a chunk of mono PCM samples through I2S (stereo) ───────────

static void play_pcm_chunk(const int16_t *mono, int samples)
{
    update_play_rms(mono, samples);

    // Mono → stereo expansion
    for (int i = 0; i < samples; i++) {
        s_stereo[i * 2]     = mono[i];
        s_stereo[i * 2 + 1] = mono[i];
    }

    size_t written = 0;
    i2s_channel_write(s_tx_chan, s_stereo,
                      (size_t)samples * 2 * sizeof(int16_t),
                      &written, pdMS_TO_TICKS(2000));
}

// ── Flush DMA with silence and tear down I2S ────────────────────────────────

static void flush_and_stop_i2s(void)
{
    memset(s_stereo, 0, STEREO_BUF_SIZE);
    size_t zw = 0;
    i2s_channel_write(s_tx_chan, s_stereo, STEREO_BUF_SIZE, &zw, pdMS_TO_TICKS(500));
    vTaskDelay(pdMS_TO_TICKS(50));
    i2s_stop_ch();
}

// ── Stream-decode: download PCM + play simultaneously ───────────────────────
// Opens HTTP GET, reads raw PCM chunks, expands mono→stereo, plays via I2S.

static void stream_play_pcm(const char *message_id)
{
    if (xSemaphoreTake(s_play_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Audio busy, skipping HTTP play for %s", message_id);
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

    ESP_LOGI(TAG, "Streaming PCM for msg %s", message_id);

    s_stop = false;
    xEventGroupSetBits(g_events, EVT_AUDIO_PLAYING);
    display_set_state(DISPLAY_STATE_PLAYING, "Playing...");

    i2s_start(PCM_SAMPLE_RATE);
    i2s_started = true;

    size_t buf_fill = 0;
    bool   http_done = false;

    while (!s_stop) {
        // Fill buffer from HTTP
        if (!http_done && buf_fill < PCM_BUF_SIZE) {
            int rd = esp_http_client_read(client,
                        (char *)(s_pcm_buf + buf_fill),
                        PCM_BUF_SIZE - buf_fill);
            if (rd > 0) {
                buf_fill += rd;
            } else {
                http_done = true;
            }
        }

        // Align to sample boundary (2 bytes per 16-bit sample)
        size_t aligned = buf_fill & ~(size_t)1;
        if (aligned == 0) {
            if (http_done) break;
            continue;
        }

        int samples = (int)(aligned / sizeof(int16_t));
        play_pcm_chunk((const int16_t *)s_pcm_buf, samples);

        // Keep any leftover odd byte
        if (buf_fill > aligned) {
            s_pcm_buf[0] = s_pcm_buf[aligned];
            buf_fill = 1;
        } else {
            buf_fill = 0;
        }
    }

    if (i2s_started) {
        flush_and_stop_i2s();
    }

    g_audio_rms = 0;
    xEventGroupClearBits(g_events, EVT_AUDIO_PLAYING);

    // Restore display
    const char *msg = strlen(g_config.chat_id) > 0 ? "" : "No chat linked";
    display_set_state(DISPLAY_STATE_WIFI_OK, msg);

cleanup:
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    xSemaphoreGive(s_play_mutex);
}

// ── Play task ─────────────────────────────────────────────────────────────────

static void audio_play_task(void *arg)
{
    play_req_t req;
    while (1) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE) continue;
        stream_play_pcm(req.message_id);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void audio_init(void)
{
    // Allocate buffers from PSRAM so internal SRAM stays free for TLS/WiFi heap
    s_pcm_buf = heap_caps_malloc(PCM_BUF_SIZE,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_stereo  = heap_caps_malloc(STEREO_BUF_SIZE,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_pcm_buf && s_stereo);

    // Task stack in PSRAM — PCM playback needs minimal stack (~4 KB)
    StackType_t *audio_stack = heap_caps_malloc(
        16384, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(audio_stack);

    s_play_mutex = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(4, sizeof(play_req_t));
    xTaskCreateStaticPinnedToCore(audio_play_task, "audio_play",
        16384 / sizeof(StackType_t), NULL, 5, audio_stack, &s_audio_tcb, 0);

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

    ESP_LOGI(TAG, "Audio subsystem ready (PCM, %d Hz, PSRAM buffers)", PCM_SAMPLE_RATE);
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
// Receives raw PCM (16-bit, 24 kHz, mono) from the stream buffer, expands
// to stereo, and plays through I2S.  Blocks until stream ends or audio_stop().

void audio_stream_play(StreamBufferHandle_t stream,
                       volatile bool *stream_active,
                       volatile bool *stream_error,
                       uint8_t first_byte)
{
    if (xSemaphoreTake(s_play_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Audio busy, skipping stream play");
        return;
    }

    s_stop = false;
    xEventGroupSetBits(g_events, EVT_AUDIO_PLAYING);
    display_set_state(DISPLAY_STATE_PLAYING, "Playing...");

    i2s_start(PCM_SAMPLE_RATE);

    // Seed buffer with the first peeked byte
    s_pcm_buf[0] = first_byte;
    size_t buf_fill = 1;

    while (!s_stop) {
        // Fill buffer from StreamBuffer — wait longer when stream is active
        // to avoid premature exit on temporary gaps between server bursts
        if (buf_fill < PCM_BUF_SIZE) {
            TickType_t wait = *stream_active ? pdMS_TO_TICKS(2000) : pdMS_TO_TICKS(200);
            size_t rd = xStreamBufferReceive(stream,
                            s_pcm_buf + buf_fill,
                            PCM_BUF_SIZE - buf_fill,
                            wait);
            buf_fill += rd;
        }

        // Check for error or stop
        if (*stream_error && buf_fill == 0) {
            ESP_LOGW(TAG, "Stream error, aborting playback");
            break;
        }

        // Align to sample boundary (2 bytes per 16-bit sample)
        size_t aligned = buf_fill & ~(size_t)1;
        if (aligned == 0) {
            // Only exit when stream is truly done (not active + buffer empty)
            if (!(*stream_active) && xStreamBufferBytesAvailable(stream) == 0)
                break;
            continue;
        }

        int samples = (int)(aligned / sizeof(int16_t));
        play_pcm_chunk((const int16_t *)s_pcm_buf, samples);

        // Keep any leftover odd byte
        if (buf_fill > aligned) {
            s_pcm_buf[0] = s_pcm_buf[aligned];
            buf_fill = 1;
        } else {
            buf_fill = 0;
        }
    }

    flush_and_stop_i2s();

    g_audio_rms = 0;
    xEventGroupClearBits(g_events, EVT_AUDIO_PLAYING);

    const char *msg = strlen(g_config.chat_id) > 0 ? "" : "No chat linked";
    display_set_state(DISPLAY_STATE_WIFI_OK, msg);

    xSemaphoreGive(s_play_mutex);
}
