#include "record.h"
#include "audio.h"
#include "stream_player.h"
#include "mqtt.h"
#include "board.h"
#include "config.h"
#include "events.h"
#include "display.h"
#include "touch.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "driver/i2s_std.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/stream_buffer.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "record";

volatile uint16_t g_audio_rms = 0;

#define SAMPLE_RATE      16000
#define RECORD_MAX_S     20
#define I2S_READ_BYTES   2048           // stereo read buffer per iteration
#define RING_BUF_BYTES   (128 * 1024)   // 128 KB ring buffer in PSRAM (~4 s audio)
#define SEND_CHUNK       4096           // 4 KB per WS send

#define ES7243_ADDR  0x14   // Confirmed by I2C scan on SenseCAP Watcher

// ── VAD configuration ────────────────────────────────────────────────────────

#define VAD_RMS_THRESHOLD       200     // RMS level above which speech is detected
#define VAD_SILENCE_TIMEOUT_MS  1500    // 1.5 s of silence ends recording
#define VAD_CONFIRM_CHUNKS      3       // ~96 ms of consecutive speech to confirm onset
#define LISTEN_TIMEOUT_S        60      // Max time in LISTENING before auto-exit
#define WAIT_RESPONSE_TIMEOUT_S 30      // Max time waiting for server response

// Pre-speech circular buffer — captures ~300 ms before VAD triggers
#define PRE_SPEECH_BYTES  (10 * 1024)   // ~312 ms at 16 kHz mono 16-bit

typedef enum {
    CONV_OFF,
    CONV_LISTENING,
    CONV_RECORDING,
    CONV_WAITING,
    CONV_PLAYING,
} conv_state_t;

static volatile conv_state_t s_conv_state = CONV_OFF;

// ── Shared state ─────────────────────────────────────────────────────────────

static i2s_chan_handle_t s_rx_chan  = NULL;
static bool             s_mic_init = false;

// Ring buffer for Core 0 → Core 1 audio transfer
static StreamBufferHandle_t s_ring_buf;
static StaticStreamBuffer_t s_ring_struct;
static uint8_t             *s_ring_storage;
static volatile bool        s_reader_running;

// Reader task stack in PSRAM (keeps internal heap free for TLS)
#define READER_STACK_WORDS 4096   // 4096 words = 16 KB stack
static StackType_t         *s_reader_stack;
static StaticTask_t         s_reader_tcb;

// Pre-speech circular buffer
static uint8_t  *s_pre_buf;
static size_t    s_pre_write;
static size_t    s_pre_fill;

// VAD event group (reader → main task signaling)
static EventGroupHandle_t s_vad_events;
#define VAD_EVT_SPEECH   (1 << 0)
#define VAD_EVT_SILENCE  (1 << 1)

static volatile uint32_t s_speech_chunks;  // consecutive loud chunks counter
static TimerHandle_t     s_silence_timer;
static TickType_t        s_wait_start;

// Send buffer (allocated once, reused)
static uint8_t *s_send_buf;

// ── Persistent WebSocket to stream-recorder ─────────────────────────────────

static esp_websocket_client_handle_t s_ws_client = NULL;
static volatile bool s_ws_connected = false;

// ── WAV header ────────────────────────────────────────────────────────────────

typedef struct __attribute__((packed)) {
    char     riff[4];           // "RIFF"
    uint32_t file_size;         // total file size - 8
    char     wave[4];           // "WAVE"
    char     fmt_id[4];         // "fmt "
    uint32_t fmt_size;          // 16
    uint16_t audio_format;      // 1 = PCM
    uint16_t channels;          // 1
    uint32_t sample_rate;       // 16000
    uint32_t byte_rate;         // sample_rate * channels * bits/8
    uint16_t block_align;       // channels * bits/8
    uint16_t bits_per_sample;   // 16
    char     data_id[4];        // "data"
    uint32_t data_size;         // PCM byte count
} wav_hdr_t;

static void build_wav_header(wav_hdr_t *h, uint32_t pcm_bytes)
{
    memcpy(h->riff,    "RIFF", 4);
    h->file_size       = pcm_bytes + sizeof(wav_hdr_t) - 8;
    memcpy(h->wave,    "WAVE", 4);
    memcpy(h->fmt_id,  "fmt ", 4);
    h->fmt_size        = 16;
    h->audio_format    = 1;
    h->channels        = 1;
    h->sample_rate     = SAMPLE_RATE;
    h->byte_rate       = SAMPLE_RATE * 2;   // 1 ch × 2 bytes
    h->block_align     = 2;
    h->bits_per_sample = 16;
    memcpy(h->data_id, "data", 4);
    h->data_size       = pcm_bytes;
}

// ── ES7243E ADC init ──────────────────────────────────────────────────────────

static void es7243_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ES7243_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(AUDIO_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ES7243E write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
}

static void es7243e_init(void)
{
    es7243_write(0x01, 0x3A);
    es7243_write(0x00, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    es7243_write(0xF9, 0x00);
    es7243_write(0x04, 0x02);
    es7243_write(0x04, 0x01);
    es7243_write(0xF9, 0x01);
    es7243_write(0x00, 0x1E);
    es7243_write(0x01, 0x00);
    es7243_write(0x02, 0x00);
    es7243_write(0x03, 0x20);
    es7243_write(0x04, 0x01);
    es7243_write(0x0D, 0x00);
    es7243_write(0x05, 0x00);
    es7243_write(0x06, 0x03);
    es7243_write(0x07, 0x00);
    es7243_write(0x08, 0xFF);
    es7243_write(0x09, 0xCA);
    es7243_write(0x0A, 0x85);
    es7243_write(0x0B, 0x00);
    es7243_write(0x0E, 0xBF);
    es7243_write(0x0F, 0x80);
    es7243_write(0x14, 0x0C);
    es7243_write(0x15, 0x0C);
    es7243_write(0x17, 0x02);
    es7243_write(0x18, 0x26);
    es7243_write(0x19, 0x77);
    es7243_write(0x1A, 0xF4);
    es7243_write(0x1B, 0x66);
    es7243_write(0x1C, 0x44);
    es7243_write(0x1E, 0x00);
    es7243_write(0x1F, 0x0C);
    es7243_write(0x20, 0x1A);
    es7243_write(0x21, 0x1A);
    es7243_write(0x00, 0x80);
    es7243_write(0x01, 0x3A);
    es7243_write(0x16, 0x3F);
    es7243_write(0x16, 0x00);
    ESP_LOGI(TAG, "ES7243E init done (addr=0x%02X, chip ID 0x7A43)", ES7243_ADDR);
}

// ── Knob button (for sleep toggle) ──────────────────────────────────────────

static bool s_knob_btn_ok = false;

static bool knob_btn_pressed(void)
{
    uint8_t reg = PCA9535_INPUT0;
    uint8_t val = 0xFF;
    esp_err_t err = i2c_master_write_read_device(
        AUDIO_I2C_PORT, IO_EXP_ADDR,
        &reg, 1, &val, 1, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        if (s_knob_btn_ok) {
            ESP_LOGW(TAG, "Knob I2C read failed: %s", esp_err_to_name(err));
        }
        return false;
    }
    if (!s_knob_btn_ok) {
        s_knob_btn_ok = true;
        ESP_LOGI(TAG, "Knob IO expander 0x%02X responding, port0=0x%02X",
                 IO_EXP_ADDR, val);
    }
    return !(val & (1 << KNOB_BTN_BIT));
}

static void knob_init(void)
{
    uint8_t cmd[] = { PCA9535_CONFIG0, (1 << KNOB_BTN_BIT) };
    esp_err_t err = i2c_master_write_to_device(
        AUDIO_I2C_PORT, IO_EXP_ADDR,
        cmd, sizeof(cmd), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Knob IO expander 0x%02X not found (%s) — button disabled",
                 IO_EXP_ADDR, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Knob button configured (IO exp 0x%02X, port0 pin %d)",
                 IO_EXP_ADDR, KNOB_BTN_BIT);
    }
}

// ── Touch helpers (for conversation toggle) ──────────────────────────────────

static bool screen_tapped(void)
{
    uint16_t tx, ty;
    return touch_get_point(&tx, &ty);
}

static void wait_touch_release(void)
{
    while (screen_tapped()) vTaskDelay(pdMS_TO_TICKS(30));
}

// ── I2S RX ────────────────────────────────────────────────────────────────────

static void i2s_rx_start(void)
{
    i2s_chan_config_t cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&cfg, NULL, &s_rx_chan));

    i2s_std_config_t std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK,
            .ws   = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx_chan, &std));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));
    ESP_LOGI(TAG, "I2S RX started at %d Hz mono", SAMPLE_RATE);
}

static void i2s_rx_stop(void)
{
    if (s_rx_chan) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        ESP_LOGI(TAG, "I2S RX stopped");
    }
}

// ── Persistent WebSocket (like webapp useStreamRecorder) ────────────────────

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t event_id, void *event_data)
{
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Stream-recorder WS connected");
        s_ws_connected = true;
        display_set_ws_status(
            (xEventGroupGetBits(g_events) & EVT_STREAM_CONNECTED) != 0, true);
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Stream-recorder WS disconnected");
        s_ws_connected = false;
        display_set_ws_status(
            (xEventGroupGetBits(g_events) & EVT_STREAM_CONNECTED) != 0, false);
        break;
    case WEBSOCKET_EVENT_DATA: {
        esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
        if (data->op_code == 0x01 && data->data_len > 0) {
            ESP_LOGI(TAG, "Stream-recorder: %.*s", data->data_len, data->data_ptr);
        }
        break;
    }
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "Stream-recorder WS error");
        break;
    default:
        break;
    }
}

static void build_ws_url(char *url, size_t url_size)
{
    if (strncmp(g_config.stream_recorder_url, "https://", 8) == 0) {
        snprintf(url, url_size, "wss://%s/ws-stream?chatId=%s&auth=%s",
                 g_config.stream_recorder_url + 8, g_config.chat_id, g_config.apikey);
    } else if (strncmp(g_config.stream_recorder_url, "http://", 7) == 0) {
        snprintf(url, url_size, "ws://%s/ws-stream?chatId=%s&auth=%s",
                 g_config.stream_recorder_url + 7, g_config.chat_id, g_config.apikey);
    } else {
        snprintf(url, url_size, "ws://%s/ws-stream?chatId=%s&auth=%s",
                 g_config.stream_recorder_url, g_config.chat_id, g_config.apikey);
    }
}

static void ws_recorder_disconnect(void)
{
    if (!s_ws_client) return;
    esp_websocket_client_stop(s_ws_client);
    esp_websocket_client_destroy(s_ws_client);
    s_ws_client = NULL;
    s_ws_connected = false;
    ESP_LOGI(TAG, "Stream-recorder disconnected");
}

static void ws_recorder_connect(void)
{
    if (s_ws_client) return;
    if (strlen(g_config.chat_id) == 0) return;

    char url[384];
    build_ws_url(url, sizeof(url));

    esp_websocket_client_config_t ws_cfg = {
        .uri                    = url,
        .buffer_size            = 4096,
        .disable_auto_reconnect = false,
        .reconnect_timeout_ms   = 5000,
        .pingpong_timeout_sec   = 30,
        .task_stack             = 8192,
        .task_prio              = 3,
    };
    if (strncmp(url, "wss://", 6) == 0) {
        ws_cfg.crt_bundle_attach = esp_crt_bundle_attach;
    }

    s_ws_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(s_ws_client, WEBSOCKET_EVENT_ANY,
                                   ws_event_handler, NULL);
    esp_websocket_client_start(s_ws_client);
    ESP_LOGI(TAG, "Stream-recorder connecting...");
}

// ── Display helper ────────────────────────────────────────────────────────────

static void restore_idle_display(void)
{
    const char *msg = strlen(g_config.chat_id) > 0 ? "Tap to talk" : "No chat linked";
    display_set_state(DISPLAY_STATE_WIFI_OK, msg);
}

// ── VAD helpers ──────────────────────────────────────────────────────────────

static uint16_t compute_rms(const int16_t *samples, size_t count)
{
    if (count == 0) return 0;
    uint64_t sum_sq = 0;
    for (size_t i = 0; i < count; i++) {
        int32_t s = samples[i];
        sum_sq += (uint64_t)(s * s);
    }
    return (uint16_t)sqrtf((float)sum_sq / count);
}

static void pre_buf_write(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        s_pre_buf[s_pre_write] = data[i];
        s_pre_write = (s_pre_write + 1) % PRE_SPEECH_BYTES;
    }
    s_pre_fill += len;
    if (s_pre_fill > PRE_SPEECH_BYTES) s_pre_fill = PRE_SPEECH_BYTES;
}

static size_t pre_buf_drain_to_ring(void)
{
    if (s_pre_fill == 0) return 0;
    size_t start = (s_pre_write + PRE_SPEECH_BYTES - s_pre_fill) % PRE_SPEECH_BYTES;
    size_t to_read = s_pre_fill;

    if (start + to_read <= PRE_SPEECH_BYTES) {
        xStreamBufferSend(s_ring_buf, s_pre_buf + start, to_read, pdMS_TO_TICKS(100));
    } else {
        size_t first = PRE_SPEECH_BYTES - start;
        xStreamBufferSend(s_ring_buf, s_pre_buf + start, first, pdMS_TO_TICKS(50));
        xStreamBufferSend(s_ring_buf, s_pre_buf, to_read - first, pdMS_TO_TICKS(50));
    }

    size_t drained = s_pre_fill;
    s_pre_fill = 0;
    s_pre_write = 0;
    return drained;
}

static void silence_timer_cb(TimerHandle_t timer)
{
    xEventGroupSetBits(s_vad_events, VAD_EVT_SILENCE);
}

// ── I2S reader task — runs on Core 0, feeds ring buffer / pre-buf ───────────

static void i2s_reader_task(void *arg)
{
    uint8_t *buf = heap_caps_malloc(I2S_READ_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "i2s_reader: malloc failed");
        s_reader_running = false;
        vTaskDelete(NULL);
        return;
    }

    while (s_reader_running) {
        size_t got = 0;
        i2s_channel_read(s_rx_chan, buf, I2S_READ_BYTES, &got, pdMS_TO_TICKS(200));
        if (got == 0) continue;

        // Downsample stereo→mono in-place (keep right channel)
        int16_t *s = (int16_t *)buf;
        size_t n_mono = got / 4;
        for (size_t i = 0; i < n_mono; i++) {
            s[i] = s[i * 2 + 1];
        }
        size_t mono_bytes = n_mono * 2;

        uint16_t rms = compute_rms(s, n_mono);
        g_audio_rms = rms;
        bool loud = (rms > VAD_RMS_THRESHOLD);

        conv_state_t state = s_conv_state;

        if (state == CONV_LISTENING) {
            pre_buf_write(buf, mono_bytes);
            if (loud) {
                s_speech_chunks++;
                if (s_speech_chunks >= VAD_CONFIRM_CHUNKS) {
                    xEventGroupSetBits(s_vad_events, VAD_EVT_SPEECH);
                }
            } else {
                s_speech_chunks = 0;
            }
        } else if (state == CONV_RECORDING) {
            xStreamBufferSend(s_ring_buf, buf, mono_bytes, pdMS_TO_TICKS(50));
            if (loud) {
                xTimerReset(s_silence_timer, 0);
            }
        }
    }

    g_audio_rms = 0;
    heap_caps_free(buf);
    ESP_LOGI(TAG, "I2S reader task exiting");
    vTaskDelete(NULL);
}

// ── Start listening (I2S RX + reader task) ───────────────────────────────────

static void start_listening(void)
{
    audio_speaker_mute();
    i2s_rx_start();
    if (!s_mic_init) {
        vTaskDelay(pdMS_TO_TICKS(50));
        es7243e_init();
        s_mic_init = true;
    }

    xStreamBufferReset(s_ring_buf);
    s_pre_fill = 0;
    s_pre_write = 0;
    s_speech_chunks = 0;
    xEventGroupClearBits(s_vad_events, VAD_EVT_SPEECH | VAD_EVT_SILENCE);

    s_conv_state = CONV_LISTENING;
    s_reader_running = true;
    xTaskCreateStaticPinnedToCore(i2s_reader_task, "i2s_rd",
        READER_STACK_WORDS, NULL, 5,
        s_reader_stack, &s_reader_tcb, 0);

    xEventGroupSetBits(g_events, EVT_CONV_LISTENING);
    display_set_state(DISPLAY_STATE_WIFI_OK, "Listening...");
    ESP_LOGI(TAG, "Listening for speech...");
}

static void stop_listening(void)
{
    s_reader_running = false;
    vTaskDelay(pdMS_TO_TICKS(300));
    i2s_rx_stop();
    xEventGroupClearBits(g_events, EVT_CONV_LISTENING);
}

// ── Record and send via persistent WebSocket ────────────────────────────────

static bool conv_record_and_send(void)
{
    size_t pre_len = pre_buf_drain_to_ring();
    ESP_LOGI(TAG, "Speech detected! %zu pre-speech bytes flushed", pre_len);

    s_conv_state = CONV_RECORDING;
    xEventGroupClearBits(g_events, EVT_CONV_LISTENING);
    xEventGroupSetBits(g_events, EVT_AUDIO_RECORDING);

    xEventGroupClearBits(s_vad_events, VAD_EVT_SILENCE);
    xTimerReset(s_silence_timer, 0);
    xTimerStart(s_silence_timer, 0);

    ESP_LOGI(TAG, "Free internal heap: %lu B",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    if (!s_ws_connected) {
        ESP_LOGW(TAG, "Stream-recorder not connected, waiting...");
        for (int i = 0; i < 80 && !s_ws_connected; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!s_ws_connected) {
            ESP_LOGE(TAG, "Stream-recorder connect timeout");
            xTimerStop(s_silence_timer, 0);
            stop_listening();
            audio_speaker_unmute();
            xEventGroupClearBits(g_events, EVT_AUDIO_RECORDING);
            return false;
        }
    }

    const char *start_msg = "{\"type\":\"recording_start\"}";
    esp_websocket_client_send_text(s_ws_client, start_msg, strlen(start_msg),
                                    pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Sent recording_start");

    bool header_sent  = false;
    bool ws_ok        = true;
    bool touch_exit   = false;
    size_t total_mono = 0;
    size_t max_mono   = RECORD_MAX_S * SAMPLE_RATE * 2;

    while (ws_ok && total_mono < max_mono) {
        EventBits_t vad = xEventGroupGetBits(s_vad_events);
        if (vad & VAD_EVT_SILENCE) {
            ESP_LOGI(TAG, "Silence timeout — stopping recording");
            break;
        }

        // Touch screen or knob → exit conversation
        if (screen_tapped()) {
            wait_touch_release();
            touch_exit = true;
            break;
        }
        if (knob_btn_pressed()) {
            while (knob_btn_pressed()) vTaskDelay(pdMS_TO_TICKS(30));
            touch_exit = true;
            break;
        }

        if (!s_ws_connected) {
            ESP_LOGE(TAG, "WS disconnected during recording");
            ws_ok = false;
            break;
        }

        if (!header_sent) {
            wav_hdr_t hdr;
            build_wav_header(&hdr, max_mono);
            esp_websocket_client_send_bin(s_ws_client, (const char *)&hdr,
                sizeof(wav_hdr_t), pdMS_TO_TICKS(5000));
            header_sent = true;
        }

        size_t got = xStreamBufferReceive(s_ring_buf, s_send_buf,
                                           SEND_CHUNK, pdMS_TO_TICKS(100));
        if (got > 0) {
            int ret = esp_websocket_client_send_bin(s_ws_client,
                (const char *)s_send_buf, got, pdMS_TO_TICKS(5000));
            if (ret < 0) { ws_ok = false; break; }
            total_mono += got;
        }
    }

    xTimerStop(s_silence_timer, 0);
    stop_listening();
    audio_speaker_unmute();
    xEventGroupClearBits(g_events, EVT_AUDIO_RECORDING);

    // Drain remaining ring buffer
    if (ws_ok && s_ws_connected) {
        size_t got;
        while ((got = xStreamBufferReceive(s_ring_buf, s_send_buf, SEND_CHUNK, 0)) > 0) {
            esp_websocket_client_send_bin(s_ws_client,
                (const char *)s_send_buf, got, pdMS_TO_TICKS(5000));
            total_mono += got;
        }
    }

    if (ws_ok && s_ws_connected) {
        const char *end_msg = "{\"type\":\"recording_end\"}";
        esp_websocket_client_send_text(s_ws_client, end_msg, strlen(end_msg),
                                        pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Sent recording_end");
    }

    float dur = (float)total_mono / (SAMPLE_RATE * 2);
    ESP_LOGI(TAG, "Conv streamed %.1f s (%zu B mono)", dur, total_mono);

    if (touch_exit) {
        xEventGroupClearBits(g_events, EVT_CONV_MODE);
        return false;
    }

    if (total_mono < SAMPLE_RATE) {
        ESP_LOGW(TAG, "Too short (%.1f s), likely noise", dur);
        return false;
    }

    return true;
}

// ── Sleep mode (knob toggles) ───────────────────────────────────────────────

static void enter_sleep_mode(void)
{
    ESP_LOGI(TAG, "Entering sleep mode");

    // Stop conversation if active
    if (xEventGroupGetBits(g_events) & EVT_CONV_MODE) {
        xEventGroupClearBits(g_events, EVT_CONV_MODE);
        if (s_reader_running) stop_listening();
    }

    // Stop audio playback
    audio_stop();
    audio_speaker_mute();

    // Disconnect all connections
    stream_player_pause();
    ws_recorder_disconnect();  // in case conversation was active
    mqtt_stop();

    xEventGroupSetBits(g_events, EVT_DEEP_SLEEP);
    display_sleep();
}

static void exit_sleep_mode(void)
{
    ESP_LOGI(TAG, "Exiting sleep mode");
    xEventGroupClearBits(g_events, EVT_DEEP_SLEEP);
    display_wake();

    // Reconnect ready-mode connections (recorder connects during conversation)
    mqtt_reconnect();
    stream_player_resume();

    restore_idle_display();
}

// ── Conversation mode (touch toggles) ───────────────────────────────────────

static void enter_conversation_mode(void)
{
    ESP_LOGI(TAG, "Entering conversation mode");
    xEventGroupSetBits(g_events, EVT_CONV_MODE);

    // Enable audio + connect stream-recorder for conversation
    audio_speaker_unmute();
    ws_recorder_connect();

    start_listening();
    TickType_t listen_start = xTaskGetTickCount();

    while (xEventGroupGetBits(g_events) & EVT_CONV_MODE) {

        // Knob press → exit conversation + enter sleep immediately
        if (knob_btn_pressed()) {
            while (knob_btn_pressed()) vTaskDelay(pdMS_TO_TICKS(30));
            ESP_LOGI(TAG, "Knob pressed — exiting conversation + sleep");
            // Stop incoming data FIRST to prevent decode task restart
            stream_player_pause();
            audio_stop();
            if (s_reader_running) stop_listening();
            else i2s_rx_stop();
            for (int i = 0; i < 10; i++) {
                if (!(xEventGroupGetBits(g_events) & EVT_AUDIO_PLAYING)) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            audio_speaker_mute();
            ws_recorder_disconnect();
            xEventGroupClearBits(g_events,
                EVT_CONV_MODE | EVT_CONV_LISTENING | EVT_AUDIO_RECORDING |
                EVT_STREAM_PLAYING);
            xTimerStop(s_silence_timer, 0);
            s_conv_state = CONV_OFF;
            // Go directly to sleep — display off immediately
            mqtt_stop();
            xEventGroupSetBits(g_events, EVT_DEEP_SLEEP);
            display_sleep();
            ESP_LOGI(TAG, "Sleeping");
            return;  // exit enter_conversation_mode entirely
        }

        switch (s_conv_state) {

        case CONV_LISTENING: {
            // Touch screen → exit conversation
            if (screen_tapped()) {
                wait_touch_release();
                ESP_LOGI(TAG, "Touch during listening — exiting conversation");
                goto exit_conv;
            }

            // Listen timeout
            if ((xTaskGetTickCount() - listen_start) >
                pdMS_TO_TICKS(LISTEN_TIMEOUT_S * 1000)) {
                ESP_LOGI(TAG, "Listen timeout, exiting conversation mode");
                goto exit_conv;
            }

            // Wait for VAD speech event
            EventBits_t vad = xEventGroupWaitBits(s_vad_events, VAD_EVT_SPEECH,
                pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
            if (!(vad & VAD_EVT_SPEECH)) break;

            bool sent = conv_record_and_send();
            if (!sent) {
                if (!(xEventGroupGetBits(g_events) & EVT_CONV_MODE)) {
                    goto exit_conv;
                }
                start_listening();
                listen_start = xTaskGetTickCount();
                break;
            }

            s_conv_state = CONV_WAITING;
            s_wait_start = xTaskGetTickCount();
            display_set_state(DISPLAY_STATE_WIFI_OK, "");
            ESP_LOGI(TAG, "Waiting for server response...");
            break;
        }

        case CONV_WAITING: {
            // Touch screen → exit conversation
            if (screen_tapped()) {
                wait_touch_release();
                ESP_LOGI(TAG, "Touch during waiting — exiting conversation");
                goto exit_conv;
            }

            EventBits_t bits = xEventGroupGetBits(g_events);
            if (bits & EVT_AUDIO_PLAYING) {
                s_conv_state = CONV_PLAYING;
                ESP_LOGI(TAG, "Response playing");
                break;
            }

            if ((xTaskGetTickCount() - s_wait_start) >
                pdMS_TO_TICKS(WAIT_RESPONSE_TIMEOUT_S * 1000)) {
                ESP_LOGW(TAG, "Response timeout, resuming listening");
                start_listening();
                listen_start = xTaskGetTickCount();
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }

        case CONV_PLAYING: {
            // Touch screen → stop playback and exit
            if (screen_tapped()) {
                audio_stop();
                wait_touch_release();
                for (int i = 0; i < 20; i++) {
                    if (!(xEventGroupGetBits(g_events) & EVT_AUDIO_PLAYING)) break;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                goto exit_conv;
            }

            EventBits_t bits = xEventGroupGetBits(g_events);
            if (!(bits & EVT_AUDIO_PLAYING)) {
                ESP_LOGI(TAG, "Playback done, reconnecting stream-player");

                // Always force a clean reconnect after playback to avoid
                // stale/dirty WS connections that drop TTS for the next round
                stream_player_pause();
                stream_player_resume();

                xEventGroupSetBits(g_events, EVT_CONV_LISTENING);
                vTaskDelay(pdMS_TO_TICKS(200));
                start_listening();
                listen_start = xTaskGetTickCount();
            }

            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }

        default:
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }
    }

exit_conv:
    // Pause stream-player FIRST to stop incoming data and prevent decode restart
    stream_player_pause();

    if (s_reader_running) {
        stop_listening();
    } else {
        i2s_rx_stop();
    }

    audio_stop();
    for (int i = 0; i < 10; i++) {
        if (!(xEventGroupGetBits(g_events) & EVT_AUDIO_PLAYING)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    audio_speaker_mute();
    ws_recorder_disconnect();
    xEventGroupClearBits(g_events,
        EVT_CONV_MODE | EVT_CONV_LISTENING | EVT_AUDIO_RECORDING |
        EVT_STREAM_PLAYING);
    xTimerStop(s_silence_timer, 0);
    s_conv_state = CONV_OFF;

    // Reconnect stream-player for ready mode
    stream_player_resume();

    restore_idle_display();
    ESP_LOGI(TAG, "Exited conversation mode");
}

// ── Record task ──────────────────────────────────────────────────────────────

static void record_task(void *arg)
{
    touch_init();
    knob_init();

    s_ring_storage = heap_caps_malloc(RING_BUF_BYTES + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_send_buf     = heap_caps_malloc(SEND_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_reader_stack = heap_caps_malloc(READER_STACK_WORDS * sizeof(StackType_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_pre_buf      = heap_caps_malloc(PRE_SPEECH_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring_storage || !s_send_buf || !s_reader_stack || !s_pre_buf) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        vTaskDelete(NULL);
        return;
    }
    s_ring_buf = xStreamBufferCreateStatic(RING_BUF_BYTES, 1, s_ring_storage, &s_ring_struct);

    s_vad_events = xEventGroupCreate();
    s_silence_timer = xTimerCreate("silence", pdMS_TO_TICKS(VAD_SILENCE_TIMEOUT_MS),
                                    pdFALSE, NULL, silence_timer_cb);

    // Wait for prerequisites
    xEventGroupWaitBits(g_events, EVT_DOLL_READY | EVT_IMAGES_DONE,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    // Connect stream-player (always on in ready mode for incoming TTS)
    // Stream-recorder connects only during conversation
    stream_player_resume();

    // Ready mode: mic and speaker OFF — only enabled during conversation
    audio_speaker_mute();

    // Discard any spurious touch events from boot/init
    vTaskDelay(pdMS_TO_TICKS(500));
    while (screen_tapped()) vTaskDelay(pdMS_TO_TICKS(30));

    ESP_LOGI(TAG, "Ready — touch screen to talk, knob to sleep");

    while (1) {
        // ── Knob press → toggle sleep mode ──────────────────────────
        if (knob_btn_pressed()) {
            while (knob_btn_pressed()) vTaskDelay(pdMS_TO_TICKS(30));

            EventBits_t bits = xEventGroupGetBits(g_events);
            if (bits & EVT_DEEP_SLEEP) {
                exit_sleep_mode();
            } else {
                enter_sleep_mode();
            }
            continue;
        }

        // Skip everything while sleeping
        if (xEventGroupGetBits(g_events) & EVT_DEEP_SLEEP) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ── Touch screen → toggle conversation mode ─────────────────
        if (screen_tapped()) {
            wait_touch_release();

            EventBits_t bits = xEventGroupGetBits(g_events);
            if (bits & EVT_AUDIO_PLAYING) continue;
            if (strlen(g_config.chat_id) == 0) {
                ESP_LOGW(TAG, "No chat linked, ignoring touch");
                continue;
            }

            if (bits & EVT_CONV_MODE) {
                xEventGroupClearBits(g_events, EVT_CONV_MODE);
                continue;
            }

            enter_conversation_mode();
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

static StaticTask_t s_record_tcb;

void record_init(void)
{
    StackType_t *stack = heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(stack);
    xTaskCreateStaticPinnedToCore(record_task, "record",
        8192 / sizeof(StackType_t), NULL, 4, stack, &s_record_tcb, 1);
    ESP_LOGI(TAG, "Record task spawned");
}
