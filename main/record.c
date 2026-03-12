#include "record.h"
#include "audio.h"
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
    // Chip at 0x14 is ES7243E (ID 0x7A43), NOT plain ES7243.
    // Sequence from ESP-ADF es7243e driver — paged register map.
    es7243_write(0x01, 0x3A);
    es7243_write(0x00, 0x80);   // Reset all registers
    vTaskDelay(pdMS_TO_TICKS(10));
    es7243_write(0xF9, 0x00);   // Select page 0
    es7243_write(0x04, 0x02);
    es7243_write(0x04, 0x01);
    es7243_write(0xF9, 0x01);   // Select page 1
    es7243_write(0x00, 0x1E);
    es7243_write(0x01, 0x00);
    es7243_write(0x02, 0x00);
    es7243_write(0x03, 0x20);
    es7243_write(0x04, 0x01);
    es7243_write(0x0D, 0x00);
    es7243_write(0x05, 0x00);
    es7243_write(0x06, 0x03);   // SCLK = MCLK / 4
    es7243_write(0x07, 0x00);   // LRCK = MCLK / 256 (high byte)
    es7243_write(0x08, 0xFF);   // LRCK = MCLK / 256 (low byte)
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
    es7243_write(0x20, 0x1A);  // MIC PGA gain +30 dB
    es7243_write(0x21, 0x1A);  // MIC PGA gain +30 dB
    es7243_write(0x00, 0x80);  // Slave mode, enable
    es7243_write(0x01, 0x3A);
    es7243_write(0x16, 0x3F);
    es7243_write(0x16, 0x00);
    ESP_LOGI(TAG, "ES7243E init done (addr=0x%02X, chip ID 0x7A43)", ES7243_ADDR);
}

// ── Knob button ──────────────────────────────────────────────────────────────

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

// ── WebSocket event helpers ──────────────────────────────────────────────────

static EventGroupHandle_t              s_ws_events       = NULL;
static esp_websocket_client_handle_t   s_preconnect_client = NULL;

#define WS_EVT_CONNECTED   (1 << 0)
#define WS_EVT_CLOSED      (1 << 1)
#define WS_EVT_ERROR       (1 << 2)

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t event_id, void *event_data)
{
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WS connected");
        xEventGroupSetBits(s_ws_events, WS_EVT_CONNECTED);
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "WS disconnected");
        xEventGroupSetBits(s_ws_events, WS_EVT_CLOSED);
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WS error");
        xEventGroupSetBits(s_ws_events, WS_EVT_ERROR);
        break;
    default:
        break;
    }
}

// ── Display helper ────────────────────────────────────────────────────────────

static void restore_idle_display(void)
{
    const char *msg = strlen(g_config.chat_id) > 0 ? "" : "No chat linked";
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

    // Send in two parts if wrapping
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
    // Keep lightweight — timer service task has a small stack (2 KB)
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

        // Compute RMS for VAD and LED level metering
        uint16_t rms = compute_rms(s, n_mono);
        g_audio_rms = rms;
        bool loud = (rms > VAD_RMS_THRESHOLD);

        conv_state_t state = s_conv_state;

        if (state == CONV_LISTENING) {
            // Write to pre-speech circular buffer (overwrites oldest)
            pre_buf_write(buf, mono_bytes);

            // Track consecutive loud chunks for speech onset
            if (loud) {
                s_speech_chunks++;
                if (s_speech_chunks >= VAD_CONFIRM_CHUNKS) {
                    xEventGroupSetBits(s_vad_events, VAD_EVT_SPEECH);
                }
            } else {
                s_speech_chunks = 0;
            }
        } else if (state == CONV_RECORDING) {
            // Write to ring buffer for WS streaming
            xStreamBufferSend(s_ring_buf, buf, mono_bytes, pdMS_TO_TICKS(50));

            // Reset silence timer on loud chunks
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

// ── WebSocket URL builder ────────────────────────────────────────────────────

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

// ── WebSocket pre-connect helpers ────────────────────────────────────────────

static void ws_preconnect_start(void)
{
    if (s_preconnect_client) return;  // already connecting

    if (s_ws_events) {
        vEventGroupDelete(s_ws_events);
    }
    s_ws_events = xEventGroupCreate();

    char url[384];
    build_ws_url(url, sizeof(url));

    esp_websocket_client_config_t ws_cfg = {
        .uri        = url,
        .task_stack = 8192,
    };
    if (strncmp(url, "wss://", 6) == 0) {
        ws_cfg.crt_bundle_attach = esp_crt_bundle_attach;
    }

    s_preconnect_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(s_preconnect_client, WEBSOCKET_EVENT_ANY,
                                   ws_event_handler, NULL);
    esp_websocket_client_start(s_preconnect_client);
    ESP_LOGI(TAG, "WebSocket pre-connecting...");
}

static void ws_preconnect_cancel(void)
{
    if (!s_preconnect_client) return;
    esp_websocket_client_stop(s_preconnect_client);
    esp_websocket_client_destroy(s_preconnect_client);
    s_preconnect_client = NULL;
    if (s_ws_events) {
        vEventGroupDelete(s_ws_events);
        s_ws_events = NULL;
    }
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

// ── Stop listening (I2S RX + reader task) ────────────────────────────────────

static void stop_listening(void)
{
    s_reader_running = false;
    vTaskDelay(pdMS_TO_TICKS(300));  // wait for reader task to exit
    i2s_rx_stop();
    xEventGroupClearBits(g_events, EVT_CONV_LISTENING);
}

// ── Record and send via WebSocket ────────────────────────────────────────────
// Returns true if a meaningful recording was sent, false if too short / error.

static bool conv_record_and_send(void)
{
    // Flush pre-speech buffer into ring buffer
    size_t pre_len = pre_buf_drain_to_ring();
    ESP_LOGI(TAG, "Speech detected! %zu pre-speech bytes flushed", pre_len);

    s_conv_state = CONV_RECORDING;
    xEventGroupClearBits(g_events, EVT_CONV_LISTENING);
    xEventGroupSetBits(g_events, EVT_AUDIO_RECORDING);

    // Start silence timer
    xEventGroupClearBits(s_vad_events, VAD_EVT_SILENCE);
    xTimerReset(s_silence_timer, 0);
    xTimerStart(s_silence_timer, 0);

    ESP_LOGI(TAG, "Free internal heap: %lu B",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    // Use pre-connected WebSocket if available, otherwise connect now
    esp_websocket_client_handle_t client;
    if (s_preconnect_client) {
        client = s_preconnect_client;
        s_preconnect_client = NULL;
        ESP_LOGI(TAG, "Using pre-connected WebSocket");
    } else {
        char url[384];
        build_ws_url(url, sizeof(url));
        if (s_ws_events) vEventGroupDelete(s_ws_events);
        s_ws_events = xEventGroupCreate();
        esp_websocket_client_config_t ws_cfg = {
            .uri        = url,
            .task_stack = 8192,
        };
        if (strncmp(url, "wss://", 6) == 0) {
            ws_cfg.crt_bundle_attach = esp_crt_bundle_attach;
        }
        client = esp_websocket_client_init(&ws_cfg);
        esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY,
                                       ws_event_handler, NULL);
        esp_websocket_client_start(client);
        ESP_LOGI(TAG, "WebSocket connecting now (no pre-connect available)");
    }

    bool ws_connected = false;
    bool header_sent  = false;
    bool ws_ok        = true;
    bool knob_exit    = false;
    size_t total_mono = 0;
    size_t max_mono   = RECORD_MAX_S * SAMPLE_RATE * 2;

    // Stream loop: runs until silence timeout, knob press, or max duration
    while (ws_ok && total_mono < max_mono) {
        // Check silence timer
        EventBits_t vad = xEventGroupGetBits(s_vad_events);
        if (vad & VAD_EVT_SILENCE) {
            ESP_LOGI(TAG, "Silence timeout — stopping recording");
            break;
        }

        // Check knob press (exit conversation)
        if (knob_btn_pressed()) {
            knob_exit = true;
            break;
        }

        if (!ws_connected) {
            EventBits_t bits = xEventGroupWaitBits(s_ws_events,
                WS_EVT_CONNECTED | WS_EVT_ERROR, pdTRUE, pdFALSE, 0);
            if (bits & WS_EVT_ERROR) { ws_ok = false; break; }
            if (bits & WS_EVT_CONNECTED) {
                ws_connected = true;
                size_t prebuf = xStreamBufferBytesAvailable(s_ring_buf);
                ESP_LOGI(TAG, "WS connected, %zu bytes buffered (%.1f s)",
                         prebuf, (float)prebuf / (SAMPLE_RATE * 2));
            }
        }

        if (!ws_connected) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!header_sent) {
            wav_hdr_t hdr;
            build_wav_header(&hdr, max_mono);
            esp_websocket_client_send_bin(client, (const char *)&hdr,
                sizeof(wav_hdr_t), pdMS_TO_TICKS(5000));
            header_sent = true;
        }

        size_t got = xStreamBufferReceive(s_ring_buf, s_send_buf,
                                           SEND_CHUNK, pdMS_TO_TICKS(100));
        if (got > 0) {
            int ret = esp_websocket_client_send_bin(client,
                (const char *)s_send_buf, got, pdMS_TO_TICKS(5000));
            if (ret < 0) { ws_ok = false; break; }
            total_mono += got;
        }
    }

    // Stop reader + I2S RX (free bus for playback)
    xTimerStop(s_silence_timer, 0);
    stop_listening();
    audio_speaker_unmute();
    xEventGroupClearBits(g_events, EVT_AUDIO_RECORDING);

    // If WS never connected, try waiting
    if (!ws_connected && !knob_exit) {
        EventBits_t bits = xEventGroupWaitBits(s_ws_events,
            WS_EVT_CONNECTED | WS_EVT_ERROR,
            pdTRUE, pdFALSE, pdMS_TO_TICKS(8000));
        ws_connected = !!(bits & WS_EVT_CONNECTED);
        if (ws_connected) {
            size_t remaining = xStreamBufferBytesAvailable(s_ring_buf);
            wav_hdr_t hdr;
            build_wav_header(&hdr, remaining);
            esp_websocket_client_send_bin(client, (const char *)&hdr,
                sizeof(wav_hdr_t), pdMS_TO_TICKS(5000));
        }
    }

    // Drain remaining ring buffer
    if (ws_connected) {
        size_t got;
        while ((got = xStreamBufferReceive(s_ring_buf, s_send_buf, SEND_CHUNK, 0)) > 0) {
            esp_websocket_client_send_bin(client,
                (const char *)s_send_buf, got, pdMS_TO_TICKS(5000));
            total_mono += got;
        }
    }

    float dur = (float)total_mono / (SAMPLE_RATE * 2);
    ESP_LOGI(TAG, "Conv streamed %.1f s (%zu B mono)", dur, total_mono);

    if (!ws_connected) {
        ESP_LOGE(TAG, "WS connect failed");
    }
    esp_websocket_client_stop(client);
    esp_websocket_client_destroy(client);
    vEventGroupDelete(s_ws_events);
    s_ws_events = NULL;

    // If knob was pressed, signal caller to exit conversation mode
    if (knob_exit) {
        while (knob_btn_pressed()) vTaskDelay(pdMS_TO_TICKS(30));
        xEventGroupClearBits(g_events, EVT_CONV_MODE);
        return false;
    }

    // Too-short recording = noise, go back to listening
    if (total_mono < SAMPLE_RATE) {
        ESP_LOGW(TAG, "Too short (%.1f s), likely noise", dur);
        return false;
    }

    return true;
}

// ── Conversation mode ────────────────────────────────────────────────────────

static void enter_conversation_mode(void)
{
    ESP_LOGI(TAG, "Entering conversation mode");
    xEventGroupSetBits(g_events, EVT_CONV_MODE);

    start_listening();
    ws_preconnect_start();  // begin WS handshake while waiting for speech onset
    TickType_t listen_start = xTaskGetTickCount();

    while (xEventGroupGetBits(g_events) & EVT_CONV_MODE) {

        switch (s_conv_state) {

        case CONV_LISTENING: {
            // Check knob press → exit
            if (knob_btn_pressed()) {
                while (knob_btn_pressed()) vTaskDelay(pdMS_TO_TICKS(30));
                goto exit_conv;
            }

            // Check listen timeout
            if ((xTaskGetTickCount() - listen_start) >
                pdMS_TO_TICKS(LISTEN_TIMEOUT_S * 1000)) {
                ESP_LOGI(TAG, "Listen timeout, exiting conversation mode");
                goto exit_conv;
            }

            // Wait for VAD speech event
            EventBits_t vad = xEventGroupWaitBits(s_vad_events, VAD_EVT_SPEECH,
                pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
            if (!(vad & VAD_EVT_SPEECH)) break;

            // Speech detected → record and send
            bool sent = conv_record_and_send();
            if (!sent) {
                // Noise or knob exit or error
                if (!(xEventGroupGetBits(g_events) & EVT_CONV_MODE)) {
                    goto exit_conv;  // knob_exit cleared EVT_CONV_MODE
                }
                // Restart listening after noise/error
                start_listening();
                ws_preconnect_start();
                listen_start = xTaskGetTickCount();
                break;
            }

            // Recording sent successfully → wait for response
            s_conv_state = CONV_WAITING;
            s_wait_start = xTaskGetTickCount();
            ws_preconnect_start();  // pre-connect for next recording round
            display_set_state(DISPLAY_STATE_WIFI_OK, "");
            ESP_LOGI(TAG, "Waiting for server response...");
            break;
        }

        case CONV_WAITING: {
            // Check knob press → exit
            if (knob_btn_pressed()) {
                while (knob_btn_pressed()) vTaskDelay(pdMS_TO_TICKS(30));
                goto exit_conv;
            }

            // Check if playback started
            EventBits_t bits = xEventGroupGetBits(g_events);
            if (bits & EVT_AUDIO_PLAYING) {
                s_conv_state = CONV_PLAYING;
                ESP_LOGI(TAG, "Response playing");
                break;
            }

            // Timeout waiting for response
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
            // Check knob press → stop playback and exit
            if (knob_btn_pressed()) {
                audio_stop();
                while (knob_btn_pressed()) vTaskDelay(pdMS_TO_TICKS(30));
                // Wait for playback to actually stop
                for (int i = 0; i < 20; i++) {
                    if (!(xEventGroupGetBits(g_events) & EVT_AUDIO_PLAYING)) break;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                goto exit_conv;
            }

            // Check if playback finished
            EventBits_t bits = xEventGroupGetBits(g_events);
            if (!(bits & EVT_AUDIO_PLAYING)) {
                ESP_LOGI(TAG, "Playback done, resuming listening");
                xEventGroupSetBits(g_events, EVT_CONV_LISTENING);  // immediately show red, not white
                vTaskDelay(pdMS_TO_TICKS(200));  // brief pause
                start_listening();
                ws_preconnect_start();
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
    ws_preconnect_cancel();  // discard any pending pre-connect

    // Ensure reader/I2S stopped
    if (s_reader_running) {
        stop_listening();
    } else {
        i2s_rx_stop();  // might already be stopped
    }

    // Stop any ongoing playback
    if (xEventGroupGetBits(g_events) & EVT_AUDIO_PLAYING) {
        audio_stop();
    }

    audio_speaker_mute();
    xEventGroupClearBits(g_events,
        EVT_CONV_MODE | EVT_CONV_LISTENING | EVT_AUDIO_RECORDING);
    xTimerStop(s_silence_timer, 0);
    s_conv_state = CONV_OFF;
    restore_idle_display();
    ESP_LOGI(TAG, "Exited conversation mode");
}

// ── Record task ──────────────────────────────────────────────────────────────

static void record_task(void *arg)
{
    touch_init();
    knob_init();

    // Allocate buffers in PSRAM (once, reused across recordings)
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

    // VAD signaling
    s_vad_events = xEventGroupCreate();
    s_silence_timer = xTimerCreate("silence", pdMS_TO_TICKS(VAD_SILENCE_TIMEOUT_MS),
                                    pdFALSE, NULL, silence_timer_cb);

    ESP_LOGI(TAG, "Ready — conversation mode (VAD), streaming to: %s",
             g_config.stream_recorder_url);

    while (1) {
        // Wait for knob press
        while (!knob_btn_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        // Debounce: wait for release
        while (knob_btn_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        // Guard: skip if already playing or no chat linked
        EventBits_t bits = xEventGroupGetBits(g_events);
        if (bits & EVT_AUDIO_PLAYING) continue;
        if (strlen(g_config.chat_id) == 0) {
            ESP_LOGW(TAG, "No chat linked, ignoring knob press");
            continue;
        }

        // Toggle conversation mode
        if (bits & EVT_CONV_MODE) {
            // Already in conversation mode — signal exit
            xEventGroupClearBits(g_events, EVT_CONV_MODE);
            // enter_conversation_mode() will detect this and exit
            continue;
        }

        enter_conversation_mode();
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
