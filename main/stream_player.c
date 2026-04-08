#include "stream_player.h"
#include "audio.h"
#include "config.h"
#include "events.h"
#include "display.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include <string.h>

static const char *TAG = "stream_player";

// ── Stream buffer: WS handler (producer) → decode task (consumer) ───────────

#define SP_STREAM_BUF_SIZE  (512 * 1024)  // 512 KB — ~10.7 s of PCM at 24 kHz 16-bit mono

static StreamBufferHandle_t  s_sp_stream;
static StaticStreamBuffer_t  s_sp_stream_struct;
static uint8_t              *s_sp_stream_storage;

// ── Current stream state ────────────────────────────────────────────────────

static volatile bool s_stream_active = false;   // between tts_start and tts_end
static volatile bool s_stream_error  = false;   // tts_error received
static char          s_current_msg_id[80];

// WS client handle (module-level for pause/resume)
static esp_websocket_client_handle_t s_ws_client = NULL;

// ── Text frame accumulation (for fragmented frames) ─────────────────────────

#define SP_TEXT_BUF_SIZE  512
static char s_text_buf[SP_TEXT_BUF_SIZE];
static int  s_text_buf_len = 0;

// ── URL builder (https→wss, same pattern as record.c) ───────────────────────

static void build_ws_player_url(char *url, size_t url_size)
{
    if (strncmp(g_config.stream_player_url, "https://", 8) == 0) {
        snprintf(url, url_size, "wss://%s/ws-player?chatId=%s&auth=%s",
                 g_config.stream_player_url + 8,
                 g_config.chat_id, g_config.apikey);
    } else if (strncmp(g_config.stream_player_url, "http://", 7) == 0) {
        snprintf(url, url_size, "ws://%s/ws-player?chatId=%s&auth=%s",
                 g_config.stream_player_url + 7,
                 g_config.chat_id, g_config.apikey);
    } else {
        snprintf(url, url_size, "ws://%s/ws-player?chatId=%s&auth=%s",
                 g_config.stream_player_url,
                 g_config.chat_id, g_config.apikey);
    }
}

// ── Text frame handler (JSON control messages) ──────────────────────────────

static void handle_text_frame(const char *json_str, int len)
{
    cJSON *json = cJSON_ParseWithLength(json_str, len);
    if (!json) {
        ESP_LOGW(TAG, "Failed to parse text frame");
        return;
    }

    const char *type = cJSON_GetStringValue(cJSON_GetObjectItem(json, "type"));
    if (!type) { cJSON_Delete(json); return; }

    if (strcmp(type, "tts_start") == 0) {
        const char *mid = cJSON_GetStringValue(cJSON_GetObjectItem(json, "messageId"));
        if (mid) {
            strlcpy(s_current_msg_id, mid, sizeof(s_current_msg_id));
            s_stream_active = true;
            s_stream_error  = false;
            xStreamBufferReset(s_sp_stream);
            xEventGroupSetBits(g_events, EVT_STREAM_PLAYING);
            ESP_LOGI(TAG, "tts_start: %.36s", mid);
        }
    } else if (strcmp(type, "tts_end") == 0) {
        ESP_LOGI(TAG, "tts_end: %.36s", s_current_msg_id);
        s_stream_active = false;
    } else if (strcmp(type, "tts_error") == 0) {
        const char *err = cJSON_GetStringValue(cJSON_GetObjectItem(json, "error"));
        ESP_LOGE(TAG, "tts_error: %.36s — %s", s_current_msg_id,
                 err ? err : "unknown");
        s_stream_active = false;
        s_stream_error  = true;
    }

    cJSON_Delete(json);
}

// ── WebSocket event handler ─────────────────────────────────────────────────

static void sp_ws_event_handler(void *arg, esp_event_base_t base,
                                 int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to stream-player");
        xEventGroupSetBits(g_events, EVT_STREAM_CONNECTED);
        display_set_ws_status(true, false);  // player dot on
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from stream-player");
        xEventGroupClearBits(g_events, EVT_STREAM_CONNECTED);
        display_set_ws_status(false, false);  // player dot off
        if (s_stream_active) {
            s_stream_active = false;
            s_stream_error  = true;
        }
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x01) {
            // Text frame — accumulate fragments
            if (data->payload_offset == 0) {
                s_text_buf_len = 0;
            }
            int copy = data->data_len;
            if (s_text_buf_len + copy > SP_TEXT_BUF_SIZE - 1) {
                copy = SP_TEXT_BUF_SIZE - 1 - s_text_buf_len;
            }
            if (copy > 0) {
                memcpy(s_text_buf + s_text_buf_len, data->data_ptr, copy);
                s_text_buf_len += copy;
            }
            // Process when final fragment arrives
            if (data->payload_offset + data->data_len >= data->payload_len) {
                s_text_buf[s_text_buf_len] = '\0';
                handle_text_frame(s_text_buf, s_text_buf_len);
                s_text_buf_len = 0;
            }
        } else if (data->op_code == 0x02) {
            // Binary frame — raw PCM audio chunk (16-bit, 24 kHz, mono)
            if (s_stream_active && data->data_len > 0) {
                size_t sent = xStreamBufferSend(s_sp_stream,
                    (const uint8_t *)data->data_ptr, data->data_len,
                    0);  // non-blocking — never stall WS client task
                if ((int)sent < data->data_len) {
                    ESP_LOGW(TAG, "Stream buffer overflow, lost %d bytes",
                             data->data_len - (int)sent);
                }
            }
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WebSocket error");
        break;

    default:
        break;
    }
}

// ── Playback task — reads PCM from stream buffer, expands + plays ───────────

static void sp_decode_task(void *arg)
{
    while (1) {
        // Block until data arrives in the stream buffer
        uint8_t peek;
        size_t got = xStreamBufferReceive(s_sp_stream, &peek, 1,
                                           pdMS_TO_TICKS(5000));
        if (got == 0) continue;

        // Check guards — only play during conversation mode
        EventBits_t bits = xEventGroupGetBits(g_events);
        if ((bits & EVT_AUDIO_RECORDING) || !(bits & EVT_CONV_MODE)) {
            if (bits & EVT_AUDIO_RECORDING) {
                ESP_LOGW(TAG, "Recording in progress, discarding stream");
            } else {
                ESP_LOGW(TAG, "Not in conversation mode, discarding stream");
            }
            while (s_stream_active || xStreamBufferBytesAvailable(s_sp_stream) > 0) {
                uint8_t discard[256];
                xStreamBufferReceive(s_sp_stream, discard, sizeof(discard),
                                      pdMS_TO_TICKS(100));
            }
            xEventGroupClearBits(g_events, EVT_STREAM_PLAYING);
            continue;
        }

        ESP_LOGI(TAG, "Starting stream playback for %.36s", s_current_msg_id);
        audio_stream_play(s_sp_stream, &s_stream_active, &s_stream_error, peek);
        xEventGroupClearBits(g_events, EVT_STREAM_PLAYING);
        ESP_LOGI(TAG, "Stream playback finished for %.36s", s_current_msg_id);
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

void stream_player_init(void)
{
    // Allocate stream buffer in PSRAM
    s_sp_stream_storage = heap_caps_malloc(SP_STREAM_BUF_SIZE + 1,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_sp_stream_storage);
    s_sp_stream = xStreamBufferCreateStatic(SP_STREAM_BUF_SIZE, 1,
                                             s_sp_stream_storage,
                                             &s_sp_stream_struct);

    // Start decode/playback task (runs forever, waits for data in stream buffer)
    static StaticTask_t s_dec_tcb;
    StackType_t *dec_stack = heap_caps_malloc(32768,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(dec_stack);
    xTaskCreateStaticPinnedToCore(sp_decode_task, "sp_decode",
        32768 / sizeof(StackType_t), NULL, 5, dec_stack, &s_dec_tcb, 0);

    // WS not connected yet — stream_player_resume() connects during conversation
    ESP_LOGI(TAG, "Stream-player module initialized (disconnected)");
}

void stream_player_pause(void)
{
    // Clear stream state so decode task doesn't replay stale data
    s_stream_active = false;
    s_stream_error  = true;   // signal decode task to abort current playback
    xStreamBufferReset(s_sp_stream);

    if (s_ws_client) {
        esp_websocket_client_stop(s_ws_client);
        esp_websocket_client_destroy(s_ws_client);
        s_ws_client = NULL;
        ESP_LOGI(TAG, "Paused (destroyed WS client, flushed stream buffer)");
    }
}

void stream_player_resume(void)
{
    if (s_ws_client) return;              // already running
    if (strlen(g_config.chat_id) == 0) return;

    char url[384];
    build_ws_player_url(url, sizeof(url));

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
                                   sp_ws_event_handler, NULL);
    esp_websocket_client_start(s_ws_client);
    ESP_LOGI(TAG, "Resumed (recreated WS client, reconnecting)");
}
