#include "mqtt.h"
#include "audio.h"
#include "config.h"
#include "events.h"
#include "display.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t s_client = NULL;
static char s_client_id[80]; // "doll_{doll_id}"

// ── Publish helpers ───────────────────────────────────────────────────────────

static void publish_connection_event(const char *status)
{
    cJSON *body = cJSON_CreateObject();
    cJSON_AddStringToObject(body, "clientId",   s_client_id);
    cJSON_AddStringToObject(body, "deviceType", "doll");
    cJSON_AddStringToObject(body, "deviceId",   g_config.doll_id);
    cJSON_AddStringToObject(body, "status",     status);
    char *payload = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);
    esp_mqtt_client_publish(s_client, "connections", payload, 0, 0, 0);
    display_mqtt_tx_pulse();
    free(payload);
}

// ── Incoming message handler ──────────────────────────────────────────────────

static void handle_action_event(const char *data, int data_len)
{
    cJSON *json = cJSON_ParseWithLength(data, data_len);
    if (!json) return;

    const char *type   = cJSON_GetStringValue(cJSON_GetObjectItem(json, "type"));
    const char *action = cJSON_GetStringValue(cJSON_GetObjectItem(json, "action"));

    if (type && action) {
        if (strcmp(type, "audio") == 0) {
            if (strcmp(action, "play") == 0 || strcmp(action, "replay") == 0) {
                const char *mid = cJSON_GetStringValue(cJSON_GetObjectItem(json, "messageId"));
                if (mid) {
                    if (xEventGroupGetBits(g_events) & EVT_AUDIO_RECORDING) {
                        ESP_LOGW(TAG, "Recording in progress, skipping play %.36s", mid);
                    } else {
                        ESP_LOGI(TAG, "Audio message arrived: %.36s", mid);
                        audio_play_message(mid);
                    }
                } else {
                    ESP_LOGW(TAG, "audio play missing messageId");
                }
            } else if (strcmp(action, "stop") == 0) {
                audio_stop();
            }
        } else if (strcmp(type, "system") == 0) {
            ESP_LOGI(TAG, "system action: %s", action);
            if (strcmp(action, "deepsleep") == 0) {
                xEventGroupSetBits(g_events, EVT_DEEP_SLEEP);
            } else if (strcmp(action, "restart") == 0) {
                esp_restart();
            }
        }
    }

    cJSON_Delete(json);
}

// ── MQTT event handler ────────────────────────────────────────────────────────

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to %s", g_config.mqtt_url);
        xEventGroupClearBits(g_events, EVT_MQTT_DISCONNECTED);
        xEventGroupSetBits(g_events, EVT_MQTT_CONNECTED);
        display_set_mqtt_connected(true);

        publish_connection_event("connected");

        // Subscribe to doll-level action events
        char action_topic[128];
        snprintf(action_topic, sizeof(action_topic),
                 "dolls/%s/actionEvents", g_config.doll_id);
        esp_mqtt_client_subscribe(s_client, action_topic, 0);
        ESP_LOGI(TAG, "Subscribed to %s", action_topic);

        // Subscribe to chat-level action events (audio play arrives here)
        if (strlen(g_config.chat_id) > 0) {
            char chat_topic[128];
            snprintf(chat_topic, sizeof(chat_topic),
                     "chats/%s/actionEvents", g_config.chat_id);
            esp_mqtt_client_subscribe(s_client, chat_topic, 0);
            ESP_LOGI(TAG, "Subscribed to %s", chat_topic);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected");
        xEventGroupClearBits(g_events, EVT_MQTT_CONNECTED);
        xEventGroupSetBits(g_events, EVT_MQTT_DISCONNECTED);
        display_set_mqtt_connected(false);
        break;

    case MQTT_EVENT_DATA: {
        // Copy topic to null-terminated buffer for comparison
        char topic[128] = {};
        int tlen = evt->topic_len < (int)sizeof(topic) - 1
                 ? evt->topic_len : (int)sizeof(topic) - 1;
        memcpy(topic, evt->topic, tlen);

        char doll_topic[128], chat_topic[128];
        snprintf(doll_topic, sizeof(doll_topic),
                 "dolls/%s/actionEvents", g_config.doll_id);
        snprintf(chat_topic, sizeof(chat_topic),
                 "chats/%s/actionEvents", g_config.chat_id);

        if (strcmp(topic, doll_topic) == 0 ||
            (strlen(g_config.chat_id) > 0 && strcmp(topic, chat_topic) == 0)) {
            display_mqtt_rx_pulse();
            handle_action_event(evt->data, evt->data_len);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "Error");
        break;

    default:
        break;
    }
}

// ── Metrics task — publishes every 30 s while connected ──────────────────────

static void metrics_task(void *arg)
{
    char topic[128];
    snprintf(topic, sizeof(topic), "dolls/%s/metrics", g_config.doll_id);

    while (1) {
        xEventGroupWaitBits(g_events, EVT_MQTT_CONNECTED,
                            pdFALSE, pdFALSE, portMAX_DELAY);

        wifi_ap_record_t ap = {};
        esp_wifi_sta_get_ap_info(&ap);

        EventBits_t bits = xEventGroupGetBits(g_events);

        cJSON *body = cJSON_CreateObject();
        cJSON_AddNumberToObject(body, "recording",          (bits & EVT_AUDIO_RECORDING) ? 1 : 0);
        cJSON_AddBoolToObject  (body, "t1",                 false);
        cJSON_AddBoolToObject  (body, "t2",                 false);
        cJSON_AddNumberToObject(body, "freeSRAM",           (int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        cJSON_AddNumberToObject(body, "freePSRAM",          (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        cJSON_AddNumberToObject(body, "wifiRSSI",           ap.rssi);
        cJSON_AddNumberToObject(body, "deepSleepCountdown", 0);
        char *payload = cJSON_PrintUnformatted(body);
        cJSON_Delete(body);

        esp_mqtt_client_publish(s_client, topic, payload, 0, 0, 0);
        display_mqtt_tx_pulse();
        ESP_LOGD(TAG, "metrics → %s", payload);
        free(payload);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ── Connect task — waits for doll_id then starts the client ──────────────────

static void mqtt_connect_task(void *arg)
{
    // Wait for doll registration AND image downloads to finish before connecting.
    // Image downloads use TLS which needs most of internal SRAM for RSA operations;
    // MQTT socket allocation during that window causes PK verify failures.
    xEventGroupWaitBits(g_events, EVT_DOLL_READY | EVT_IMAGES_DONE,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    snprintf(s_client_id, sizeof(s_client_id), "doll_%s", g_config.doll_id);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri                       = g_config.mqtt_url,
        .credentials.client_id                    = s_client_id,
        .credentials.username                     = s_client_id,
        .credentials.authentication.password      = g_config.apikey,
    };

    s_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
    ESP_LOGI(TAG, "Connecting to %s as %s", g_config.mqtt_url, s_client_id);

    static StaticTask_t s_metrics_tcb;
    StackType_t *mstack = heap_caps_malloc(4096, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (mstack) {
        xTaskCreateStaticPinnedToCore(metrics_task, "mqtt_metrics",
            4096 / sizeof(StackType_t), NULL, 2, mstack, &s_metrics_tcb, 1);
    }

    vTaskDelete(NULL);
}

// ── Public API ────────────────────────────────────────────────────────────────

static StaticTask_t s_mqtt_tcb;

void mqtt_start(void)
{
    StackType_t *stack = heap_caps_malloc(4096, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(stack);
    xTaskCreateStaticPinnedToCore(mqtt_connect_task, "mqtt_connect",
        4096 / sizeof(StackType_t), NULL, 3, stack, &s_mqtt_tcb, 1);
}
