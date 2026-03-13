#include "http.h"
#include "config.h"
#include "config_store.h"
#include "display.h"
#include "avatar_img.h"
#include "events.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_sntp.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "http";

#define MAX_RETRIES    5
#define RETRY_DELAY_MS 5000
#define RESP_BUF_SIZE  2048

// ── HTTP response accumulator ─────────────────────────────────────────────────

typedef struct {
    char buf[RESP_BUF_SIZE];
    int  len;
} resp_t;

static esp_err_t on_data(esp_http_client_event_t *evt)
{
    resp_t *r = (resp_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && r) {
        int space = RESP_BUF_SIZE - r->len - 1;
        int copy  = evt->data_len < space ? evt->data_len : space;
        memcpy(r->buf + r->len, evt->data, copy);
        r->len += copy;
        r->buf[r->len] = '\0';
    }
    return ESP_OK;
}

// ── Simple GET — returns HTTP status code, -1 on transport error ──────────────

static int http_get(const char *url, const char *auth, resp_t *resp)
{
    esp_http_client_config_t cfg = {
        .url                = url,
        .crt_bundle_attach  = esp_crt_bundle_attach,
        .event_handler      = on_data,
        .user_data          = resp,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Authorization", auth);

    int status = -1;
    if (esp_http_client_perform(client) == ESP_OK) {
        status = esp_http_client_get_status_code(client);
    }
    esp_http_client_cleanup(client);
    return status;
}

// ── MAC address helper ────────────────────────────────────────────────────────

static void get_mac_str(char *out, size_t out_sz)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out, out_sz, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ── Main sync task ────────────────────────────────────────────────────────────

static void show_chat_status(const char *resp_json)
{
    char msg[128];
    cJSON *j = cJSON_Parse(resp_json);
    const char *cid = j ? cJSON_GetStringValue(cJSON_GetObjectItem(j, "chatId")) : NULL;
    if (cid && strlen(cid) > 0) {
        strlcpy(g_config.chat_id, cid, sizeof(g_config.chat_id));
        msg[0] = '\0';

        // Extract avatarId and scenarioId from nested chat object (requires ?include=chat)
        cJSON *chat = j ? cJSON_GetObjectItem(j, "chat") : NULL;
        const char *aid = chat ? cJSON_GetStringValue(cJSON_GetObjectItem(chat, "avatarId")) : NULL;
        if (aid && strlen(aid) > 0) {
            strlcpy(g_config.avatar_id, aid, sizeof(g_config.avatar_id));
            ESP_LOGI(TAG, "Avatar ID: %s", g_config.avatar_id);
        } else {
            memset(g_config.avatar_id, 0, sizeof(g_config.avatar_id));
        }
        const char *sid = chat ? cJSON_GetStringValue(cJSON_GetObjectItem(chat, "scenarioId")) : NULL;
        if (sid && strlen(sid) > 0) {
            strlcpy(g_config.scenario_id, sid, sizeof(g_config.scenario_id));
            ESP_LOGI(TAG, "Scenario ID: %s", g_config.scenario_id);
        } else {
            memset(g_config.scenario_id, 0, sizeof(g_config.scenario_id));
        }
    } else {
        memset(g_config.chat_id, 0, sizeof(g_config.chat_id));
        memset(g_config.avatar_id, 0, sizeof(g_config.avatar_id));
        memset(g_config.scenario_id, 0, sizeof(g_config.scenario_id));
        snprintf(msg, sizeof(msg), "No chat linked");
    }
    if (j) cJSON_Delete(j);
    display_set_state(DISPLAY_STATE_WIFI_OK, msg);
}

static void sync_task(void *arg)
{
    // Sync system clock via NTP — required for TLS certificate date validation
    display_set_state(DISPLAY_STATE_PROCESSING, "Syncing time...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    int sntp_retries = 0;
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED && sntp_retries++ < 10) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (sntp_retries >= 10) {
        ESP_LOGW(TAG, "SNTP sync timed out");
    } else {
        ESP_LOGI(TAG, "SNTP synced OK");
    }

    char url[256];
    char auth[128];
    char mac[18];
    get_mac_str(mac, sizeof(mac));
    snprintf(auth, sizeof(auth), "Bearer %s", g_config.apikey);
    ESP_LOGI(TAG, "API key: '%.8s...' (len=%d)", g_config.apikey, (int)strlen(g_config.apikey));

    // GET /dolls and POST /dolls both accept Bearer auth.
    // 401 on either means the API key is wrong — surface that immediately.
    display_set_state(DISPLAY_STATE_PROCESSING, "Connecting...");

    resp_t resp = {.len = 0};
    int status  = 0;

    // ── Register / verify doll ────────────────────────────────────────────────
    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        resp.len = 0;
        status   = 0;

        // If we already have a doll_id, verify it still exists on the backend
        if (strlen(g_config.doll_id) > 0) {
            display_set_state(DISPLAY_STATE_PROCESSING, "Checking API key...");
            snprintf(url, sizeof(url), "%s/dolls/%s?include=chat", g_config.server_url, g_config.doll_id);
            ESP_LOGI(TAG, "GET %s", url);

            status = http_get(url, auth, &resp);

            if (status == 401) {
                ESP_LOGE(TAG, "API key invalid");
                display_set_state(DISPLAY_STATE_ERROR, "Invalid API key\nCheck .env");
                goto done;
            }
            if (status == 200) {
                ESP_LOGI(TAG, "Doll verified: %s", g_config.doll_id);
                show_chat_status(resp.buf);
                xEventGroupSetBits(g_events, EVT_DOLL_READY);
                avatar_img_start();
                goto done;
            }

            // 404 → doll deleted on backend, fall through to POST
            ESP_LOGW(TAG, "GET /dolls/:id returned %d — re-registering", status);
            memset(g_config.doll_id, 0, sizeof(g_config.doll_id));
            resp.len = 0;
        }

        // POST /dolls to register this device — also validates the API key
        display_set_state(DISPLAY_STATE_PROCESSING, "Checking API key...");
        snprintf(url, sizeof(url), "%s/dolls", g_config.server_url);
        ESP_LOGI(TAG, "POST %s  mac=%s  dollBodyId=%s", url, mac, g_config.doll_body_id);

        cJSON *body = cJSON_CreateObject();
        cJSON_AddStringToObject(body, "macAddress", mac);
        cJSON_AddStringToObject(body, "dollBodyId", g_config.doll_body_id);
        char *body_str = cJSON_PrintUnformatted(body);
        cJSON_Delete(body);

        esp_http_client_config_t cfg = {
            .url               = url,
            .method            = HTTP_METHOD_POST,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .event_handler     = on_data,
            .user_data         = &resp,
        };
        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        esp_http_client_set_header(client, "Authorization", auth);
        esp_http_client_set_header(client, "Content-Type",  "application/json");
        esp_http_client_set_post_field(client, body_str, strlen(body_str));

        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            status = esp_http_client_get_status_code(client);
        }
        esp_http_client_cleanup(client);
        free(body_str);

        ESP_LOGI(TAG, "POST status=%d  body=%s", status, resp.buf);

        if (status == 401) {
            ESP_LOGE(TAG, "API key invalid");
            display_set_state(DISPLAY_STATE_ERROR, "Invalid API key\nCheck .env");
            goto done;
        }

        display_set_state(DISPLAY_STATE_PROCESSING, "Registering doll...");

        if (status >= 200 && status < 300) {
            cJSON *json = cJSON_Parse(resp.buf);
            if (json) {
                const char *id = cJSON_GetStringValue(cJSON_GetObjectItem(json, "id"));
                if (id) {
                    strlcpy(g_config.doll_id, id, sizeof(g_config.doll_id));
                    config_store_save();
                    ESP_LOGI(TAG, "Registered — doll_id=%s", g_config.doll_id);
                    show_chat_status(resp.buf);
                    xEventGroupSetBits(g_events, EVT_DOLL_READY);
                    avatar_img_start();
                }
                cJSON_Delete(json);
            }
            goto done;
        }

        // Extract message from error JSON for logging
        cJSON *err_json = cJSON_Parse(resp.buf);
        const char *err_msg = NULL;
        if (err_json) {
            err_msg = cJSON_GetStringValue(cJSON_GetObjectItem(err_json, "message"));
        }
        ESP_LOGW(TAG, "Attempt %d failed (status=%d): %s",
                 attempt + 1, status, err_msg ? err_msg : resp.buf);
        if (err_json) cJSON_Delete(err_json);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "Max retries reached — registration failed");
    display_set_state(DISPLAY_STATE_ERROR, "Registration failed\nCheck doll body ID");

done:
    vTaskDelete(NULL);
}

// ── Public entry point ────────────────────────────────────────────────────────

// Use PSRAM stack to keep internal SRAM free for TLS RSA operations
static StaticTask_t s_sync_tcb;

void http_sync_doll(void)
{
    StackType_t *stack = heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(stack);
    xTaskCreateStaticPinnedToCore(sync_task, "http_sync",
        8192 / sizeof(StackType_t), NULL, 3, stack, &s_sync_tcb, 1);
}
