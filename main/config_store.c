#include "config_store.h"
#include "config.h"
#include "secret_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "config_store";
#define NVS_NAMESPACE "doll_cfg"

esp_err_t config_store_load(void)
{
    // Always apply compile-time secrets — no NVS read needed for these
    strlcpy(g_config.ssid,         SECRET_SSID,        sizeof(g_config.ssid));
    strlcpy(g_config.password,     SECRET_PASSWORD,     sizeof(g_config.password));
    strlcpy(g_config.apikey,       SECRET_APIKEY,       sizeof(g_config.apikey));
    strlcpy(g_config.doll_body_id, SECRET_DOLL_BODY_ID, sizeof(g_config.doll_body_id));
    strlcpy(g_config.server_url,   SECRET_SERVER_URL,   sizeof(g_config.server_url));
    strlcpy(g_config.mqtt_url,     SECRET_MQTT_URL,     sizeof(g_config.mqtt_url));
    strlcpy(g_config.stream_recorder_url, SECRET_STREAM_RECORDER_URL, sizeof(g_config.stream_recorder_url));
    strlcpy(g_config.stream_player_url,   SECRET_STREAM_PLAYER_URL,   sizeof(g_config.stream_player_url));
    g_config.provisioned = (strlen(g_config.ssid) > 0);
    g_config.doll_id[0]  = '\0';

    // Load only doll_id from NVS (populated after first successful POST /dolls)
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No NVS entry yet — doll_id will be set after registration");
        return ESP_OK;
    }
    ESP_ERROR_CHECK(err);

    size_t len = sizeof(g_config.doll_id);
    nvs_get_str(h, "doll_id", g_config.doll_id, &len);
    nvs_close(h);

    ESP_LOGI(TAG, "Config loaded: ssid='%s' doll_id='%s'", g_config.ssid, g_config.doll_id);
    return ESP_OK;
}

esp_err_t config_store_save(void)
{
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h));
    nvs_set_str(h, "doll_id", g_config.doll_id);
    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Config saved: doll_id='%s'", g_config.doll_id);
    return err;
}

esp_err_t config_store_clear(void)
{
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h));
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    g_config.doll_id[0] = '\0';
    ESP_LOGI(TAG, "Config cleared");
    return ESP_OK;
}
