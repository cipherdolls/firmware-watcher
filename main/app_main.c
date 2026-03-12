#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "events.h"
#include "config.h"
#include "config_store.h"
#include "display.h"
#include "touch.h"
#include "led.h"
#include "wifi_mgr.h"
#include "wifi_prov.h"
#include "power.h"
#include "http.h"
#include "mqtt.h"
#include "audio.h"
#include "record.h"
#include "stream_player.h"
#include "battery.h"

static const char *TAG = "main";

EventGroupHandle_t g_events;

void app_main(void)
{
    ESP_LOGI(TAG, "=== CipherDolls Watcher ===");

    // Core init
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    g_events = xEventGroupCreate();

    // Load saved config
    config_store_load();

    // Display (includes IO expander power-on + LVGL)
    ESP_ERROR_CHECK(display_init());
    display_set_state(DISPLAY_STATE_BOOT, "Starting...");

    // Battery monitor (ADC + display label, 30s interval)
    battery_init();

    // LED (solid white, deep sleep turns it off)
    xTaskCreatePinnedToCore(led_task_fn, "led", 4096, NULL, 3, NULL, 1);

    // WiFi init (always needed)
    wifi_mgr_init();

    if (!g_config.provisioned || strlen(g_config.ssid) == 0) {
        // First boot — run provisioning flow
        display_set_state(DISPLAY_STATE_WIFI_PROV, "Setup WiFi");
        // Touch init (needed for keyboard input)
        touch_init();
        xTaskCreate(wifi_prov_task_fn, "wifi_prov", 8192, NULL, 4, NULL);

        // Wait for provisioning to complete
        xEventGroupWaitBits(g_events, EVT_PROV_DONE, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "Provisioning done");
    } else {
        // Reconnect with saved credentials
        display_set_state(DISPLAY_STATE_WIFI_CONNECTING, g_config.ssid);
        wifi_mgr_connect(g_config.ssid, g_config.password);

        EventBits_t bits = xEventGroupWaitBits(g_events,
            EVT_WIFI_GOT_IP | EVT_WIFI_DISCONNECTED,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));

        if (bits & EVT_WIFI_GOT_IP) {
            display_set_state(DISPLAY_STATE_WIFI_OK, "Connected!");
            audio_init();
            http_sync_doll();
            mqtt_start();
            record_init();
            stream_player_init();
        } else {
            display_set_state(DISPLAY_STATE_ERROR, "WiFi failed\nHold button to re-setup");
        }
    }

    // Power management (deep sleep on inactivity)
    xTaskCreate(power_task_fn, "power", 2048, NULL, 1, NULL);

    // Main loop — keep task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
