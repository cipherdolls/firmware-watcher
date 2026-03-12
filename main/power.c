#include "power.h"
#include "display.h"
#include "touch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "power";
#define DISPLAY_OFF_TIMEOUT_MS  (60 * 1000)  // 1 minute → display off

static volatile uint32_t s_last_activity_ms = 0;
static volatile bool     s_display_asleep   = false;

void power_reset_sleep_timer(void)
{
    s_last_activity_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (s_display_asleep) {
        s_display_asleep = false;
        display_wake();
        ESP_LOGI(TAG, "Display woke (touch)");
    }
}

bool power_display_is_off(void)
{
    return s_display_asleep;
}

void power_task_fn(void *pvParameter)
{
    power_reset_sleep_timer();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(s_display_asleep ? 500 : 10000));

        // Check touch to wake display
        if (s_display_asleep) {
            uint16_t tx, ty;
            if (touch_get_point(&tx, &ty)) {
                power_reset_sleep_timer();
            }
            continue;
        }

        // Display off after 1 minute of inactivity
        uint32_t now     = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t elapsed = now - s_last_activity_ms;
        if (elapsed >= DISPLAY_OFF_TIMEOUT_MS) {
            ESP_LOGI(TAG, "Inactivity, display off");
            display_sleep();
            s_display_asleep = true;
        }
    }
}
