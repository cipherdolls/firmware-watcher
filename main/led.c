#include "led.h"
#include "board.h"
#include "events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

void led_task_fn(void *pvParameter)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num   = LED_GPIO,
        .max_leds         = LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model        = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    led_strip_handle_t strip;
    if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip) != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }

    bool led_on = false;
    while (1) {
        EventBits_t bits = xEventGroupGetBits(g_events);
        bool connected      = (bits & EVT_WIFI_GOT_IP) != 0;
        bool recording       = (bits & EVT_AUDIO_RECORDING) != 0;
        bool playing         = (bits & EVT_AUDIO_PLAYING) != 0;
        bool conv_listening  = (bits & EVT_CONV_LISTENING) != 0;
        bool conv_mode       = (bits & EVT_CONV_MODE) != 0;

        if (recording) {
            // Solid red while recording
            led_strip_set_pixel(strip, 0, 30, 0, 0);
            led_strip_refresh(strip);
            led_on = true;
        } else if (playing) {
            // Solid blue while playing
            led_strip_set_pixel(strip, 0, 0, 0, 30);
            led_strip_refresh(strip);
            led_on = true;
        } else if (conv_listening) {
            // Solid green — listening for speech
            led_strip_set_pixel(strip, 0, 0, 30, 0);
            led_strip_refresh(strip);
            led_on = true;
        } else if (conv_mode) {
            // Dim green — waiting for server response
            led_strip_set_pixel(strip, 0, 0, 15, 0);
            led_strip_refresh(strip);
            led_on = true;
        } else if (connected) {
            // Off when idle + WiFi connected
            if (led_on) {
                led_strip_clear(strip);
                led_strip_refresh(strip);
                led_on = false;
            }
        } else {
            // Blink orange when not connected
            led_on = !led_on;
            if (led_on) {
                led_strip_set_pixel(strip, 0, 20, 8, 0);
            } else {
                led_strip_clear(strip);
            }
            led_strip_refresh(strip);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
