#include "led.h"
#include "board.h"
#include "events.h"
#include "power.h"
#include "record.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

// Map RMS (0–2000 typical speech range) to LED brightness (min–max).
// Uses a square-root curve so quiet speech still shows movement.
static uint8_t rms_to_brightness(uint16_t rms, uint8_t min_br, uint8_t max_br)
{
    if (rms == 0) return min_br;
    if (rms > 2000) rms = 2000;
    float norm = __builtin_sqrtf((float)rms / 2000.0f);
    uint8_t br = (uint8_t)(min_br + norm * (max_br - min_br));
    return br;
}

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
    uint32_t standby_counter = 0;
    while (1) {
        EventBits_t bits = xEventGroupGetBits(g_events);
        bool recording      = (bits & EVT_AUDIO_RECORDING) != 0;
        bool playing        = (bits & EVT_AUDIO_PLAYING) != 0;
        bool conv_listening = (bits & EVT_CONV_LISTENING) != 0;
        bool conv_mode      = (bits & EVT_CONV_MODE) != 0;

        // ── Standby mode: display off → white blink every 5 s ───────
        if (power_display_is_off()) {
            standby_counter++;
            if (standby_counter >= 100) {  // 100 × 50 ms = 5 s
                standby_counter = 0;
                led_strip_set_pixel(strip, 0, 15, 15, 15);
                led_strip_refresh(strip);
                vTaskDelay(pdMS_TO_TICKS(80));
                led_strip_clear(strip);
                led_strip_refresh(strip);
            }
            if (led_on) {
                led_strip_clear(strip);
                led_strip_refresh(strip);
                led_on = false;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        standby_counter = 0;

        // ── Talking mode: recording / playing / listening ───────────
        if (recording) {
            // Red pulsing with voice volume
            uint8_t br = rms_to_brightness(g_audio_rms, 8, 30);
            led_strip_set_pixel(strip, 0, br, 0, 0);
            led_strip_refresh(strip);
            led_on = true;
        } else if (playing) {
            // Blue pulsing with playback volume
            uint8_t br = rms_to_brightness(g_audio_rms, 8, 30);
            led_strip_set_pixel(strip, 0, 0, 0, br);
            led_strip_refresh(strip);
            led_on = true;
        } else if (conv_mode && !conv_listening) {
            // Blink white — waiting for server response
            static bool blink_phase = false;
            blink_phase = !blink_phase;
            if (blink_phase) {
                led_strip_set_pixel(strip, 0, 4, 4, 4);
            } else {
                led_strip_clear(strip);
            }
            led_strip_refresh(strip);
            led_on = true;
        } else if (conv_listening) {
            // Dim red pulsing with ambient level
            uint8_t br = rms_to_brightness(g_audio_rms, 3, 12);
            led_strip_set_pixel(strip, 0, br, 0, 0);
            led_strip_refresh(strip);
            led_on = true;
        } else {
            // ── Ready mode: LED off ─────────────────────────────────
            if (led_on) {
                led_strip_clear(strip);
                led_strip_refresh(strip);
                led_on = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
