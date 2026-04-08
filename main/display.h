#pragma once
#include "esp_err.h"
#include "lvgl.h"
#include <stdbool.h>

esp_err_t display_init(void);
bool display_lvgl_lock(int timeout_ms);
void display_lvgl_unlock(void);

// State colors (RGB packed: 0xRRGGBB)
typedef enum {
    DISPLAY_STATE_BOOT,
    DISPLAY_STATE_WIFI_PROV,
    DISPLAY_STATE_WIFI_CONNECTING,
    DISPLAY_STATE_WIFI_OK,
    DISPLAY_STATE_RECORDING,
    DISPLAY_STATE_PLAYING,
    DISPLAY_STATE_PROCESSING,
    DISPLAY_STATE_ERROR,
} display_state_t;

void display_set_state(display_state_t state, const char *text);
void display_set_wifi_status(bool connected, int rssi);  // WiFi icon + label
void display_set_mqtt_connected(bool connected);
void display_mqtt_tx_pulse(void);   // flash TX dot (outgoing)
void display_mqtt_rx_pulse(void);   // flash RX dot (incoming)
void display_set_battery(int percent, bool charging);
void display_set_scenario(uint16_t *rgb565, int w, int h); // takes ownership of rgb565 buffer
void display_set_avatar(uint16_t *rgb565, int w, int h);   // takes ownership of rgb565 buffer
void display_set_ws_status(bool player_connected, bool recorder_connected);
void display_sleep(void);   // turn off backlight (before light sleep)
void display_wake(void);    // turn backlight back on
