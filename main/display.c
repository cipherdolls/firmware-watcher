#include "display.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_spd2010.h"
#include "esp_heap_caps.h"
#include "lvgl.h"

static const char *TAG = "display";

// LVGL
#define LVGL_TICK_MS        2
#define LVGL_BUFF_LINES     20
#define LVGL_TASK_STACK     (6 * 1024)
#define LVGL_TASK_PRIO      2

static SemaphoreHandle_t s_lvgl_mux = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;
static lv_disp_drv_t s_disp_drv;
static lv_disp_draw_buf_t s_disp_buf;

// Current status label (on the main screen)
static lv_obj_t *s_label = NULL;

// Battery indicator (top of screen)
static lv_obj_t *s_batt_label = NULL;

// Bottom status panel: WiFi + MQTT in 2x2 grid
static lv_obj_t *s_wifi_icon  = NULL;   // WiFi signal icon (top-left)
static lv_obj_t *s_wifi_label = NULL;   // "WiFi" text (top-right)
static lv_obj_t *s_mqtt_label = NULL;   // "MQTT" text (bottom-right)

// Scenario image (full-screen background)
static lv_obj_t *s_scenario_img = NULL;
static lv_img_dsc_t s_scenario_dsc;

// Avatar image (small overlay) + white ring behind it
static lv_obj_t *s_avatar_ring = NULL;
static lv_obj_t *s_avatar_img  = NULL;
static lv_img_dsc_t s_avatar_dsc;

// Traffic dots: TX (outgoing, left) and RX (incoming, right) below MQTT label
static lv_obj_t *s_dot_tx = NULL;
static lv_obj_t *s_dot_rx = NULL;
static esp_timer_handle_t s_dim_timer_tx = NULL;
static esp_timer_handle_t s_dim_timer_rx = NULL;

#define DOT_SIZE 8
#define DOT_DIM  lv_color_make(0x33, 0x33, 0x33)
#define DOT_TX   lv_color_make(0x00, 0xFF, 0x88)   // green — outgoing
#define DOT_RX   lv_color_make(0x00, 0xCC, 0xFF)   // cyan  — incoming

static lv_obj_t *make_dot(lv_obj_t *scr)
{
    lv_obj_t *d = lv_obj_create(scr);
    lv_obj_set_size(d, DOT_SIZE, DOT_SIZE);
    lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(d, DOT_DIM, 0);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(d, 0, 0);
    lv_obj_set_style_pad_all(d, 0, 0);
    lv_obj_clear_flag(d, LV_OBJ_FLAG_SCROLLABLE);
    return d;
}

// esp_timer callbacks — run in esp_timer task, safe to acquire LVGL mutex
static void dim_tx_cb(void *arg)
{
    if (!s_dot_tx) return;
    if (!display_lvgl_lock(100)) return;
    lv_obj_set_style_bg_color(s_dot_tx, DOT_DIM, 0);
    display_lvgl_unlock();
}

static void dim_rx_cb(void *arg)
{
    if (!s_dot_rx) return;
    if (!display_lvgl_lock(100)) return;
    lv_obj_set_style_bg_color(s_dot_rx, DOT_DIM, 0);
    display_lvgl_unlock();
}

// ─────────────────────────────────────────────────────────────────────────────
// LCD power via PCA9535 IO expander
// ─────────────────────────────────────────────────────────────────────────────
static void lcd_power_on(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = AUDIO_I2C_SDA,
        .scl_io_num       = AUDIO_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AUDIO_I2C_FREQ,
    };
    i2c_param_config(AUDIO_I2C_PORT, &cfg);
    i2c_driver_install(AUDIO_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    // Configure port 1 (pins 8-15) as outputs
    uint8_t config_cmd[] = { PCA9535_CONFIG1, 0x00 };
    i2c_master_write_to_device(AUDIO_I2C_PORT, IO_EXP_ADDR,
        config_cmd, sizeof(config_cmd), pdMS_TO_TICKS(100));

    // Set all port-1 outputs HIGH (pin 9 = BSP_PWR_LCD)
    uint8_t output_cmd[] = { PCA9535_OUTPUT1, 0xFF };
    i2c_master_write_to_device(AUDIO_I2C_PORT, IO_EXP_ADDR,
        output_cmd, sizeof(output_cmd), pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "LCD power ON (IO expander 0x21 pin 9)");
    vTaskDelay(pdMS_TO_TICKS(200));
}

// ─────────────────────────────────────────────────────────────────────────────
// Backlight via LEDC PWM
// ─────────────────────────────────────────────────────────────────────────────
static void backlight_set(int percent)
{
    uint32_t duty = ((1 << 10) - 1) * percent / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void backlight_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t ch = {
        .gpio_num   = LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

// ─────────────────────────────────────────────────────────────────────────────
// LVGL flush callback (async, triggered by on_color_trans_done ISR)
// ─────────────────────────────────────────────────────────────────────────────
static bool lvgl_flush_done_cb(esp_lcd_panel_io_handle_t io,
                                esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    lv_disp_drv_t *drv = (lv_disp_drv_t *)ctx;
    lv_disp_flush_ready(drv);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_draw_bitmap(s_panel,
        area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
}

// SPD2010 requires x coords aligned to multiples of 4
static void lvgl_rounder_cb(lv_disp_drv_t *drv, lv_area_t *area)
{
    area->x1 = (area->x1 >> 2) << 2;
    area->x2 = ((area->x2 >> 2) << 2) + 3;
}

static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_MS);
}

static void lvgl_task(void *arg)
{
    while (1) {
        if (display_lvgl_lock(-1)) {
            uint32_t delay_ms = lv_timer_handler();
            display_lvgl_unlock();
            if (delay_ms > 500) delay_ms = 500;
            if (delay_ms < 2)   delay_ms = 2;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t display_init(void)
{
    // 1. Power on LCD via IO expander (MUST be first)
    lcd_power_on();

    // 2. SPI bus
    spi_bus_config_t bus = SPD2010_PANEL_BUS_QSPI_CONFIG(
        LCD_PCLK, LCD_DATA0, LCD_DATA1, LCD_DATA2, LCD_DATA3,
        LCD_H_RES * LVGL_BUFF_LINES * sizeof(lv_color_t));
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    // 3. Panel IO (on_color_trans_done wired to LVGL flush ready)
    esp_lcd_panel_io_handle_t io = NULL;
    lv_disp_drv_init(&s_disp_drv);  // init early so pointer is valid for callback
    esp_lcd_panel_io_spi_config_t io_cfg = SPD2010_PANEL_IO_QSPI_CONFIG(
        LCD_CS, lvgl_flush_done_cb, &s_disp_drv);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &io));

    // 4. SPD2010 panel
    spd2010_vendor_config_t vendor = { .flags.use_qspi_interface = 1 };
    esp_lcd_panel_dev_config_t pcfg = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config  = &vendor,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(io, &pcfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false));

    // 5. Backlight (starts off, will fade in after LVGL draws first frame)
    backlight_init();

    // 6. LVGL
    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(
        LCD_H_RES * LVGL_BUFF_LINES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(
        LCD_H_RES * LVGL_BUFF_LINES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 && buf2);
    lv_disp_draw_buf_init(&s_disp_buf, buf1, buf2,
        LCD_H_RES * LVGL_BUFF_LINES);

    s_disp_drv.hor_res       = LCD_H_RES;
    s_disp_drv.ver_res       = LCD_V_RES;
    s_disp_drv.flush_cb      = lvgl_flush_cb;
    s_disp_drv.rounder_cb    = lvgl_rounder_cb;
    s_disp_drv.draw_buf      = &s_disp_buf;
    s_disp_drv.user_data     = s_panel;
    lv_disp_drv_register(&s_disp_drv);

    // 7. LVGL tick timer
    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .name     = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_MS * 1000));

    // 8. LVGL mutex + task
    s_lvgl_mux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_TASK_STACK, NULL,
        LVGL_TASK_PRIO, NULL, 0);

    // 9. Turn backlight on
    backlight_set(100);
    ESP_LOGI(TAG, "Display init OK");
    return ESP_OK;
}

bool display_lvgl_lock(int timeout_ms)
{
    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_lvgl_mux, ticks) == pdTRUE;
}

void display_lvgl_unlock(void)
{
    xSemaphoreGive(s_lvgl_mux);
}

static lv_color_t state_bg(display_state_t s)
{
    switch (s) {
        case DISPLAY_STATE_BOOT:            return lv_color_make(0x10, 0x10, 0x30);
        case DISPLAY_STATE_WIFI_PROV:       return lv_color_make(0x10, 0x10, 0x10);
        case DISPLAY_STATE_WIFI_CONNECTING: return lv_color_make(0x10, 0x30, 0x10);
        case DISPLAY_STATE_WIFI_OK:         return lv_color_make(0x00, 0x40, 0x00);
        case DISPLAY_STATE_RECORDING:       return lv_color_make(0x40, 0x00, 0x00);
        case DISPLAY_STATE_PLAYING:         return lv_color_make(0x00, 0x20, 0x40);
        case DISPLAY_STATE_PROCESSING:      return lv_color_make(0x20, 0x20, 0x00);
        case DISPLAY_STATE_ERROR:           return lv_color_make(0x50, 0x10, 0x10);
        default:                            return lv_color_make(0x00, 0x00, 0x00);
    }
}

void display_set_state(display_state_t state, const char *text)
{
    if (!display_lvgl_lock(100)) return;

    lv_obj_t *scr = lv_scr_act();

    // Show scenario + avatar for WIFI_OK and PLAYING, hide for other states
    bool show_images = (state == DISPLAY_STATE_WIFI_OK || state == DISPLAY_STATE_PLAYING);
    bool has_images = (s_scenario_img || s_avatar_img);
    if (has_images) {
        if (show_images) {
            if (s_scenario_img) lv_obj_clear_flag(s_scenario_img, LV_OBJ_FLAG_HIDDEN);
            if (s_avatar_ring)  lv_obj_clear_flag(s_avatar_ring, LV_OBJ_FLAG_HIDDEN);
            if (s_avatar_img)   lv_obj_clear_flag(s_avatar_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_opa(scr, LV_OPA_TRANSP, 0);
        } else {
            if (s_scenario_img) lv_obj_add_flag(s_scenario_img, LV_OBJ_FLAG_HIDDEN);
            if (s_avatar_ring)  lv_obj_add_flag(s_avatar_ring, LV_OBJ_FLAG_HIDDEN);
            if (s_avatar_img)   lv_obj_add_flag(s_avatar_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(scr, state_bg(state), 0);
            lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
        }
    } else {
        lv_obj_set_style_bg_color(scr, state_bg(state), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    }

    if (!s_label) {
        s_label = lv_label_create(scr);
        lv_obj_align(s_label, LV_ALIGN_CENTER, 0, -16);
        lv_label_set_long_mode(s_label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(s_label, LCD_H_RES - 60);
        lv_obj_set_style_text_align(s_label, LV_TEXT_ALIGN_CENTER, 0);
    }
    lv_obj_set_style_text_color(s_label, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    if (text) lv_label_set_text(s_label, text);

    display_lvgl_unlock();
}

// ── Bottom status panel helpers ───────────────────────────────────────────────

// Anchor positions for the 2x2 grid in the bottom arc area
#define STATUS_LEFT_X   175       // left column X (icons/dots)
#define STATUS_RIGHT_X  215       // right column X (text)
#define STATUS_ROW1_Y   (-38)     // top row Y offset from bottom
#define STATUS_ROW2_Y   (-20)     // bottom row Y offset from bottom

static void ensure_bottom_panel(void)
{
    lv_obj_t *scr = lv_scr_act();

    // Row 1 left: WiFi icon
    if (!s_wifi_icon) {
        s_wifi_icon = lv_label_create(scr);
        lv_obj_set_pos(s_wifi_icon, STATUS_LEFT_X, LCD_V_RES + STATUS_ROW1_Y);
        lv_obj_set_style_text_color(s_wifi_icon, lv_color_make(0x66, 0x66, 0x66), 0);
        lv_label_set_text(s_wifi_icon, LV_SYMBOL_WIFI);
    }

    // Row 1 right: "WiFi" text
    if (!s_wifi_label) {
        s_wifi_label = lv_label_create(scr);
        lv_obj_set_pos(s_wifi_label, STATUS_RIGHT_X, LCD_V_RES + STATUS_ROW1_Y);
        lv_obj_set_style_text_color(s_wifi_label, lv_color_make(0x66, 0x66, 0x66), 0);
        lv_label_set_text(s_wifi_label, "WiFi");
    }

    // Row 2 left: TX + RX dots
    if (!s_dot_tx) {
        s_dot_tx = make_dot(scr);
        lv_obj_set_pos(s_dot_tx, STATUS_LEFT_X, LCD_V_RES + STATUS_ROW2_Y);

        s_dot_rx = make_dot(scr);
        lv_obj_set_pos(s_dot_rx, STATUS_LEFT_X + DOT_SIZE + 4, LCD_V_RES + STATUS_ROW2_Y);

        esp_timer_create_args_t ta = { .callback = dim_tx_cb, .name = "dot_tx" };
        esp_timer_create(&ta, &s_dim_timer_tx);
        esp_timer_create_args_t rb = { .callback = dim_rx_cb, .name = "dot_rx" };
        esp_timer_create(&rb, &s_dim_timer_rx);
    }

    // Row 2 right: "MQTT" text
    if (!s_mqtt_label) {
        s_mqtt_label = lv_label_create(scr);
        lv_obj_set_pos(s_mqtt_label, STATUS_RIGHT_X, LCD_V_RES + STATUS_ROW2_Y);
        lv_obj_set_style_text_color(s_mqtt_label, lv_color_make(0x66, 0x66, 0x66), 0);
        lv_label_set_text(s_mqtt_label, "MQTT");
    }
}

void display_set_wifi_status(bool connected, int rssi)
{
    if (!display_lvgl_lock(100)) return;
    ensure_bottom_panel();

    // WiFi icon color based on signal strength
    lv_color_t icon_col;
    if (!connected) {
        icon_col = lv_color_make(0x66, 0x66, 0x66);      // gray
    } else if (rssi >= -50) {
        icon_col = lv_color_make(0x00, 0xFF, 0x88);       // green (excellent)
    } else if (rssi >= -65) {
        icon_col = lv_color_make(0x88, 0xFF, 0x00);       // lime (good)
    } else if (rssi >= -75) {
        icon_col = lv_color_make(0xFF, 0xCC, 0x00);       // yellow (fair)
    } else {
        icon_col = lv_color_make(0xFF, 0x66, 0x33);       // orange (weak)
    }
    lv_obj_set_style_text_color(s_wifi_icon, icon_col, 0);

    // "WiFi" label: green when connected, gray otherwise
    lv_obj_set_style_text_color(s_wifi_label,
        connected ? lv_color_make(0x00, 0xFF, 0x88)
                  : lv_color_make(0x66, 0x66, 0x66), 0);

    display_lvgl_unlock();
}

void display_set_mqtt_connected(bool connected)
{
    if (!display_lvgl_lock(100)) return;
    ensure_bottom_panel();

    lv_obj_set_style_text_color(s_mqtt_label,
        connected ? lv_color_make(0x00, 0xFF, 0x88)
                  : lv_color_make(0x66, 0x66, 0x66), 0);

    display_lvgl_unlock();
}

void display_mqtt_tx_pulse(void)
{
    if (!s_dot_tx || !s_dim_timer_tx) return;
    if (!display_lvgl_lock(50)) return;
    lv_obj_set_style_bg_color(s_dot_tx, DOT_TX, 0);
    display_lvgl_unlock();
    esp_timer_stop(s_dim_timer_tx);
    esp_timer_start_once(s_dim_timer_tx, 300 * 1000);  // 300 ms
}

void display_mqtt_rx_pulse(void)
{
    if (!s_dot_rx || !s_dim_timer_rx) return;
    if (!display_lvgl_lock(50)) return;
    lv_obj_set_style_bg_color(s_dot_rx, DOT_RX, 0);
    display_lvgl_unlock();
    esp_timer_stop(s_dim_timer_rx);
    esp_timer_start_once(s_dim_timer_rx, 300 * 1000);  // 300 ms
}

void display_set_battery(int percent, bool charging)
{
    if (!display_lvgl_lock(100)) return;

    lv_obj_t *scr = lv_scr_act();
    if (!s_batt_label) {
        s_batt_label = lv_label_create(scr);
        lv_obj_align(s_batt_label, LV_ALIGN_TOP_MID, 0, 8);
        lv_obj_set_style_text_color(s_batt_label, lv_color_make(0xCC, 0xCC, 0xCC), 0);
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "%s %d%%", charging ? LV_SYMBOL_CHARGE : LV_SYMBOL_BATTERY_FULL, percent);
    lv_label_set_text(s_batt_label, buf);

    // Colour: green > 50%, yellow 20-50%, red < 20%
    lv_color_t col;
    if (charging)        col = lv_color_make(0x00, 0xCC, 0xFF);  // cyan
    else if (percent > 50) col = lv_color_make(0x00, 0xDD, 0x44);  // green
    else if (percent > 20) col = lv_color_make(0xFF, 0xCC, 0x00);  // yellow
    else                   col = lv_color_make(0xFF, 0x33, 0x33);  // red
    lv_obj_set_style_text_color(s_batt_label, col, 0);

    display_lvgl_unlock();
}

// Helper to enforce z-order: scenario (back) → avatar (middle) → text labels (front)
static void enforce_z_order(void)
{
    if (s_scenario_img) lv_obj_move_background(s_scenario_img);
    if (s_avatar_ring)  lv_obj_move_foreground(s_avatar_ring);
    if (s_avatar_img)   lv_obj_move_foreground(s_avatar_img);
    if (s_label)        lv_obj_move_foreground(s_label);
    if (s_batt_label)   lv_obj_move_foreground(s_batt_label);
    if (s_wifi_icon)    lv_obj_move_foreground(s_wifi_icon);
    if (s_wifi_label)   lv_obj_move_foreground(s_wifi_label);
    if (s_mqtt_label)   lv_obj_move_foreground(s_mqtt_label);
    if (s_dot_tx)       lv_obj_move_foreground(s_dot_tx);
    if (s_dot_rx)       lv_obj_move_foreground(s_dot_rx);
}

void display_set_scenario(uint16_t *rgb565, int w, int h)
{
    if (!rgb565 || w <= 0 || h <= 0) return;
    if (!display_lvgl_lock(1000)) return;

    // Set up image descriptor pointing to the PSRAM framebuffer
    s_scenario_dsc.header.always_zero = 0;
    s_scenario_dsc.header.w           = w;
    s_scenario_dsc.header.h           = h;
    s_scenario_dsc.header.cf          = LV_IMG_CF_TRUE_COLOR;
    s_scenario_dsc.data_size          = w * h * sizeof(lv_color_t);
    s_scenario_dsc.data               = (const uint8_t *)rgb565;

    lv_obj_t *scr = lv_scr_act();

    if (!s_scenario_img) {
        s_scenario_img = lv_img_create(scr);
    }
    lv_img_set_src(s_scenario_img, &s_scenario_dsc);
    lv_obj_align(s_scenario_img, LV_ALIGN_CENTER, 0, 0);

    // Circular clip for round display
    lv_obj_set_style_radius(s_scenario_img, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_clip_corner(s_scenario_img, true, 0);

    enforce_z_order();

    // Make the screen background transparent so the image shows through
    lv_obj_set_style_bg_opa(scr, LV_OPA_TRANSP, 0);

    ESP_LOGI(TAG, "Scenario image set (%dx%d)", w, h);
    display_lvgl_unlock();
}

void display_sleep(void)
{
    backlight_set(0);
    ESP_LOGI(TAG, "Display sleep (backlight off)");
}

void display_wake(void)
{
    backlight_set(100);
    ESP_LOGI(TAG, "Display wake (backlight on)");
}

void display_set_avatar(uint16_t *rgb565, int w, int h)
{
    if (!rgb565 || w <= 0 || h <= 0) return;
    if (!display_lvgl_lock(1000)) return;

    // Set up image descriptor pointing to the PSRAM framebuffer
    s_avatar_dsc.header.always_zero = 0;
    s_avatar_dsc.header.w           = w;
    s_avatar_dsc.header.h           = h;
    s_avatar_dsc.header.cf          = LV_IMG_CF_TRUE_COLOR;
    s_avatar_dsc.data_size          = w * h * sizeof(lv_color_t);
    s_avatar_dsc.data               = (const uint8_t *)rgb565;

    lv_obj_t *scr = lv_scr_act();

    // Create white ring (slightly larger circle behind the avatar)
    #define RING_PAD 3
    if (!s_avatar_ring) {
        s_avatar_ring = lv_obj_create(scr);
        lv_obj_set_size(s_avatar_ring, w + RING_PAD * 2, h + RING_PAD * 2);
        lv_obj_set_style_radius(s_avatar_ring, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s_avatar_ring, lv_color_make(0xFF, 0xFF, 0xFF), 0);
        lv_obj_set_style_bg_opa(s_avatar_ring, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(s_avatar_ring, 0, 0);
        lv_obj_set_style_pad_all(s_avatar_ring, 0, 0);
        lv_obj_clear_flag(s_avatar_ring, LV_OBJ_FLAG_SCROLLABLE);
    }
    lv_obj_align(s_avatar_ring, LV_ALIGN_BOTTOM_MID, 0, -50 + RING_PAD);

    if (!s_avatar_img) {
        s_avatar_img = lv_img_create(scr);
    }
    lv_img_set_src(s_avatar_img, &s_avatar_dsc);

    // Position small avatar at bottom-center (inside the circular display area)
    lv_obj_align(s_avatar_img, LV_ALIGN_BOTTOM_MID, 0, -50);

    // Circular clip
    lv_obj_set_style_radius(s_avatar_img, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_clip_corner(s_avatar_img, true, 0);

    enforce_z_order();

    // Make the screen background transparent so the image shows through
    lv_obj_set_style_bg_opa(scr, LV_OPA_TRANSP, 0);

    ESP_LOGI(TAG, "Avatar image set (%dx%d)", w, h);
    display_lvgl_unlock();
}
