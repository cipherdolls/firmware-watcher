#include "scenario_img.h"
#include "board.h"
#include "config.h"
#include "events.h"
#include "display.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/tjpgd.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "scenario_img";

#define JPEG_BUF_MAX   (200 * 1024)  // max JPEG download size
#define TJPGD_WORK_SZ  3100          // tjpgd work buffer

// ── JPEG download accumulator ────────────────────────────────────────────────

typedef struct {
    uint8_t *buf;   // PSRAM buffer for JPEG data
    int      len;
    int      cap;
} dl_buf_t;

static esp_err_t on_data(esp_http_client_event_t *evt)
{
    dl_buf_t *d = (dl_buf_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && d) {
        int space = d->cap - d->len;
        int copy  = evt->data_len < space ? evt->data_len : space;
        if (copy > 0) {
            memcpy(d->buf + d->len, evt->data, copy);
            d->len += copy;
        }
    }
    return ESP_OK;
}

// ── tjpgd callbacks ──────────────────────────────────────────────────────────

typedef struct {
    const uint8_t *jpeg;     // source JPEG data
    int            jpeg_len;
    int            jpeg_pos;
    uint16_t      *fb;       // RGB565 framebuffer (PSRAM)
    int            fb_w;     // framebuffer width
} decode_ctx_t;

// Input function: feed JPEG data to decoder
static UINT tjpgd_input(JDEC *jd, BYTE *buf, UINT ndata)
{
    decode_ctx_t *ctx = (decode_ctx_t *)jd->device;
    int avail = ctx->jpeg_len - ctx->jpeg_pos;
    if ((int)ndata > avail) ndata = avail;
    if (buf) {
        memcpy(buf, ctx->jpeg + ctx->jpeg_pos, ndata);
    }
    ctx->jpeg_pos += ndata;
    return ndata;
}

// Output function: convert RGB888 MCU block to RGB565 and write to framebuffer
// ROM tjpgd outputs RGB888 (JD_FORMAT=0): 3 bytes per pixel (R, G, B)
static UINT tjpgd_output(JDEC *jd, void *bitmap, JRECT *rect)
{
    decode_ctx_t *ctx = (decode_ctx_t *)jd->device;
    uint8_t *rgb = (uint8_t *)bitmap;

    for (int y = rect->top; y <= rect->bottom; y++) {
        for (int x = rect->left; x <= rect->right; x++) {
            uint8_t r = *rgb++;
            uint8_t g = *rgb++;
            uint8_t b = *rgb++;
            // RGB565 with byte swap (CONFIG_LV_COLOR_16_SWAP=y)
            uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            ctx->fb[y * ctx->fb_w + x] = (c >> 8) | (c << 8);
        }
    }
    return 1; // continue decoding
}

// ── Download + decode task ───────────────────────────────────────────────────

static void scenario_task(void *arg)
{
    if (strlen(g_config.scenario_id) == 0) {
        ESP_LOGW(TAG, "No scenario_id — skipping download");
        goto done;
    }

    // Build URL
    char url[256];
    snprintf(url, sizeof(url), "%s/scenarios/%s/picture.jpg?x=%d&y=%d",
             g_config.server_url, g_config.scenario_id, LCD_H_RES, LCD_V_RES);

    char auth[128];
    snprintf(auth, sizeof(auth), "Bearer %s", g_config.apikey);

    ESP_LOGI(TAG, "Downloading scenario: %s", url);

    // Allocate JPEG download buffer in PSRAM
    dl_buf_t dl = {
        .buf = heap_caps_malloc(JPEG_BUF_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT),
        .len = 0,
        .cap = JPEG_BUF_MAX,
    };
    if (!dl.buf) {
        ESP_LOGE(TAG, "Failed to alloc JPEG buffer");
        goto done;
    }

    // Download JPEG
    esp_http_client_config_t cfg = {
        .url               = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler     = on_data,
        .user_data         = &dl,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Authorization", auth);

    esp_err_t err = esp_http_client_perform(client);
    int status = -1;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
    }
    esp_http_client_cleanup(client);

    if (status != 200 || dl.len == 0) {
        ESP_LOGE(TAG, "Download failed: status=%d len=%d", status, dl.len);
        heap_caps_free(dl.buf);
        goto done;
    }

    ESP_LOGI(TAG, "Downloaded %d bytes JPEG", dl.len);

    // Allocate RGB565 framebuffer in PSRAM
    int fb_size = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    uint16_t *fb = heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!fb) {
        ESP_LOGE(TAG, "Failed to alloc framebuffer (%d bytes)", fb_size);
        heap_caps_free(dl.buf);
        goto done;
    }
    memset(fb, 0, fb_size);

    // Decode JPEG
    decode_ctx_t ctx = {
        .jpeg     = dl.buf,
        .jpeg_len = dl.len,
        .jpeg_pos = 0,
        .fb       = fb,
        .fb_w     = LCD_H_RES,
    };

    void *work = heap_caps_malloc(TJPGD_WORK_SZ, MALLOC_CAP_DEFAULT);
    if (!work) {
        ESP_LOGE(TAG, "Failed to alloc tjpgd work buffer");
        heap_caps_free(dl.buf);
        heap_caps_free(fb);
        goto done;
    }

    JDEC jdec;
    JRESULT res = jd_prepare(&jdec, tjpgd_input, work, TJPGD_WORK_SZ, &ctx);
    if (res != JDR_OK) {
        ESP_LOGE(TAG, "jd_prepare failed: %d", res);
        heap_caps_free(work);
        heap_caps_free(dl.buf);
        heap_caps_free(fb);
        goto done;
    }

    ESP_LOGI(TAG, "JPEG: %ux%u", jdec.width, jdec.height);

    res = jd_decomp(&jdec, tjpgd_output, 0); // scale=0 → 1:1
    heap_caps_free(work);
    heap_caps_free(dl.buf);

    if (res != JDR_OK) {
        ESP_LOGE(TAG, "jd_decomp failed: %d", res);
        heap_caps_free(fb);
        goto done;
    }

    ESP_LOGI(TAG, "JPEG decoded %ux%u → RGB565", jdec.width, jdec.height);

    // Hand framebuffer to display (display takes ownership)
    display_set_scenario(fb, jdec.width, jdec.height);

done:
    // Signal that all image downloads are complete — MQTT can now safely connect
    xEventGroupSetBits(g_events, EVT_IMAGES_DONE);
    vTaskDelete(NULL);
}

// Use PSRAM stack to keep internal SRAM free for TLS operations
static StaticTask_t s_scenario_tcb;

void scenario_img_start(void)
{
    StackType_t *stack = heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stack) {
        xTaskCreateStaticPinnedToCore(scenario_task, "scenario_dl",
            8192 / sizeof(StackType_t), NULL, 3, stack, &s_scenario_tcb, 1);
    }
}
