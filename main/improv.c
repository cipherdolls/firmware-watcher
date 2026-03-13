#include "improv.h"
#include "config.h"
#include "config_store.h"
#include "wifi_mgr.h"
#include "events.h"
#include "led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "improv";

// ─── Improv Serial protocol constants ───────────────────────────────────────

static const uint8_t IMPROV_HEADER[6] = {'I', 'M', 'P', 'R', 'O', 'V'};
#define IMPROV_VERSION          0x01

// Packet types
#define TYPE_CURRENT_STATE      0x01
#define TYPE_ERROR_STATE        0x02
#define TYPE_RPC                0x03
#define TYPE_RPC_RESPONSE       0x04

// States
#define STATE_AUTHORIZED        0x02
#define STATE_PROVISIONING      0x03
#define STATE_PROVISIONED       0x04

// Errors
#define ERROR_NONE              0x00
#define ERROR_INVALID_RPC       0x01
#define ERROR_UNKNOWN_RPC       0x02
#define ERROR_UNABLE_TO_CONNECT 0x03

// RPC commands
#define CMD_WIFI_SETTINGS       0x01
#define CMD_GET_CURRENT_STATE   0x02
#define CMD_GET_DEVICE_INFO     0x03
#define CMD_GET_WIFI_NETWORKS   0x04

#define IMPROV_MAX_PACKET       256
#define UART_NUM                UART_NUM_0
#define WIFI_CONNECT_TIMEOUT_MS 30000

// ─── Transmit helpers ───────────────────────────────────────────────────────

static void improv_send_packet(uint8_t type, const uint8_t *data, uint8_t data_len)
{
    uint8_t pkt[IMPROV_MAX_PACKET];
    size_t pos = 0;

    memcpy(pkt, IMPROV_HEADER, 6);
    pos = 6;
    pkt[pos++] = IMPROV_VERSION;
    pkt[pos++] = type;
    pkt[pos++] = data_len;
    if (data_len > 0) {
        memcpy(&pkt[pos], data, data_len);
        pos += data_len;
    }

    // Checksum: sum of all bytes before checksum
    uint8_t cksum = 0;
    for (size_t i = 0; i < pos; i++) cksum += pkt[i];
    pkt[pos++] = cksum;
    pkt[pos++] = '\n';  // trailing newline (like ESPHome)

    uart_write_bytes(UART_NUM, pkt, pos);
}

static void improv_send_state(uint8_t state)
{
    improv_send_packet(TYPE_CURRENT_STATE, &state, 1);
}

static void improv_send_error(uint8_t error)
{
    improv_send_packet(TYPE_ERROR_STATE, &error, 1);
}

// Build an RPC response with a list of length-prefixed strings
static void improv_send_rpc_response(uint8_t cmd, const char **strings, int count)
{
    uint8_t buf[IMPROV_MAX_PACKET];
    size_t pos = 0;

    buf[pos++] = cmd;
    // Reserve byte for inner length, fill in later
    size_t len_pos = pos++;

    for (int i = 0; i < count; i++) {
        size_t slen = strlen(strings[i]);
        if (slen > 127) slen = 127;
        buf[pos++] = (uint8_t)slen;
        memcpy(&buf[pos], strings[i], slen);
        pos += slen;
    }

    buf[len_pos] = (uint8_t)(pos - len_pos - 1);
    improv_send_packet(TYPE_RPC_RESPONSE, buf, (uint8_t)pos);
}

// ─── RPC handlers ───────────────────────────────────────────────────────────

static void handle_wifi_settings(const uint8_t *data, uint8_t len)
{
    if (len < 2) {
        improv_send_error(ERROR_INVALID_RPC);
        return;
    }

    uint8_t ssid_len = data[0];
    if (1 + ssid_len >= len) {
        improv_send_error(ERROR_INVALID_RPC);
        return;
    }
    uint8_t pass_len = data[1 + ssid_len];
    if (1 + ssid_len + 1 + pass_len > len) {
        improv_send_error(ERROR_INVALID_RPC);
        return;
    }

    char ssid[64] = {0};
    char pass[64] = {0};
    memcpy(ssid, &data[1], ssid_len);
    memcpy(pass, &data[1 + ssid_len + 1], pass_len);

    ESP_LOGI(TAG, "Improv: WiFi credentials received, ssid='%s'", ssid);

    // Report provisioning state
    improv_send_error(ERROR_NONE);
    improv_send_state(STATE_PROVISIONING);

    // Disconnect if currently connected
    if (wifi_mgr_is_connected()) {
        wifi_mgr_disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Clear stale event bits
    xEventGroupClearBits(g_events,
        EVT_WIFI_DISCONNECTED | EVT_WIFI_CONNECTED | EVT_WIFI_GOT_IP);

    wifi_mgr_connect(ssid, pass);

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(g_events,
        EVT_WIFI_GOT_IP | EVT_WIFI_DISCONNECTED,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & EVT_WIFI_GOT_IP) {
        ESP_LOGI(TAG, "Improv: WiFi connected successfully");

        // Save credentials
        strlcpy(g_config.ssid, ssid, sizeof(g_config.ssid));
        strlcpy(g_config.password, pass, sizeof(g_config.password));
        g_config.provisioned = true;
        config_store_save();

        improv_send_state(STATE_PROVISIONED);

        // RPC response: redirect URL (empty list is fine)
        const char *urls[] = {""};
        improv_send_rpc_response(CMD_WIFI_SETTINGS, urls, 1);
    } else {
        ESP_LOGW(TAG, "Improv: WiFi connection failed");
        improv_send_error(ERROR_UNABLE_TO_CONNECT);
        improv_send_state(STATE_AUTHORIZED);
    }
}

static void handle_get_current_state(void)
{
    if (wifi_mgr_is_connected()) {
        improv_send_state(STATE_PROVISIONED);
    } else {
        improv_send_state(STATE_AUTHORIZED);
    }
}

static void handle_get_device_info(void)
{
    const char *info[] = {
        "CipherDolls",       // firmware name
        "1.0.0",             // firmware version
        "ESP32-S3",          // chip
        "DollBody",          // device name
    };
    improv_send_rpc_response(CMD_GET_DEVICE_INFO, info, 4);
}

static void handle_get_wifi_networks(void)
{
    wifi_ap_info_t *aps = NULL;
    int count = wifi_mgr_scan(&aps);

    for (int i = 0; i < count; i++) {
        char rssi_str[8];
        snprintf(rssi_str, sizeof(rssi_str), "%d", aps[i].rssi);
        const char *net[] = {
            aps[i].ssid,
            rssi_str,
            (aps[i].authmode != WIFI_AUTH_OPEN) ? "YES" : "NO",
        };
        improv_send_rpc_response(CMD_GET_WIFI_NETWORKS, net, 3);
    }

    // Empty response signals end of scan
    improv_send_rpc_response(CMD_GET_WIFI_NETWORKS, NULL, 0);

    if (aps) free(aps);
}

static void handle_rpc(const uint8_t *data, uint8_t data_len)
{
    if (data_len < 2) {
        improv_send_error(ERROR_INVALID_RPC);
        return;
    }

    uint8_t cmd = data[0];
    uint8_t inner_len = data[1];

    // Reset error on any RPC
    improv_send_error(ERROR_NONE);

    switch (cmd) {
    case CMD_WIFI_SETTINGS:
        handle_wifi_settings(&data[2], inner_len);
        break;
    case CMD_GET_CURRENT_STATE:
        handle_get_current_state();
        break;
    case CMD_GET_DEVICE_INFO:
        handle_get_device_info();
        break;
    case CMD_GET_WIFI_NETWORKS:
        handle_get_wifi_networks();
        break;
    default:
        ESP_LOGW(TAG, "Unknown RPC command 0x%02x", cmd);
        improv_send_error(ERROR_UNKNOWN_RPC);
        break;
    }
}

// ─── Custom serial commands (line-based, alongside Improv) ──────────────────
// Format: "COMMAND:value\n"

#define LINE_BUF_SIZE 256

static void handle_serial_line(const char *line)
{
    if (strncmp(line, "APIKEY:", 7) == 0) {
        const char *key = line + 7;
        if (strlen(key) > 0) {
            strlcpy(g_config.apikey, key, sizeof(g_config.apikey));
            config_store_save();
            ESP_LOGI(TAG, "API key set via serial, rebooting...");
            uart_write_bytes(UART_NUM, "OK:APIKEY\n", 10);
            uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            uart_write_bytes(UART_NUM, "ERR:APIKEY_EMPTY\n", 17);
        }
    } else if (strncmp(line, "VOLUME:", 7) == 0) {
        int vol = atoi(line + 7);
        if (vol >= 0 && vol <= 100) {
            g_config.speaker_volume = (uint8_t)vol;
            config_store_save();
            ESP_LOGI(TAG, "Volume set to %d via serial", vol);
            uart_write_bytes(UART_NUM, "OK:VOLUME\n", 10);
        } else {
            uart_write_bytes(UART_NUM, "ERR:VOLUME_RANGE\n", 17);
        }
    } else if (strcmp(line, "GETCONFIG") == 0) {
        char buf[256];
        int len = snprintf(buf, sizeof(buf),
            "CONFIG:ssid=%s,apikey=%s,volume=%d,provisioned=%d,doll_body_id=%s,doll_id=%s\n",
            g_config.ssid,
            strlen(g_config.apikey) > 0 ? "***" : "",
            g_config.speaker_volume,
            g_config.provisioned ? 1 : 0,
            g_config.doll_body_id,
            g_config.doll_id);
        uart_write_bytes(UART_NUM, buf, len);
    } else if (strcmp(line, "CLEARCONFIG") == 0) {
        config_store_clear();
        memset(g_config.ssid, 0, sizeof(g_config.ssid));
        memset(g_config.password, 0, sizeof(g_config.password));
        memset(g_config.apikey, 0, sizeof(g_config.apikey));
        g_config.provisioned = false;
        ESP_LOGI(TAG, "Config cleared via serial");
        uart_write_bytes(UART_NUM, "OK:CLEARCONFIG\n", 15);
    }
}

// ─── Packet parser (byte-by-byte state machine) ────────────────────────────

typedef enum {
    PARSE_HEADER,   // matching 'I','M','P','R','O','V'
    PARSE_VERSION,
    PARSE_TYPE,
    PARSE_LENGTH,
    PARSE_DATA,
    PARSE_CHECKSUM,
} parse_state_t;

void improv_task_fn(void *pvParameter)
{
    // Install UART driver on UART0 (console) — ESP_LOG already uses it,
    // but we need the driver for uart_read_bytes()
    const int rx_buf = 512;
    uart_driver_install(UART_NUM, rx_buf, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "Improv Serial listener started");

    parse_state_t state = PARSE_HEADER;
    uint8_t header_pos = 0;
    uint8_t pkt_type = 0;
    uint8_t pkt_len = 0;
    uint8_t pkt_data[IMPROV_MAX_PACKET];
    uint8_t pkt_pos = 0;
    uint8_t running_sum = 0;

    // Line buffer for custom serial commands
    char line_buf[LINE_BUF_SIZE];
    size_t line_pos = 0;

    while (1) {
        uint8_t byte;
        int n = uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        switch (state) {
        case PARSE_HEADER:
            if (byte == IMPROV_HEADER[header_pos]) {
                if (header_pos == 0) running_sum = 0;
                running_sum += byte;
                header_pos++;
                if (header_pos == 6) {
                    state = PARSE_VERSION;
                    line_pos = 0;  // clear line buffer — this is Improv
                }
            } else {
                // Not Improv — accumulate into line buffer for custom commands
                header_pos = 0;
                if (byte == '\n' || byte == '\r') {
                    if (line_pos > 0) {
                        line_buf[line_pos] = '\0';
                        handle_serial_line(line_buf);
                        line_pos = 0;
                    }
                } else if (line_pos < LINE_BUF_SIZE - 1) {
                    line_buf[line_pos++] = (char)byte;
                }
            }
            break;

        case PARSE_VERSION:
            running_sum += byte;
            if (byte == IMPROV_VERSION) {
                state = PARSE_TYPE;
            } else {
                header_pos = 0;
                state = PARSE_HEADER;
            }
            break;

        case PARSE_TYPE:
            running_sum += byte;
            pkt_type = byte;
            state = PARSE_LENGTH;
            break;

        case PARSE_LENGTH:
            running_sum += byte;
            pkt_len = byte;
            pkt_pos = 0;
            if (pkt_len == 0) {
                state = PARSE_CHECKSUM;
            } else {
                state = PARSE_DATA;
            }
            break;

        case PARSE_DATA:
            running_sum += byte;
            if (pkt_pos < sizeof(pkt_data)) {
                pkt_data[pkt_pos] = byte;
            }
            pkt_pos++;
            if (pkt_pos >= pkt_len) {
                state = PARSE_CHECKSUM;
            }
            break;

        case PARSE_CHECKSUM:
            if (byte == (running_sum & 0xFF)) {
                // Valid packet
                if (pkt_type == TYPE_RPC) {
                    handle_rpc(pkt_data, pkt_len);
                }
            } else {
                ESP_LOGW(TAG, "Improv checksum mismatch: got 0x%02x expected 0x%02x",
                         byte, running_sum & 0xFF);
            }
            header_pos = 0;
            state = PARSE_HEADER;
            break;
        }
    }
}
