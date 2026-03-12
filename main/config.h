#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CONFIG_SSID_MAX     64
#define CONFIG_PASS_MAX     64
#define CONFIG_APIKEY_MAX   64
#define CONFIG_DOLL_BODY_ID_MAX  64
#define CONFIG_DOLL_ID_MAX       64
#define CONFIG_SERVER_MAX        128
#define CONFIG_MQTT_URL_MAX      128
#define CONFIG_CHAT_ID_MAX       64
#define CONFIG_STREAM_REC_MAX    128
#define CONFIG_STREAM_PLAYER_MAX 128
#define CONFIG_AVATAR_ID_MAX     64
#define CONFIG_SCENARIO_ID_MAX   64

typedef struct {
    char ssid[CONFIG_SSID_MAX];
    char password[CONFIG_PASS_MAX];
    char apikey[CONFIG_APIKEY_MAX];
    char doll_body_id[CONFIG_DOLL_BODY_ID_MAX]; // flashed at build time — identifies the body model
    char doll_id[CONFIG_DOLL_ID_MAX];            // obtained from backend after POST /dolls
    char server_url[CONFIG_SERVER_MAX];
    char mqtt_url[CONFIG_MQTT_URL_MAX];
    char chat_id[CONFIG_CHAT_ID_MAX];   // populated at runtime from GET /dolls/:id
    char avatar_id[CONFIG_AVATAR_ID_MAX]; // populated at runtime from GET /dolls/:id?include=chat
    char scenario_id[CONFIG_SCENARIO_ID_MAX]; // populated at runtime from GET /dolls/:id?include=chat
    char stream_recorder_url[CONFIG_STREAM_REC_MAX];
    char stream_player_url[CONFIG_STREAM_PLAYER_MAX];
    bool provisioned;
} doll_config_t;

extern doll_config_t g_config;
