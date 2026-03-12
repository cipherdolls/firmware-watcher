#include "config.h"
#include <string.h>

doll_config_t g_config = {
    .ssid        = "",
    .password    = "",
    .apikey      = "",
    .doll_body_id = "",
    .doll_id      = "",
    .server_url   = "https://api.cipherdolls.com",
    .stream_recorder_url = "https://stream-recorder.cipherdolls.com",
    .stream_player_url   = "https://stream-player.cipherdolls.com",
    .provisioned = false,
};
