#pragma once

// Secrets are injected at build time from environment variables.
// Copy .env.example to .env, fill in your values, then:
//   set -a && source .env && set +a && idf.py build
// Note: plain `source .env` does NOT export vars to cmake subprocesses.

#ifndef SECRET_SSID
#define SECRET_SSID       "your_wifi_ssid"
#endif

#ifndef SECRET_PASSWORD
#define SECRET_PASSWORD   "your_wifi_password"
#endif

#ifndef SECRET_APIKEY
#define SECRET_APIKEY     "your_api_key"
#endif

#define SECRET_DOLL_BODY_ID    "b37e3f13-ab0a-4373-a6fa-eb9dc5ef1586"

#ifndef SECRET_SERVER_URL
#define SECRET_SERVER_URL "https://api.cipherdolls.com"
#endif

#ifndef SECRET_MQTT_URL
#define SECRET_MQTT_URL "mqtt://api.cipherdolls.com:1883"
#endif

#ifndef SECRET_STREAM_RECORDER_URL
#define SECRET_STREAM_RECORDER_URL "https://stream-recorder.cipherdolls.com"
#endif

#ifndef SECRET_STREAM_PLAYER_URL
#define SECRET_STREAM_PLAYER_URL "https://stream-player.cipherdolls.com"
#endif
