#pragma once

// Connect to the MQTT broker and start the metrics publish loop.
// Internally waits for EVT_DOLL_READY before connecting, so it is safe
// to call this immediately after http_sync_doll().
void mqtt_start(void);
void mqtt_stop(void);
void mqtt_reconnect(void);
