#pragma once

void stream_player_init(void);
void stream_player_pause(void);   // disconnect WS to free TLS memory for recording
void stream_player_resume(void);  // reconnect WS after recording
