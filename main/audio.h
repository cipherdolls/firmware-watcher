#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include <stdbool.h>

void audio_init(void);
void audio_play_message(const char *message_id);
void audio_stop(void);
void audio_speaker_mute(void);
void audio_speaker_unmute(void);

// Play raw PCM stream from a StreamBuffer (fed by stream_player module).
// Expects 16-bit signed LE, 24 kHz, mono.  Expands to stereo for I2S.
// Blocks until stream is fully played or audio_stop() is called.
void audio_stream_play(StreamBufferHandle_t stream,
                       volatile bool *stream_active,
                       volatile bool *stream_error,
                       uint8_t first_byte);
