# SenseCAP Watcher — Interaction Flow

## Boot

WiFi connect, HTTP sync doll, download avatar/scenario images, audio init.
MQTT connect, stream-player WS connect.
Speaker and microphone muted.
Enters READY mode.

## Ready Mode

- Display: ON — avatar, scenario, "Tap to talk"
- LED: OFF
- Speaker: muted
- Microphone: off
- MQTT: connected
- Stream-player WS: connected
- Stream-recorder WS: disconnected
- Tap screen: enter Conversation Mode
- Knob press: enter Standby Mode

## Standby Mode

- Display: OFF
- LED: white blink every 5 seconds
- Audio: muted
- All connections disconnected (MQTT, stream-player)
- Knob press: reconnect MQTT + stream-player, back to Ready Mode

## Conversation Mode

On enter: unmute speaker, connect stream-recorder WS.

### Listening

- Display: "Listening..."
- LED: dim red, pulsing with ambient sound level
- Microphone: ON (16kHz mono)
- VAD monitors for speech, pre-speech buffer captures last 300ms
- Speech detected: go to Recording
- Tap screen or knob: exit conversation

### Recording

- LED: red, pulsing with voice volume
- Sends recording_start JSON to stream-recorder
- Sends WAV header + PCM chunks over WebSocket
- 1.5s silence: stop recording, go to Sending
- Tap screen or knob: stop recording, exit conversation

### Sending

- Drains remaining audio from ring buffer
- Sends recording_end JSON to stream-recorder
- Goes to Waiting

### Waiting

- LED: white blink
- Waiting for tts_start from stream-player
- TTS starts: go to Playing
- Tap screen or knob: exit conversation
- 30s timeout: back to Listening

### Playing

- LED: blue, pulsing with playback volume
- PCM from stream-player (24kHz, 16-bit, mono) expanded to stereo, played via I2S
- Playback finished: back to Listening (reconnects stream-player if dropped)
- Tap screen or knob: stop playback, exit conversation

### Exit

Stop microphone, stop playback, mute speaker.
Disconnect stream-recorder WS.
Back to Ready Mode (MQTT + stream-player stay connected).
