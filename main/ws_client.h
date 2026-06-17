#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start WebSocket client, connect to cloud server.
 * Sends {"type":"text","text":"..."} messages.
 * Receives tts_audio_chunk PCM and plays through I2S.
 */
void ws_client_start(const char *uri);
void ws_client_restart(void);

/**
 * Send a text message to the cloud LLM.
 * Wraps in {"type":"text","text":"..."} JSON.
 */
bool ws_client_send_text(const char *text);

/**
 * Send a pre-built JSON string as-is (no wrapping).
 * Use for audio, control messages etc. that are already valid JSON.
 */
bool ws_client_send_raw(const char *json_str);

bool ws_client_is_connected(void);
bool ws_client_is_tts_active(void);
void ws_client_clear_events(void);
bool ws_client_consume_turn_done(void);
bool ws_client_consume_dialog_end(void);

#ifdef __cplusplus
}
#endif
