#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Initialize I2S microphone (INMP441 on GPIO4, shared BCLK/LRC with speaker). */
void audio_in_init(void);

/**
 * Read mono PCM samples from microphone.
 * @param buf    output buffer (mono int16)
 * @param samples number of samples to read
 * @return actual number of samples read
 */
int audio_in_read(int16_t *buf, int samples);

#ifdef __cplusplus
}
#endif
