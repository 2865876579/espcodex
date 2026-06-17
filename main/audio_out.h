#pragma once

#include <stdint.h>
#include <stddef.h>
#include "driver/i2s_std.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 I2S 输出（接 MAX98357A）。
 * 同时创建 RX 通道供麦克风使用（共享 BCLK/LRC）。
 * 采样率固定 16kHz / 16bit / 立体声。
 */
void audio_out_init(void);

/** 写 PCM 数据到 I2S 输出（阻塞直到写完）。 */
void audio_out_write(const uint8_t *data, size_t len);

/** 播放测试正弦音（440Hz, 1秒）。 */
void audio_out_play_test_tone(void);

/** 获取 I2S RX 通道句柄（供 audio_in 使用）。 */
i2s_chan_handle_t audio_out_get_rx_chan(void);

#ifdef __cplusplus
}
#endif
