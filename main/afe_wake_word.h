#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 唤醒词检测回调 — 在 AFE fetch task 上下文中调用，不应阻塞太久 */
typedef void (*wake_word_callback_t)(void);

/**
 * 初始化 AFE 唤醒词管线:
 *   加载 flash model 分区 → 创建 AFE 实例 → 启动 feed/fetch 双任务
 * 返回 0 成功，-1 失败
 */
int afe_wake_word_init(wake_word_callback_t cb);

/* ── 录音接口（从 AFE 降噪音频输出中采集）────────── */

/** 开始录音，最多 max_samples 个 mono sample，会在说话后静音一段时间自动结束 */
void afe_capture_start(int max_samples);

/** 取回录音数据 + 实际 sample 数，调用者负责 free(3) */
int16_t *afe_capture_finish(int *out_samples);
int afe_capture_samples(void);
int afe_capture_read_from(int sample_offset, int16_t *out, int max_samples);

/** 录音是否完成（已满、超时无语音、或说话结束） */
bool afe_capture_is_done(void);

bool afe_capture_seen_speech(void);

/** 本轮录音是否被 VAD 判定包含语音 */
bool afe_capture_had_speech(void);

#ifdef __cplusplus
}
#endif
