#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * D4184 气泵驱动
 *
 * 接线：
 *   D4184 正(+)  → 12V+
 *   D4184 负(-)  → 12V GND
 *   D4184 PWM   → ESP32 GPIO7（气泵）/ GPIO21（电磁阀）
 *   D4184 GND   → ESP32 GND（共地！）
 *   D4184 LOAD  → 气泵负极 / 电磁阀负极
 *   气泵正极     → 12V+（直连）
 *   电磁阀正极   → 12V+（直连）
 */

// ── 引脚定义（可改） ──────────────────────────────
#define PUMP_PIN  7
#define VALVE_PIN 21

// ── 安全参数 ─────────────────────────────────────
#define PUMP_MAX_RUN_MS      5000   // 气泵最长连续运行时间（防过热）
#define PUMP_COOLDOWN_MS     3000   // 气泵最短冷却间隔
#define VALVE_RELEASE_MS     5000   // 紧急泄气时阀保持打开时间
#define PRESSURE_MAX_KPA     5.0f   // 气囊压力上限（kPa），配合 MCP5010DP

// ── 公开 API ─────────────────────────────────────

/** 初始化 GPIO，上电默认关。 */
void pump_driver_init(void);

/** 开气泵充气。返回 false 表示被安全限制拦截。 */
bool pump_start(void);

/** 关气泵。 */
void pump_stop(void);

/** 开泄气阀放气。 */
bool valve_open(void);

/** 关泄气阀保压。 */
void valve_close(void);

/** 紧急泄气：停泵 + 开阀 N 秒（由 ESP32 本地状态机调用，不经过云端 AI）。 */
void emergency_release(void);

/** 清除冷却计时（调试/急停后手动重置）。 */
void pump_clear_cooldown(void);

/** 气泵是否正在运行。 */
bool pump_is_running(void);

/** 泄气阀是否打开。 */
bool valve_is_open(void);

/** 气泵本次已运行时间（ms），外部压力闭环用。 */
uint32_t pump_run_ms(void);

#ifdef __cplusplus
}
#endif
