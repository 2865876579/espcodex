#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 连接 Wi-Fi（阻塞直到连上或重试耗尽）。
 * 成功返回 0，串口会打印获取到的 IP。
 */
int wifi_connect(const char *ssid, const char *password);

#ifdef __cplusplus
}
#endif
