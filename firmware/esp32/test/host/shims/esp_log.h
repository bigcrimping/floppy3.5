// Host-test shim for esp_log.h — routes logs to stdout (stderr trips the test
// runner's ErrorActionPreference=Stop under PowerShell).
#pragma once
#include <stdio.h>
#define ESP_LOGE(tag, fmt, ...) fprintf(stdout, "E (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stdout, "W (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fprintf(stdout, "I (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) do {} while (0)
#define ESP_LOGV(tag, fmt, ...) do {} while (0)
