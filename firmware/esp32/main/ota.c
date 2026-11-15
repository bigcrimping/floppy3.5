// Two OTA slots, no factory. An image confirms itself only after WiFi and
// the controller link are up, so one that breaks either is rolled back.
// Uploads stream straight to flash and are refused while a drive or SD job
// holds off updates (ota_hold).
#include "ota.h"

#include <string.h>
#include <stdio.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "ota";

static esp_ota_handle_t s_handle;
static const esp_partition_t *s_target;
static bool s_active;
static bool s_done;                    // image accepted, reboot pending
static size_t s_written, s_expected;
static int s_holds;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

bool ota_in_progress(void) { return s_active; }

int ota_progress_pct(void)
{
    if (s_done) return 100;
    if (!s_active) return -1;
    size_t exp = s_expected, w = s_written;
    if (!exp) return 0;
    int p = (int)((uint64_t)w * 100 / exp);
    return p > 99 ? 99 : p;            // 100 only once ota_end() accepts it
}

bool ota_hold(void)
{
    taskENTER_CRITICAL(&s_mux);
    bool ok = !s_active;
    if (ok) s_holds++;
    taskEXIT_CRITICAL(&s_mux);
    return ok;
}

void ota_unhold(void)
{
    taskENTER_CRITICAL(&s_mux);
    if (s_holds) s_holds--;
    taskEXIT_CRITICAL(&s_mux);
}

static void fail(char *err, size_t errsz, const char *msg)
{
    if (err && errsz) snprintf(err, errsz, "%s", msg);
    ESP_LOGE(TAG, "%s", msg);
}

void ota_mark_valid_if_pending(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(run, &st) != ESP_OK)
        return;
    if (st != ESP_OTA_IMG_PENDING_VERIFY)
        return;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
        ESP_LOGW(TAG, "new firmware confirmed good (%s) — rollback cancelled",
                 run->label);
    else
        ESP_LOGE(TAG, "could not confirm this image; it will roll back");
}

bool ota_pending_verify(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(run, &st) != ESP_OK)
        return false;
    return st == ESP_OTA_IMG_PENDING_VERIFY;
}

const char *ota_running_label(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    return run ? run->label : "?";
}

esp_err_t ota_begin(size_t total_len, char *err, size_t errsz)
{
    // Claim s_active and check the holds in one step, so a job can't start
    // between the two.
    taskENTER_CRITICAL(&s_mux);
    bool was_active = s_active, held = s_holds > 0;
    if (!was_active && !held) s_active = true;
    taskEXIT_CRITICAL(&s_mux);
    if (was_active) {
        fail(err, errsz, "an update is already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    if (held) {
        fail(err, errsz, "a disk or SD job is running — try again when it finishes");
        return ESP_ERR_INVALID_STATE;
    }

    s_target = esp_ota_get_next_update_partition(NULL);
    if (!s_target) {
        s_active = false;
        fail(err, errsz, "no OTA slot — device flashed with the old "
                         "factory-only partition table (needs one USB flash)");
        return ESP_ERR_NOT_FOUND;
    }
    if (total_len && total_len > s_target->size) {
        char m[96];
        snprintf(m, sizeof(m), "image is %u KB, slot holds %u KB",
                 (unsigned)(total_len / 1024), (unsigned)(s_target->size / 1024));
        s_active = false;
        fail(err, errsz, m);
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t e = esp_ota_begin(s_target, total_len ? total_len : OTA_SIZE_UNKNOWN,
                                &s_handle);
    if (e != ESP_OK) {
        s_active = false;
        fail(err, errsz, "could not open the OTA slot");
        return e;
    }
    s_written = 0;
    s_expected = total_len;
    ESP_LOGW(TAG, "OTA started -> %s (%u KB expected)", s_target->label,
             (unsigned)(total_len / 1024));
    return ESP_OK;
}

esp_err_t ota_write(const void *data, size_t len, char *err, size_t errsz)
{
    if (!s_active) {
        fail(err, errsz, "no update in progress");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t e = esp_ota_write(s_handle, data, len);
    if (e != ESP_OK) {
        fail(err, errsz, "flash write failed");
        ota_abort();
        return e;
    }
    s_written += len;
    return ESP_OK;
}

esp_err_t ota_end(char *err, size_t errsz)
{
    if (!s_active) {
        fail(err, errsz, "no update in progress");
        return ESP_ERR_INVALID_STATE;
    }
    s_active = false;

    // Validates the image before the boot slot is touched.
    esp_err_t e = esp_ota_end(s_handle);
    if (e != ESP_OK) {
        fail(err, errsz, e == ESP_ERR_OTA_VALIDATE_FAILED
                             ? "image failed validation (truncated or not an "
                               "ESP32-S3 app?)"
                             : "OTA finalise failed");
        return e;
    }
    if (s_expected && s_written != s_expected) {
        fail(err, errsz, "upload was short");
        return ESP_ERR_INVALID_SIZE;
    }

    e = esp_ota_set_boot_partition(s_target);
    if (e != ESP_OK) {
        fail(err, errsz, "could not set the boot slot");
        return e;
    }
    s_done = true;
    ESP_LOGW(TAG, "OTA complete: %u KB into %s — reboot to run it",
             (unsigned)(s_written / 1024), s_target->label);
    return ESP_OK;
}

void ota_abort(void)
{
    if (!s_active) return;
    esp_ota_abort(s_handle);
    s_active = false;
    ESP_LOGW(TAG, "OTA aborted after %u KB", (unsigned)(s_written / 1024));
}
