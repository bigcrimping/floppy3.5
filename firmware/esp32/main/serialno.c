#include "serialno.h"
#include "wallclock.h"
#include "crc16.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "serial";

#define REC_OFF   0
#define REC_VER   1
#define REC_HDR   4                          // 'S' 'N' ver len
#define REC_MAX   (REC_HDR + SERIALNO_MAX + 2)

_Static_assert(REC_OFF + REC_MAX <= WALLCLOCK_EE_SIZE, "record exceeds RTC user EEPROM");

static char s_sn[SERIALNO_MAX + 1];

static void fail(char *err, size_t errsz, const char *msg)
{
    if (err && errsz) snprintf(err, errsz, "%s", msg);
}

static bool valid_char(char c)
{
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
}

static bool valid_sn(const char *sn, size_t len)
{
    if (len == 0 || len > SERIALNO_MAX) return false;
    for (size_t i = 0; i < len; i++) if (!valid_char(sn[i])) return false;
    return true;
}

// Read and check the record; on success copy the number into out (NUL-terminated).
static bool read_record(char *out)
{
    uint8_t rec[REC_MAX];
    if (!wallclock_ee_read(REC_OFF, rec, REC_HDR)) return false;
    if (rec[0] != 'S' || rec[1] != 'N' || rec[2] != REC_VER) return false;
    size_t len = rec[3];
    if (len == 0 || len > SERIALNO_MAX) return false;
    if (!wallclock_ee_read(REC_OFF + REC_HDR, rec + REC_HDR, len + 2)) return false;
    uint16_t crc = crc16_ccitt(0xFFFF, rec, REC_HDR + len);
    if (rec[REC_HDR + len] != (crc >> 8) || rec[REC_HDR + len + 1] != (crc & 0xFF)) return false;
    if (!valid_sn((const char *)rec + REC_HDR, len)) return false;
    memcpy(out, rec + REC_HDR, len);
    out[len] = '\0';
    return true;
}

void serialno_init(void)
{
    if (!wallclock_rtc_present()) {
        ESP_LOGW(TAG, "no RTC — serial number unavailable");
        return;
    }
    if (read_record(s_sn)) {
        ESP_LOGI(TAG, "serial number %s", s_sn);
        return;
    }
    s_sn[0] = '\0';
    uint8_t magic[2];
    if (wallclock_ee_read(REC_OFF, magic, 2) && magic[0] == 'S' && magic[1] == 'N')
        ESP_LOGE(TAG, "serial record in RTC EEPROM is damaged — set it again");
    else
        ESP_LOGW(TAG, "serial number not set (POST /api/serial)");
}

const char *serialno_get(void) { return s_sn; }

esp_err_t serialno_set(const char *sn, bool force, char *err, size_t errsz)
{
    size_t len = strlen(sn);
    if (!valid_sn(sn, len)) {
        fail(err, errsz, "serial must be 1-32 chars of A-Z a-z 0-9 - _ .");
        return ESP_ERR_INVALID_ARG;
    }
    if (!wallclock_rtc_present()) {
        fail(err, errsz, "RTC not present - nowhere to store the serial");
        return ESP_ERR_NOT_FOUND;
    }
    if (strcmp(sn, s_sn) == 0) return ESP_OK;
    if (s_sn[0] && !force) {
        char m[80];
        snprintf(m, sizeof(m), "serial already set to %s (force to change)", s_sn);
        fail(err, errsz, m);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t rec[REC_MAX];
    rec[0] = 'S'; rec[1] = 'N'; rec[2] = REC_VER; rec[3] = (uint8_t)len;
    memcpy(rec + REC_HDR, sn, len);
    uint16_t crc = crc16_ccitt(0xFFFF, rec, REC_HDR + len);
    rec[REC_HDR + len]     = crc >> 8;
    rec[REC_HDR + len + 1] = crc & 0xFF;

    char back[SERIALNO_MAX + 1];
    if (!wallclock_ee_write(REC_OFF, rec, REC_HDR + len + 2) ||
        !read_record(back) || strcmp(back, sn) != 0) {
        fail(err, errsz, "RTC EEPROM write did not verify");
        ESP_LOGE(TAG, "serial write to RTC EEPROM failed verification");
        return ESP_FAIL;
    }
    if (s_sn[0]) ESP_LOGW(TAG, "serial number changed %s -> %s", s_sn, sn);
    else         ESP_LOGI(TAG, "serial number set to %s", sn);
    memcpy(s_sn, sn, len + 1);
    return ESP_OK;
}
