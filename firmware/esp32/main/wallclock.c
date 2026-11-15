// RV-3028-C7 driver. Register map and EEPROM sequence from the Linux
// rtc-rv3028.c. VBACKUP is wired to the LiPo (R68), so writes to the backup
// register touch only BSM (bits 3:2) and never TCE, the trickle charger.
#include "wallclock.h"
#include "app_config.h"
#include "power.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "rtc";

#define RV3028_I2C_ADDR   0x52

#define REG_SEC           0x00   // 0x00..0x06: sec,min,hour,wday,day,month,year
#define REG_STATUS        0x0E
#define REG_CTRL1         0x0F
#define REG_EEPROM_ADDR   0x25
#define REG_EEPROM_DATA   0x26
#define REG_EEPROM_CMD    0x27
#define REG_BACKUP        0x37

#define STATUS_PORF       0x01   // power-on reset: the held time is meaningless
#define STATUS_EEBUSY     0x80
#define CTRL1_EERD        0x08   // 1 = automatic EEPROM refresh disabled

#define EEPROM_CMD_UPDATE 0x11
#define EEPROM_CMD_WRITE  0x21   // one byte EEDATA -> EEPROM[EEADDR]
#define EEPROM_CMD_READ   0x22   // one byte EEPROM[EEADDR] -> EEDATA

#define BACKUP_BSM_MASK   0x0C   // bits 3:2
#define BACKUP_BSM_LSM    0x0C   // level switching mode (BSM = 0b11)

#define I2C_TMO_MS        100

static i2c_master_dev_handle_t s_dev;
static bool      s_present;
static wallclock_src_t s_src = WALLCLOCK_SRC_UNSET;

// Latched in wallclock_init().
static wallclock_boot_t s_boot = WALLCLOCK_BOOT_ABSENT;
static uint8_t s_backup_reg = 0xFF;

// ---- raw register access ----

static bool rd(uint8_t reg, uint8_t *buf, size_t n)
{
    if (!s_dev) return false;
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, I2C_TMO_MS) == ESP_OK;
}

static bool wr(uint8_t reg, const uint8_t *buf, size_t n)
{
    if (!s_dev || n > 8) return false;
    uint8_t tx[9];
    tx[0] = reg;
    memcpy(tx + 1, buf, n);
    return i2c_master_transmit(s_dev, tx, n + 1, I2C_TMO_MS) == ESP_OK;
}

static bool wr8(uint8_t reg, uint8_t val) { return wr(reg, &val, 1); }

static bool update8(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    if (!rd(reg, &cur, 1)) return false;
    uint8_t next = (uint8_t)((cur & ~mask) | (val & mask));
    return next == cur ? true : wr8(reg, next);
}

// ---- BCD helpers ----
// Every address ACKs on this bus and returns 0xFF, which isn't valid BCD, so
// these double as the presence check.

static bool bcd_ok(uint8_t v)  { return (v & 0x0F) <= 9 && (v >> 4) <= 9; }
static int  bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t bin2bcd(int v)  { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// ---- time ----

// Days since 1970-01-01 (Howard Hinnant's days_from_civil). Avoids mktime()
// depending on TZ; newlib has no timegm().
static long days_from_civil(int y, int m, int d)
{
    y -= m <= 2;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const int yoe = (int)(y - era * 400);                      // 0..399
    const int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;      // 0..146096
    return era * 146097L + doe - 719468L;
}

static time_t tm_to_epoch_utc(const struct tm *t)
{
    long days = days_from_civil(t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
    return (time_t)(days * 86400L + t->tm_hour * 3600L + t->tm_min * 60L + t->tm_sec);
}

// The unused high bits aren't guaranteed zero.
static bool read_time(struct tm *out)
{
    uint8_t d[7];
    if (!rd(REG_SEC, d, sizeof(d))) return false;

    uint8_t sec = d[0] & 0x7f, min = d[1] & 0x7f, hour = d[2] & 0x3f;
    uint8_t day = d[4] & 0x3f, mon = d[5] & 0x1f, year = d[6];

    if (!bcd_ok(sec) || !bcd_ok(min) || !bcd_ok(hour) ||
        !bcd_ok(day) || !bcd_ok(mon) || !bcd_ok(year))
        return false;

    memset(out, 0, sizeof(*out));
    out->tm_sec  = bcd2bin(sec);
    out->tm_min  = bcd2bin(min);
    out->tm_hour = bcd2bin(hour);
    out->tm_mday = bcd2bin(day);
    out->tm_mon  = bcd2bin(mon) - 1;
    out->tm_year = bcd2bin(year) + 100;   // register is 00..99 = 2000..2099

    return out->tm_sec < 60 && out->tm_min < 60 && out->tm_hour < 24 &&
           out->tm_mday >= 1 && out->tm_mday <= 31 &&
           out->tm_mon  >= 0 && out->tm_mon  <= 11;
}

static bool write_time(const struct tm *t)
{
    uint8_t d[7] = {
        bin2bcd(t->tm_sec), bin2bcd(t->tm_min), bin2bcd(t->tm_hour),
        (uint8_t)t->tm_wday,                        // weekday is binary, not BCD
        bin2bcd(t->tm_mday), bin2bcd(t->tm_mon + 1),
        bin2bcd(t->tm_year - 100),
    };
    // Writing seconds also resets the sub-second prescaler.
    if (!wr(REG_SEC, d, sizeof(d))) return false;

    // Cleared PORF tells the next boot the time is trustworthy.
    return update8(REG_STATUS, STATUS_PORF, 0);
}

// ---- one-time EEPROM configuration ----

static bool eebusy_wait(int tmo_ms)
{
    for (int i = 0; i < tmo_ms; i += 10) {
        uint8_t s;
        if (!rd(REG_STATUS, &s, 1)) return false;
        if (!(s & STATUS_EEBUSY)) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

// Enable backup switchover (factory default is off) so the clock survives
// power-off. An EEPROM write, done only when the field is wrong. LSM, not DSM:
// a charged LiPo sits above 3V3, so DSM would always run from the battery.
static void ensure_backup_switchover(void)
{
    uint8_t bk;
    if (!rd(REG_BACKUP, &bk, 1)) {
        ESP_LOGW(TAG, "backup register unreadable — switchover not verified");
        return;
    }
    s_backup_reg = bk;
    if ((bk & BACKUP_BSM_MASK) == BACKUP_BSM_LSM) {
        ESP_LOGI(TAG, "backup switchover already LSM (backup reg 0x%02X)", bk);
        return;
    }

    ESP_LOGW(TAG, "backup switchover disabled (reg 0x%02X) — writing EEPROM once "
                  "to enable LSM; time would not survive power-off otherwise", bk);

    // Hold off auto-refresh, edit the RAM mirror, then commit with 0x00 +
    // UPDATE, or the part reverts on the next power-on.
    uint8_t c1;
    if (!rd(REG_CTRL1, &c1, 1)) return;
    bool had_eerd = (c1 & CTRL1_EERD) != 0;

    if (!had_eerd) {
        if (!update8(REG_CTRL1, CTRL1_EERD, CTRL1_EERD)) return;
        if (!eebusy_wait(200)) { ESP_LOGE(TAG, "EEbusy stuck entering EERD"); goto out; }
    }

    if (!update8(REG_BACKUP, BACKUP_BSM_MASK, BACKUP_BSM_LSM)) {
        ESP_LOGE(TAG, "backup register write failed");
        goto out;
    }
    if (!wr8(REG_EEPROM_CMD, 0x00) || !wr8(REG_EEPROM_CMD, EEPROM_CMD_UPDATE)) {
        ESP_LOGE(TAG, "EEPROM update command failed");
        goto out;
    }
    vTaskDelay(pdMS_TO_TICKS(70));          // datasheet: ~63 ms for a full update
    if (!eebusy_wait(200)) { ESP_LOGE(TAG, "EEbusy stuck after update"); goto out; }

    if (rd(REG_BACKUP, &bk, 1)) {
        s_backup_reg = bk;
        ESP_LOGI(TAG, "backup switchover -> LSM (reg now 0x%02X)", bk);
    }

out:
    if (!had_eerd) update8(REG_CTRL1, CTRL1_EERD, 0);
}

// ---- user EEPROM (0x00..0x2A) ----
//
// One byte at a time with auto-refresh held off. Only init and one httpd
// handler use it, so no lock.

static bool ee_begin(bool *had_eerd)
{
    uint8_t c1;
    *had_eerd = true;                   // unreadable -> leave CTRL1 alone in ee_end
    if (!rd(REG_CTRL1, &c1, 1)) return false;
    *had_eerd = (c1 & CTRL1_EERD) != 0;
    if (!*had_eerd && !update8(REG_CTRL1, CTRL1_EERD, CTRL1_EERD)) return false;
    return eebusy_wait(200);
}

static void ee_end(bool had_eerd)
{
    if (!had_eerd) update8(REG_CTRL1, CTRL1_EERD, 0);
}

bool wallclock_ee_read(uint8_t off, uint8_t *buf, size_t n)
{
    if (!s_present || off + n > WALLCLOCK_EE_SIZE) return false;
    bool had_eerd, ok = ee_begin(&had_eerd);
    for (size_t i = 0; ok && i < n; i++) {
        ok = wr8(REG_EEPROM_ADDR, (uint8_t)(off + i)) &&
             wr8(REG_EEPROM_CMD, 0x00) && wr8(REG_EEPROM_CMD, EEPROM_CMD_READ) &&
             eebusy_wait(50) && rd(REG_EEPROM_DATA, &buf[i], 1);
    }
    ee_end(had_eerd);
    return ok;
}

bool wallclock_ee_write(uint8_t off, const uint8_t *buf, size_t n)
{
    if (!s_present || off + n > WALLCLOCK_EE_SIZE) return false;
    bool had_eerd, ok = ee_begin(&had_eerd);
    for (size_t i = 0; ok && i < n; i++) {
        // Skip unchanged bytes (~16 ms and a write cycle each).
        uint8_t cur;
        ok = wr8(REG_EEPROM_ADDR, (uint8_t)(off + i)) &&
             wr8(REG_EEPROM_CMD, 0x00) && wr8(REG_EEPROM_CMD, EEPROM_CMD_READ) &&
             eebusy_wait(50) && rd(REG_EEPROM_DATA, &cur, 1);
        if (!ok || cur == buf[i]) continue;
        ok = wr8(REG_EEPROM_DATA, buf[i]) &&
             wr8(REG_EEPROM_CMD, 0x00) && wr8(REG_EEPROM_CMD, EEPROM_CMD_WRITE);
        if (ok) { vTaskDelay(pdMS_TO_TICKS(20)); ok = eebusy_wait(100); }
    }
    ee_end(had_eerd);
    return ok;
}

// ---- public API ----

void wallclock_init(void)
{
    i2c_master_bus_handle_t bus = power_i2c_bus();
    if (!bus) {
        ESP_LOGW(TAG, "no I2C bus — clock will be SNTP-only");
        return;
    }

    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = RV3028_I2C_ADDR,
        .scl_speed_hz    = PWR_I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(bus, &dc, &s_dev) != ESP_OK) {
        ESP_LOGW(TAG, "could not add RTC device handle");
        return;
    }

    // Not i2c_master_probe(): every address ACKs here. Require a sane status
    // byte and a valid BCD time instead.
    uint8_t status;
    struct tm t;
    if (!rd(REG_STATUS, &status, 1) || status == 0xFF || !read_time(&t)) {
        ESP_LOGW(TAG, "RV-3028 not responding sanely at 0x%02X — clock will be "
                      "SNTP-only", RV3028_I2C_ADDR);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return;
    }
    s_present = true;

    if (status & STATUS_PORF) {
        s_boot = WALLCLOCK_BOOT_LOST;
        ESP_LOGW(TAG, "RTC lost power (PORF) — held time is not trustworthy, "
                      "waiting for SNTP");
    } else {
        s_boot = WALLCLOCK_BOOT_HELD;
        struct timeval tv = { .tv_sec = tm_to_epoch_utc(&t), .tv_usec = 0 };
        settimeofday(&tv, NULL);
        s_src = WALLCLOCK_SRC_RTC;
        ESP_LOGI(TAG, "system clock seeded from RTC: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);
    }

    ensure_backup_switchover();
}

bool wallclock_store(void)
{
    s_src = WALLCLOCK_SRC_SNTP;
    if (!s_present) return false;

    time_t now = time(NULL);
    struct tm t;
    gmtime_r(&now, &t);
    if (!write_time(&t)) {
        ESP_LOGW(TAG, "failed to store time into RTC");
        return false;
    }
    ESP_LOGI(TAG, "RTC set from network: %04d-%02d-%02d %02d:%02d:%02d UTC",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    return true;
}

bool wallclock_rtc_present(void)     { return s_present; }
wallclock_src_t wallclock_source(void) { return s_src; }
wallclock_boot_t wallclock_boot_state(void) { return s_boot; }
uint8_t wallclock_backup_reg(void)   { return s_backup_reg; }

const char *wallclock_boot_str(void)
{
    switch (s_boot) {
    case WALLCLOCK_BOOT_HELD: return "held";
    case WALLCLOCK_BOOT_LOST: return "lost";
    default:                  return "absent";
    }
}

const char *wallclock_source_str(void)
{
    switch (s_src) {
    case WALLCLOCK_SRC_RTC:  return "rtc";
    case WALLCLOCK_SRC_SNTP: return "sntp";
    default:           return "unset";
    }
}
