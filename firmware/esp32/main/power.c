#include "power.h"
#include "lastoff.h"
#include "app_config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

static const char *TAG = "power";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_gauge;    // MAX17048
static i2c_master_dev_handle_t s_charger;  // BQ25180

static SemaphoreHandle_t s_lock;
static power_state_t     s_state;          // guarded by s_lock

// MAX17048 registers (checked against the datasheet, the Linux driver and
// Adafruit_MAX1704X). ATHD is 5 bits on the 17048: bit 5 is ALRT. The Linux
// driver's 0x3f mask is for the older 17040/41 and would clobber ALRT here.
#define MAX17048_REG_VCELL   0x02          // 78.125 uV / LSB
#define MAX17048_REG_SOC     0x04          // 1% / 256 LSB (high byte = integer %)
#define MAX17048_REG_CONFIG  0x0C          // RCOMP:15..8, SLEEP:7, ALSC:6, ALRT:5, ATHD:4..0
#define MAX17048_REG_VALRT   0x14          // high byte = MIN, low byte = MAX, 20 mV/LSB
#define MAX17048_REG_CRATE   0x16          // signed, 0.208 %/hr per LSB
#define MAX17048_REG_STATUS  0x1A          // alert flags in the high byte

#define MAX17048_CFG_ALSC    0x0040        // alert on every 1 % SOC change
#define MAX17048_CFG_ALRT    0x0020        // latched alert; write 0 to release the pin
#define MAX17048_CFG_ATHD    0x001F        // empty threshold, ATHD = 32 - percent

// STATUS high byte. Each is latched by the gauge and cleared by writing it back
// as zero; the ALRT pin itself is released by clearing CONFIG.ALRT, not these.
#define MAX17048_STATUS_RI   0x01          // reset indicator: model restarted
#define MAX17048_STATUS_VH   0x02          // voltage above VALRT.MAX
#define MAX17048_STATUS_VL   0x04          // voltage below VALRT.MIN
#define MAX17048_STATUS_VR   0x08          // voltage reset (cell swapped/collapsed)
#define MAX17048_STATUS_HD   0x10          // SOC below ATHD
#define MAX17048_STATUS_SC   0x20          // 1 % SOC change (only if ALSC set)
#define MAX17048_STATUS_ALL  0x3F

// BQ25180 registers (from the Zephyr driver, checked against libmcu/bq25180).
#define BQ25180_REG_STAT0    0x00
#define BQ25180_REG_STAT1    0x01
#define BQ25180_REG_FLAG0    0x02
#define BQ25180_REG_VBAT_CTRL 0x03
#define BQ25180_REG_ICHG_CTRL 0x04
#define BQ25180_REG_CHARGECTRL0 0x05
#define BQ25180_REG_CHARGECTRL1 0x06
#define BQ25180_REG_IC_CTRL   0x07
#define BQ25180_REG_TMR_ILIM  0x08
#define BQ25180_REG_TS_CONTROL 0x0B

#define STAT0_VIN_PGOOD      0x01          // bit 0
#define STAT0_THERMREG       0x02          // bit 1
#define STAT0_CHG_STAT_SHIFT 5             // bits 6:5
#define STAT0_CHG_STAT_MASK  0x03
#define STAT0_TS_OPEN        0x80          // bit 7

#define STAT1_TS_STAT_SHIFT  3             // bits 4:3
#define STAT1_TS_STAT_MASK   0x03

#define FLAG0_TS_FAULT       0x80          // bit 7
#define FLAG0_BUVLO_FAULT    0x02          // bit 1: battery fell below BUVLO
#define FLAG0_BAT_OCP_FAULT  0x01          // bit 0: discharge over-current tripped

#define ICHG_CHG_DIS         0x80          // bit 7
#define ICHG_CODE_MASK       0x7F
#define VBAT_CODE_MASK       0x7F

#define IC_CTRL_WD_MASK      0x03          // bits 1:0, watchdog select
#define IC_CTRL_WD_DISABLE   0x03
#define IC_CTRL_SAFETY_SHIFT 2             // bits 3:2, fast-charge timer
#define IC_CTRL_SAFETY_MASK  0x03

// ---- charger configuration fields (datasheet SLUSE99C tables 8-13..8-17) ----
// The input limit caps what the whole board draws from VIN, so the cell only
// gets what is left over; an ICHG above that headroom can't be delivered.
#define TMR_ILIM_ILIM_MASK   0x07          // TMR_ILIM bits 2:0

#define CC0_ITERM_SHIFT      4             // CHARGECTRL0 bits 5:4
#define CC0_ITERM_MASK       0x03
#define CC0_VINDPM_SHIFT     2             // bits 3:2
#define CC0_VINDPM_MASK      0x03
#define CC0_THERM_REG_MASK   0x03          // bits 1:0

#define CC1_IBAT_OCP_SHIFT   6             // CHARGECTRL1 bits 7:6
#define CC1_IBAT_OCP_MASK    0x03
#define CC1_BUVLO_SHIFT      3             // bits 5:3
#define CC1_BUVLO_MASK       0x07

// TS_CONTROL (0x0B): JEITA window edges. The charger is intentionally stricter
// than the cell datasheet (which allows charging to -20 C); don't "correct" it.
#define TS_HOT_SHIFT         6             // bits 7:6
#define TS_COLD_SHIFT        4             // bits 5:4
#define TS_2BIT_MASK         0x03
#define TS_WARM_DIS          0x08          // bit 3: 1 = warm threshold disabled
#define TS_COOL_DIS          0x04          // bit 2: 1 = cool threshold disabled
#define TS_ICHG_20PCT        0x02          // bit 1: 0 = 0.5x ICHG, 1 = 0.2x

// Read a 16-bit big-endian register (reg pointer write, then 2-byte read).
static bool rd16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *out)
{
    uint8_t b[2];
    if (i2c_master_transmit_receive(dev, &reg, 1, b, 2, 100) != ESP_OK)
        return false;
    *out = ((uint16_t)b[0] << 8) | b[1];
    return true;
}

static bool rd8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *out)
{
    if (i2c_master_transmit_receive(dev, &reg, 1, out, 1, 100) != ESP_OK)
        return false;
    return true;
}

// ICHG_CTRL code -> mA. Two slopes: 5..35 mA in 1 mA steps, then 40 mA up in
// 10 mA steps (Zephyr bq25180_ichg_to_ma).
static uint16_t ichg_code_to_ma(uint8_t code)
{
    code &= ICHG_CODE_MASK;
    return code <= 30 ? (uint16_t)(code + 5) : (uint16_t)((code - 31) * 10 + 40);
}

// ILIM code -> mA (a lookup, not a formula).
static uint16_t ilim_code_to_ma(uint8_t code)
{
    static const uint16_t tbl[8] = { 50, 100, 200, 300, 400, 500, 700, 1100 };
    return tbl[code & TMR_ILIM_ILIM_MASK];
}

// mA -> ILIM code, rounding down so we never grant more than asked.
static uint8_t ilim_ma_to_code(uint16_t ma)
{
    static const uint16_t tbl[8] = { 50, 100, 200, 300, 400, 500, 700, 1100 };
    uint8_t best = 0;
    for (uint8_t i = 0; i < 8; i++)
        if (tbl[i] <= ma) best = i;
    return best;
}

// mV -> VINDPM code. Anything but the three valid values means disabled.
static uint8_t vindpm_mv_to_code(uint16_t mv)
{
    switch (mv) {
    case 4200: return 0;
    case 4500: return 1;
    case 4700: return 2;
    default:   return 3;   // disabled
    }
}

// VINDPM code -> mV, 0 = disabled.
static uint16_t vindpm_code_to_mv(uint8_t code)
{
    switch (code & CC0_VINDPM_MASK) {
    case 0:  return 4200;
    case 1:  return 4500;
    case 2:  return 4700;
    default: return 0;      // 0b11 = disabled
    }
}

// ITERM code -> percent of ICHG, 0 = termination disabled.
static uint8_t iterm_code_to_pct(uint8_t code)
{
    static const uint8_t tbl[4] = { 0, 5, 10, 20 };
    return tbl[code & CC0_ITERM_MASK];
}

// BUVLO code -> mV. Codes 0..2 all mean 3.0 V per the datasheet.
static uint16_t buvlo_code_to_mv(uint8_t code)
{
    static const uint16_t tbl[8] = { 3000, 3000, 3000, 2800, 2600, 2400, 2200, 2000 };
    return tbl[code & CC1_BUVLO_MASK];
}

// Safety timer code -> hours, 0 = disabled.
static uint8_t safety_code_to_h(uint8_t code)
{
    static const uint8_t tbl[4] = { 3, 6, 12, 0 };
    return tbl[code & IC_CTRL_SAFETY_MASK];
}

// Battery discharge over-current limit -> mA, 0 = disabled.
static uint16_t ibat_ocp_code_to_ma(uint8_t code)
{
    static const uint16_t tbl[4] = { 500, 1000, 1500, 0 };
    return tbl[code & CC1_IBAT_OCP_MASK];
}

// TS cold/hot codes -> degrees C (different, non-monotonic tables).
static int8_t ts_cold_code_to_c(uint8_t code)
{
    static const int8_t tbl[4] = { 0, 3, 5, -3 };
    return tbl[code & TS_2BIT_MASK];
}

static int8_t ts_hot_code_to_c(uint8_t code)
{
    static const int8_t tbl[4] = { 60, 65, 50, 45 };
    return tbl[code & TS_2BIT_MASK];
}

// VBAT_CTRL code -> mV: 3500 mV + 10 mV per step.
static uint16_t vbat_code_to_mv(uint8_t code)
{
    return (uint16_t)((code & VBAT_CODE_MASK) * 10 + 3500);
}

const char *power_chg_state_str(chg_state_t s)
{
    switch (s) {
    case CHG_CONST_CURRENT: return "cc";
    case CHG_CONST_VOLTAGE: return "cv";
    case CHG_DONE:          return "done";
    default:                return "idle";
    }
}

const char *power_ts_zone_str(ts_zone_t z)
{
    switch (z) {
    case TS_SUSPENDED: return "suspended";
    case TS_DERATE_A:  return "derated";
    case TS_DERATE_B:  return "derated";
    default:           return "normal";
    }
}

static bool wr8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    return i2c_master_transmit(dev, tx, sizeof(tx), 100) == ESP_OK;
}

// Write a 16-bit big-endian register (the MAX17048's native width).
static bool wr16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t val)
{
    uint8_t tx[3] = { reg, (uint8_t)(val >> 8), (uint8_t)val };
    return i2c_master_transmit(dev, tx, sizeof(tx), 100) == ESP_OK;
}

// Inverse of ichg_code_to_ma().
static uint8_t ichg_ma_to_code(uint16_t ma)
{
    if (ma < 5)    ma = 5;
    if (ma > 1000) ma = 1000;
    return ma <= 35 ? (uint8_t)(ma - 5) : (uint8_t)((ma - 40) / 10 + 31);
}

// Program the charger once at boot, writing only registers that differ.
static void charger_configure(void)
{
    // Watchdog first: on expiry (160 s default) it resets every charger
    // register to default, so anything written before disabling it can revert
    // if the poll task ever stalls.
    uint8_t ic;
    if (rd8(s_charger, BQ25180_REG_IC_CTRL, &ic)) {
        uint8_t want = (uint8_t)((ic & ~IC_CTRL_WD_MASK) | IC_CTRL_WD_DISABLE);
        if (want == ic) {
            ESP_LOGI(TAG, "charger I2C watchdog already disabled (IC_CTRL 0x%02X)", ic);
        } else if (wr8(s_charger, BQ25180_REG_IC_CTRL, want)) {
            uint8_t back = 0;
            rd8(s_charger, BQ25180_REG_IC_CTRL, &back);
            ESP_LOGI(TAG, "charger I2C watchdog disabled: IC_CTRL 0x%02X -> 0x%02X (reads 0x%02X)",
                     ic, want, back);
        } else {
            ESP_LOGE(TAG, "could not disable charger watchdog — charge settings will revert");
        }
    }

    // VINDPM before ILIM: raising the limit without droop protection could
    // brown out a weak supply.
    uint8_t cc0r;
    if (rd8(s_charger, BQ25180_REG_CHARGECTRL0, &cc0r)) {
        uint8_t want = (uint8_t)((cc0r & ~(CC0_VINDPM_MASK << CC0_VINDPM_SHIFT)) |
                                 (vindpm_mv_to_code(BQ25180_VINDPM_MV) << CC0_VINDPM_SHIFT));
        if (want == cc0r) {
            ESP_LOGI(TAG, "VINDPM already %u mV", (unsigned)BQ25180_VINDPM_MV);
        } else if (wr8(s_charger, BQ25180_REG_CHARGECTRL0, want)) {
            uint8_t back = 0;
            rd8(s_charger, BQ25180_REG_CHARGECTRL0, &back);
            ESP_LOGW(TAG, "VINDPM %s -> %u mV (CHARGECTRL0 0x%02X -> 0x%02X, reads 0x%02X)",
                     vindpm_code_to_mv(cc0r >> CC0_VINDPM_SHIFT) ? "on" : "DISABLED",
                     (unsigned)vindpm_code_to_mv(back >> CC0_VINDPM_SHIFT),
                     cc0r, want, back);
        } else {
            ESP_LOGE(TAG, "VINDPM write failed — NOT raising the input limit");
            goto skip_ilim;
        }
    }

    // Bits 2:0 only; the rest is push-button and auto-wake timing.
    uint8_t tmr;
    if (rd8(s_charger, BQ25180_REG_TMR_ILIM, &tmr)) {
        uint8_t want = (uint8_t)((tmr & ~TMR_ILIM_ILIM_MASK) |
                                 ilim_ma_to_code(BQ25180_ILIM_TARGET_MA));
        if (want == tmr) {
            ESP_LOGI(TAG, "input limit already %u mA", ilim_code_to_ma(tmr));
        } else if (wr8(s_charger, BQ25180_REG_TMR_ILIM, want)) {
            uint8_t back = 0;
            rd8(s_charger, BQ25180_REG_TMR_ILIM, &back);
            ESP_LOGW(TAG, "input limit %u mA -> %u mA (TMR_ILIM 0x%02X -> 0x%02X, reads 0x%02X)",
                     ilim_code_to_ma(tmr), ilim_code_to_ma(back), tmr, want, back);
        } else {
            ESP_LOGE(TAG, "input limit write failed");
        }
    }
skip_ilim:

    // Safety timer: the 6 h default can stop a 4000 mAh charge part-way, so
    // use 12 h, the longest finite setting (kept finite as a backstop).
    uint8_t icr;
    if (rd8(s_charger, BQ25180_REG_IC_CTRL, &icr)) {
        uint8_t want = (uint8_t)((icr & ~(IC_CTRL_SAFETY_MASK << IC_CTRL_SAFETY_SHIFT)) |
                                 (0x02 << IC_CTRL_SAFETY_SHIFT));   // 0b10 = 12 h
        if (want == icr) {
            ESP_LOGI(TAG, "safety timer already %u h",
                     (unsigned)safety_code_to_h(icr >> IC_CTRL_SAFETY_SHIFT));
        } else if (wr8(s_charger, BQ25180_REG_IC_CTRL, want)) {
            uint8_t back = 0;
            rd8(s_charger, BQ25180_REG_IC_CTRL, &back);
            ESP_LOGW(TAG, "safety timer %u h -> %u h (IC_CTRL 0x%02X -> 0x%02X, reads 0x%02X)",
                     (unsigned)safety_code_to_h(icr >> IC_CTRL_SAFETY_SHIFT),
                     (unsigned)safety_code_to_h(back >> IC_CTRL_SAFETY_SHIFT),
                     icr, want, back);
        } else {
            ESP_LOGE(TAG, "could not set safety timer — charging will stop at 6 h");
        }
    }

    // Discharge over-current limit, bits 7:6 only (see app_config.h).
    uint8_t cc1r;
    if (rd8(s_charger, BQ25180_REG_CHARGECTRL1, &cc1r)) {
        uint8_t code = BQ25180_IBAT_OCP_MA == 0    ? 3 :
                       BQ25180_IBAT_OCP_MA >= 1500 ? 2 :
                       BQ25180_IBAT_OCP_MA >= 1000 ? 1 : 0;
        uint8_t want = (uint8_t)((cc1r & ~(CC1_IBAT_OCP_MASK << CC1_IBAT_OCP_SHIFT)) |
                                 (code << CC1_IBAT_OCP_SHIFT));
        if (want == cc1r) {
            ESP_LOGI(TAG, "discharge OCP already %u mA",
                     (unsigned)ibat_ocp_code_to_ma(cc1r >> CC1_IBAT_OCP_SHIFT));
        } else if (wr8(s_charger, BQ25180_REG_CHARGECTRL1, want)) {
            uint8_t back = 0;
            rd8(s_charger, BQ25180_REG_CHARGECTRL1, &back);
            ESP_LOGW(TAG, "discharge OCP %u mA -> %u mA (CHARGECTRL1 0x%02X -> 0x%02X, reads 0x%02X)",
                     (unsigned)ibat_ocp_code_to_ma(cc1r >> CC1_IBAT_OCP_SHIFT),
                     (unsigned)ibat_ocp_code_to_ma(back >> CC1_IBAT_OCP_SHIFT),
                     cc1r, want, back);
        } else {
            ESP_LOGE(TAG, "discharge OCP write failed — drive spin-up may trip it");
        }
    }

    // Fast-charge current, with CHG_DIS (bit 7) clear.
    uint8_t cur;
    if (rd8(s_charger, BQ25180_REG_ICHG_CTRL, &cur)) {
        uint8_t want = ichg_ma_to_code(BQ25180_ICHG_TARGET_MA);
        if (want == cur) {
            ESP_LOGI(TAG, "charge current already %u mA", ichg_code_to_ma(cur));
        } else if (wr8(s_charger, BQ25180_REG_ICHG_CTRL, want)) {
            uint8_t back = 0;
            rd8(s_charger, BQ25180_REG_ICHG_CTRL, &back);
            ESP_LOGW(TAG, "charge current %u mA -> %u mA (ICHG_CTRL 0x%02X -> 0x%02X, reads 0x%02X)",
                     ichg_code_to_ma(cur), ichg_code_to_ma(back), cur, want, back);
            ESP_LOGW(TAG, "  ^ PROVISIONAL rate (app_config.h BQ25180_ICHG_TARGET_MA) — linear "
                          "charger, unvalidated thermals; watch `thermreg` and re-test on the bench");
        } else {
            ESP_LOGE(TAG, "charge current write failed — still %u mA", ichg_code_to_ma(cur));
        }
    }

    // Read-only audit of the resulting configuration.
    uint8_t ilim, cc0, cc1, icc;
    if (rd8(s_charger, BQ25180_REG_TMR_ILIM, &ilim)) {
        uint16_t ilim_ma = ilim_code_to_ma(ilim);
        // The system load can't be measured here, so just print the headroom.
        int headroom = (int)ilim_ma - (int)BQ25180_ICHG_TARGET_MA;
        ESP_LOGI(TAG, "input limit %u mA (TMR_ILIM 0x%02X) — leaves %d mA for the "
                      "system above ICHG %u mA%s",
                 (unsigned)ilim_ma, ilim, headroom,
                 (unsigned)BQ25180_ICHG_TARGET_MA,
                 headroom <= 0 ? " <-- ICHG unreachable" : "");
    }
    if (rd8(s_charger, BQ25180_REG_CHARGECTRL0, &cc0)) {
        uint16_t vindpm = vindpm_code_to_mv(cc0 >> CC0_VINDPM_SHIFT);
        if (vindpm)
            ESP_LOGI(TAG, "ITERM %u%% of ICHG, VINDPM %u mV (CHARGECTRL0 0x%02X)",
                     (unsigned)iterm_code_to_pct(cc0 >> CC0_ITERM_SHIFT),
                     (unsigned)vindpm, cc0);
        else
            ESP_LOGI(TAG, "ITERM %u%% of ICHG, VINDPM DISABLED (CHARGECTRL0 0x%02X)",
                     (unsigned)iterm_code_to_pct(cc0 >> CC0_ITERM_SHIFT), cc0);
    }
    if (rd8(s_charger, BQ25180_REG_CHARGECTRL1, &cc1))
        ESP_LOGI(TAG, "BUVLO %u mV, discharge OCP %u mA (CHARGECTRL1 0x%02X)",
                 (unsigned)buvlo_code_to_mv(cc1 >> CC1_BUVLO_SHIFT),
                 (unsigned)ibat_ocp_code_to_ma(cc1 >> CC1_IBAT_OCP_SHIFT), cc1);
    uint8_t tsc;
    if (rd8(s_charger, BQ25180_REG_TS_CONTROL, &tsc))
        ESP_LOGI(TAG, "JEITA: cold %d C, cool %s, warm %s, hot %d C, derate to %u%% "
                      "(TS_CONTROL 0x%02X)",
                 ts_cold_code_to_c(tsc >> TS_COLD_SHIFT),
                 (tsc & TS_COOL_DIS) ? "off" : "10 C",
                 (tsc & TS_WARM_DIS) ? "off" : "45 C",
                 ts_hot_code_to_c(tsc >> TS_HOT_SHIFT),
                 (tsc & TS_ICHG_20PCT) ? 20u : 50u, tsc);

    if (rd8(s_charger, BQ25180_REG_IC_CTRL, &icc)) {
        uint8_t h = safety_code_to_h(icc >> IC_CTRL_SAFETY_SHIFT);
        if (h)
            ESP_LOGI(TAG, "safety timer %u h (IC_CTRL 0x%02X)", (unsigned)h, icc);
        else
            ESP_LOGI(TAG, "safety timer disabled (IC_CTRL 0x%02X)", icc);
    }
}

// ---- MAX17048 alert path ----
// The 2 s poll misses short events (a cell sagging under drive load, a charger
// that stops regulating); the gauge latches them in hardware. ALSC (alert on
// every 1 % SOC change) stays off: the poll already covers that.

// A persisting condition re-latches as soon as the pin is released, so rate-
// limit servicing it.
#define ALERT_COOLDOWN_US (30 * 1000 * 1000)

static SemaphoreHandle_t s_wake;           // given by the ISR or a request
static volatile bool     s_alert_pending;  // set in ISR; survives a missed take
static int64_t           s_alert_next_us;

// ---- runtime charge enable/disable (bench measurement aid) ----
// A linear charger's input current is I_SYS + I_BAT, so a USB meter reading
// with charging off and on gives the cell current. Applied by power_task,
// which owns the I2C bus.
static volatile int s_chg_req;          // 0 none, +1 enable, -1 disable

// A disable reverts on its own so the board can't be left never charging.
#define CHARGE_DISABLE_MAX_US (10LL * 60 * 1000 * 1000)
static int64_t s_chg_revert_us;         // 0 = not disabled by request

static void IRAM_ATTR alrt_isr(void *arg)
{
    (void)arg;
    s_alert_pending = true;
    if (s_wake) xSemaphoreGiveFromISR(s_wake, NULL);
}

// mV -> VALRT code: 20 mV/LSB, saturating at 8 bits.
static uint8_t valrt_code(unsigned mv)
{
    unsigned code = mv / 20;
    return (uint8_t)(code > 0xFF ? 0xFF : code);
}

// Program the thresholds and clear stale flags. Non-fatal: the poll still
// reports SOC if this fails.
static void gauge_configure(void)
{
    uint8_t lo = valrt_code(MAX17048_VALRT_MIN_MV);
    uint8_t hi = valrt_code(MAX17048_VALRT_MAX_MV);
    if (wr16(s_gauge, MAX17048_REG_VALRT, (uint16_t)(lo << 8) | hi))
        ESP_LOGI(TAG, "gauge VALRT %u..%u mV (codes 0x%02X/0x%02X)",
                 (unsigned)lo * 20, (unsigned)hi * 20, lo, hi);
    else
        ESP_LOGW(TAG, "gauge VALRT write failed");

    // ATHD = 32 - percent, clamped so it can't spill into ALRT.
    unsigned pct = MAX17048_ATHD_PCT;
    if (pct < 1)  pct = 1;
    if (pct > 32) pct = 32;

    uint16_t cfg;
    if (!rd16(s_gauge, MAX17048_REG_CONFIG, &cfg)) {
        ESP_LOGW(TAG, "gauge CONFIG unreadable — alerts not configured");
        return;
    }
    // Keep RCOMP and SLEEP; clear ALRT (may be latched from before boot) and
    // ALSC.
    uint16_t want = (uint16_t)((cfg & ~(MAX17048_CFG_ATHD | MAX17048_CFG_ALRT |
                                       MAX17048_CFG_ALSC)) | (32u - pct));
    if (!wr16(s_gauge, MAX17048_REG_CONFIG, want)) {
        ESP_LOGW(TAG, "gauge CONFIG write failed");
        return;
    }
    ESP_LOGI(TAG, "gauge empty alert at %u%% (CONFIG 0x%04X -> 0x%04X)",
             pct, cfg, want);

    // Clear the threshold flags (stale under the new thresholds) but keep RI,
    // which poll_once() reports so the UI can treat SOC as settling.
    uint16_t st;
    if (rd16(s_gauge, MAX17048_REG_STATUS, &st)) {
        uint8_t was = (uint8_t)(st >> 8) & MAX17048_STATUS_ALL;
        if (was)
            ESP_LOGI(TAG, "gauge STATUS at boot 0x%02X%s", (unsigned)was,
                     (was & MAX17048_STATUS_RI) ? " (RI kept)" : "");
        uint16_t keep = (uint16_t)(st & ~((uint16_t)(MAX17048_STATUS_ALL &
                                                    ~MAX17048_STATUS_RI) << 8));
        if (was & ~MAX17048_STATUS_RI)
            wr16(s_gauge, MAX17048_REG_STATUS, keep);
    }
}

// Arm MAX_ALRT only after the thresholds are programmed.
static void gauge_alerts_init(void)
{
    gauge_configure();

#ifdef MAX_ALRT_GPIO
    if (!s_wake) {
        ESP_LOGW(TAG, "no wake semaphore — ALRT polling only");
        return;
    }
    // power_io_init() already made this a pulled-up input.
    gpio_set_intr_type(MAX_ALRT_GPIO, GPIO_INTR_NEGEDGE);
    esp_err_t e = gpio_install_isr_service(0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO ISR service unavailable (%s) — polling only",
                 esp_err_to_name(e));
        return;
    }
    if (gpio_isr_handler_add(MAX_ALRT_GPIO, alrt_isr, NULL) != ESP_OK) {
        ESP_LOGW(TAG, "could not hook ALRT on GPIO%d — polling only", MAX_ALRT_GPIO);
        return;
    }
    gpio_intr_enable(MAX_ALRT_GPIO);

    // Already low from before boot means no edge will come.
    if (gpio_get_level(MAX_ALRT_GPIO) == 0) {
        ESP_LOGW(TAG, "ALRT already asserted at boot");
        s_alert_pending = true;
    }
    ESP_LOGI(TAG, "gauge ALRT armed on GPIO%d (falling edge)", MAX_ALRT_GPIO);
#endif
}

// Record why ALRT asserted and release it. power_task only (I2C).
static void service_alert(void)
{
    uint16_t st;
    if (!rd16(s_gauge, MAX17048_REG_STATUS, &st)) {
        ESP_LOGW(TAG, "ALRT asserted but STATUS unreadable");
        return;
    }
    uint8_t flags = (uint8_t)(st >> 8) & MAX17048_STATUS_ALL;
    if (!flags) {
        // Cause already consumed; only CONFIG.ALRT is holding the pin.
        ESP_LOGD(TAG, "ALRT with no STATUS flags — releasing pin");
    } else {
        ESP_LOGW(TAG, "gauge ALERT 0x%02X:%s%s%s%s%s%s", flags,
                 (flags & MAX17048_STATUS_VL) ? " low-voltage"  : "",
                 (flags & MAX17048_STATUS_HD) ? " low-soc"      : "",
                 (flags & MAX17048_STATUS_VH) ? " OVER-VOLTAGE" : "",
                 (flags & MAX17048_STATUS_VR) ? " voltage-reset": "",
                 (flags & MAX17048_STATUS_RI) ? " gauge-reset"  : "",
                 (flags & MAX17048_STATUS_SC) ? " soc-change"   : "");
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.gauge_alerts |= flags;      // sticky: see power.h
    s_state.alert_count++;
    xSemaphoreGive(s_lock);

    // STATUS first, then CONFIG.ALRT, or a still-set flag re-asserts the pin.
    // RI is cleared here because it has just been recorded.
    if (flags && !wr16(s_gauge, MAX17048_REG_STATUS, (uint16_t)(st & ~((uint16_t)flags << 8))))
        ESP_LOGW(TAG, "failed to clear gauge STATUS flags");

    uint16_t cfg;
    if (rd16(s_gauge, MAX17048_REG_CONFIG, &cfg) &&
        !wr16(s_gauge, MAX17048_REG_CONFIG, (uint16_t)(cfg & ~MAX17048_CFG_ALRT)))
        ESP_LOGW(TAG, "failed to release gauge ALRT");
}

static void poll_once(void)
{
    power_state_t ns;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    ns = s_state;                                   // keep old values on read failure
    xSemaphoreGive(s_lock);

    if (s_gauge) {
        uint16_t v;
        if (rd16(s_gauge, MAX17048_REG_VCELL, &v))
            ns.mv = (uint16_t)(((uint32_t)v * 5) / 64);   // 78.125 uV = 5/64 mV
        uint16_t s;
        if (rd16(s_gauge, MAX17048_REG_SOC, &s)) {
            int soc = s >> 8;                             // integer percent
            ns.soc = (int16_t)(soc > 100 ? 100 : soc);
        }
        uint16_t c;
        if (rd16(s_gauge, MAX17048_REG_CRATE, &c)) {
            int32_t cp = ((int32_t)(int16_t)c * 208) / 10;   // -> centi-%/hr
            if (cp >  32767) cp =  32767;
            if (cp < -32768) cp = -32768;
            ns.crate_cpph = (int16_t)cp;
            ns.crate_valid = true;
        }
        uint16_t st;
        if (rd16(s_gauge, MAX17048_REG_STATUS, &st)) {
            ns.gauge_status = (uint8_t)(st >> 8);
            ns.gauge_reset  = (ns.gauge_status & MAX17048_STATUS_RI) != 0;
        }
    }

    if (s_charger) {
        uint8_t st0;
        if (rd8(s_charger, BQ25180_REG_STAT0, &st0)) {
            ns.chg_stat_raw = st0;
            ns.usb_present  = (st0 & STAT0_VIN_PGOOD) != 0;
            ns.thermreg     = (st0 & STAT0_THERMREG) != 0;
            ns.ts_open      = (st0 & STAT0_TS_OPEN) != 0;
            ns.chg_state    = (chg_state_t)((st0 >> STAT0_CHG_STAT_SHIFT) & STAT0_CHG_STAT_MASK);
            // CC or CV only; DONE is a full battery, not charging.
            ns.charging     = (ns.chg_state == CHG_CONST_CURRENT ||
                               ns.chg_state == CHG_CONST_VOLTAGE);
        }
        uint8_t st1;
        if (rd8(s_charger, BQ25180_REG_STAT1, &st1)) {
            ns.stat1_raw = st1;
            ns.ts_zone   = (ts_zone_t)((st1 >> STAT1_TS_STAT_SHIFT) & STAT1_TS_STAT_MASK);
        }
        // FLAG0 is read-to-clear, so accumulate it. The first read is kept
        // separately: the charger stays powered from the cell, so it still
        // holds whatever fault cut power before this boot.
        uint8_t fl0;
        if (rd8(s_charger, BQ25180_REG_FLAG0, &fl0)) {
            if (!ns.boot_flag0_valid) {
                ns.boot_flag0 = fl0;
                ns.boot_flag0_valid = true;
                if (fl0 & (FLAG0_BAT_OCP_FAULT | FLAG0_BUVLO_FAULT))
                    ESP_LOGE(TAG, "charger faults latched before this boot: FLAG0 0x%02X%s%s",
                             fl0, (fl0 & FLAG0_BAT_OCP_FAULT) ? " BAT_OCP (discharge over-current cut the battery)" : "",
                             (fl0 & FLAG0_BUVLO_FAULT) ? " BUVLO (battery under-voltage)" : "");
                else
                    ESP_LOGI(TAG, "charger FLAG0 at boot 0x%02X", fl0);
            }
            ns.flag0_raw |= fl0;
            if (fl0 & FLAG0_TS_FAULT) ns.ts_fault = true;
        }
        uint8_t ic, vb;
        if (rd8(s_charger, BQ25180_REG_ICHG_CTRL, &ic)) {
            ns.ichg_ma      = ichg_code_to_ma(ic);
            ns.chg_disabled = (ic & ICHG_CHG_DIS) != 0;
        }
        if (rd8(s_charger, BQ25180_REG_VBAT_CTRL, &vb))
            ns.vbatreg_mv = vbat_code_to_mv(vb);

        // Re-read every poll: a watchdog expiry or charger reset would restore
        // the defaults.
        uint8_t cc0, cc1, icc, tmr;
        if (rd8(s_charger, BQ25180_REG_CHARGECTRL0, &cc0)) {
            ns.cc0_raw     = cc0;
            ns.iterm_pct   = iterm_code_to_pct(cc0 >> CC0_ITERM_SHIFT);
            ns.vindpm_mv   = vindpm_code_to_mv(cc0 >> CC0_VINDPM_SHIFT);
            // Only 0b00 (100 C) and 0b11 (disabled) are documented.
            uint8_t tr     = cc0 & CC0_THERM_REG_MASK;
            ns.treg_c      = (tr == 0) ? 100 : 0;
            ns.treg_known  = (tr == 0 || tr == 3);
        }
        if (rd8(s_charger, BQ25180_REG_CHARGECTRL1, &cc1)) {
            ns.cc1_raw     = cc1;
            ns.buvlo_mv    = buvlo_code_to_mv(cc1 >> CC1_BUVLO_SHIFT);
            ns.ibat_ocp_ma = ibat_ocp_code_to_ma(cc1 >> CC1_IBAT_OCP_SHIFT);
        }
        if (rd8(s_charger, BQ25180_REG_IC_CTRL, &icc)) {
            ns.ic_ctrl_raw    = icc;
            ns.safety_timer_h = safety_code_to_h(icc >> IC_CTRL_SAFETY_SHIFT);
        }
        if (rd8(s_charger, BQ25180_REG_TMR_ILIM, &tmr)) {
            ns.tmr_ilim_raw = tmr;
            ns.ilim_ma      = ilim_code_to_ma(tmr);
        }
        uint8_t ts;
        if (rd8(s_charger, BQ25180_REG_TS_CONTROL, &ts)) {
            ns.ts_ctrl_raw = ts;
            ns.ts_cold_c   = ts_cold_code_to_c(ts >> TS_COLD_SHIFT);
            ns.ts_hot_c    = ts_hot_code_to_c(ts >> TS_HOT_SHIFT);
            ns.ts_cool_c   = (ts & TS_COOL_DIS) ? INT8_MIN : 10;   // MIN = disabled
            ns.ts_warm_c   = (ts & TS_WARM_DIS) ? INT8_MIN : 45;
            ns.ts_derate_pct = (ts & TS_ICHG_20PCT) ? 20 : 50;
        }
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = ns;
    xSemaphoreGive(s_lock);
}

// power_task only.
static void apply_charge_enable(bool en)
{
    if (!s_charger) return;
    uint8_t cur;
    if (!rd8(s_charger, BQ25180_REG_ICHG_CTRL, &cur)) {
        ESP_LOGW(TAG, "charge enable: ICHG_CTRL unreadable");
        return;
    }
    uint8_t want = en ? (uint8_t)(cur & ~ICHG_CHG_DIS) : (uint8_t)(cur | ICHG_CHG_DIS);
    if (want != cur && !wr8(s_charger, BQ25180_REG_ICHG_CTRL, want)) {
        ESP_LOGW(TAG, "charge enable: ICHG_CTRL write failed");
        return;
    }
    s_chg_revert_us = en ? 0 : esp_timer_get_time() + CHARGE_DISABLE_MAX_US;
    ESP_LOGW(TAG, "charging %s (ICHG_CTRL 0x%02X -> 0x%02X)%s",
             en ? "ENABLED" : "DISABLED", cur, want,
             en ? "" : " — auto-reverts in 10 min");
}

void power_request_charge_enable(bool en)
{
    s_chg_req = en ? 1 : -1;
    if (s_wake) xSemaphoreGive(s_wake);
}

static void power_task(void *arg)
{
    (void)arg;
    for (;;) {
        // Before poll_once() so the snapshot reflects the change.
        int req = s_chg_req;
        if (req) {
            s_chg_req = 0;
            apply_charge_enable(req > 0);
        } else if (s_chg_revert_us && esp_timer_get_time() >= s_chg_revert_us) {
            ESP_LOGW(TAG, "charge-disable window expired — re-enabling charging");
            apply_charge_enable(true);
        }

        poll_once();
        // 2 s poll, cut short by an alert or a request.
        if (s_wake)
            xSemaphoreTake(s_wake, pdMS_TO_TICKS(2000));
        else
            vTaskDelay(pdMS_TO_TICKS(2000));

        // Check the pin level too, so a lost edge can't strand an alert.
        bool asserted = s_alert_pending;
#ifdef MAX_ALRT_GPIO
        if (gpio_get_level(MAX_ALRT_GPIO) == 0) asserted = true;
#endif
        if (asserted && s_gauge && esp_timer_get_time() >= s_alert_next_us) {
            s_alert_pending = false;
            service_alert();
            s_alert_next_us = esp_timer_get_time() + ALERT_COOLDOWN_US;
            poll_once();
        }
    }
}

static bool probe_add(uint16_t addr, i2c_master_dev_handle_t *dev)
{
    if (i2c_master_probe(s_bus, addr, 50) != ESP_OK)
        return false;
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = PWR_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &dc, dev) == ESP_OK;
}

void power_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_wake = xSemaphoreCreateBinary();
    s_state.soc = -1;
    s_state.crate_cpph = 0;

    i2c_master_bus_config_t bc = {
        .i2c_port                     = PWR_I2C_PORT,
        .sda_io_num                   = PWR_I2C_SDA_GPIO,
        .scl_io_num                   = PWR_I2C_SCL_GPIO,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,   // external pull-ups still recommended
    };
    if (i2c_new_master_bus(&bc, &s_bus) != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus init failed — power-path telemetry disabled");
        return;
    }

    bool g = probe_add(MAX17048_I2C_ADDR, &s_gauge);
    bool c = probe_add(BQ25180_I2C_ADDR, &s_charger);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.gauge_ok   = g;
    s_state.charger_ok = c;
    s_state.present    = g || c;
    xSemaphoreGive(s_lock);

    if (!(g || c)) {
        // No battery hardware. Keep the bus: the RV-3028 RTC shares it.
        ESP_LOGI(TAG, "no charger/gauge on I2C (SDA=%d SCL=%d) — not a battery build",
                 PWR_I2C_SDA_GPIO, PWR_I2C_SCL_GPIO);
        return;
    }

    if (g) gauge_alerts_init();
    ESP_LOGI(TAG, "power path: gauge(MAX17048)=%s charger(BQ25180)=%s",
             g ? "yes" : "no", c ? "yes" : "no");
    if (c)
        charger_configure();
    poll_once();
    xTaskCreatePinnedToCore(power_task, "power", 3072, NULL, 2, NULL, 0);
}

bool power_get(power_state_t *out)
{
    if (!s_lock) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool present = s_state.present;
    if (present && out) *out = s_state;
    xSemaphoreGive(s_lock);
    return present;
}

i2c_master_bus_handle_t power_i2c_bus(void) { return s_bus; }

bool power_present(void)
{
    power_state_t p;
    return power_get(&p);
}

// ---- power-path control GPIOs (independent of the I2C probe above) ----

void power_io_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << BOOST_EN_GPIO) | (1ULL << LTC_KILL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);
    gpio_set_level(BOOST_EN_GPIO, 0);   // 5 V drive rail off until a read
    gpio_set_level(LTC_KILL_GPIO, 0);   // KILL deasserted (Q2 off) — stay powered

    uint64_t in_mask = 1ULL << LTC_INT_GPIO;
#ifdef BQ_INT_GPIO
    in_mask |= 1ULL << BQ_INT_GPIO;
#endif
#ifdef MAX_ALRT_GPIO
    in_mask |= 1ULL << MAX_ALRT_GPIO;
#endif
    gpio_config_t in = {
        .pin_bit_mask = in_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // active-low lines; unconnected reads high
    };
    gpio_config(&in);
}

// The rail is on while any reason holds: a drive session, a USB host on the
// controller, or a manual hold.
static portMUX_TYPE s_boost_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_boost_session, s_boost_usb;
static int64_t s_boost_hold_until;       // esp_timer_get_time() deadline, 0 = none
static esp_timer_handle_t s_boost_timer;

static void boost_apply_locked(void)
{
    bool hold = s_boost_hold_until > esp_timer_get_time();
    gpio_set_level(BOOST_EN_GPIO, (s_boost_session || s_boost_usb || hold) ? 1 : 0);
}

static void boost_hold_expired(void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&s_boost_mux);
    s_boost_hold_until = 0;
    boost_apply_locked();
    portEXIT_CRITICAL(&s_boost_mux);
    ESP_LOGI(TAG, "5 V drive-rail hold expired");
}

void power_boost_5v(bool on)
{
    portENTER_CRITICAL(&s_boost_mux);
    s_boost_session = on;
    boost_apply_locked();
    portEXIT_CRITICAL(&s_boost_mux);
}

void power_boost_usb(bool on)
{
    portENTER_CRITICAL(&s_boost_mux);
    s_boost_usb = on;
    boost_apply_locked();
    portEXIT_CRITICAL(&s_boost_mux);
}

void power_boost_hold(uint32_t secs)
{
    if (!s_boost_timer) {
        const esp_timer_create_args_t a = { .callback = boost_hold_expired, .name = "boost_hold" };
        if (esp_timer_create(&a, &s_boost_timer) != ESP_OK) return;
    }
    esp_timer_stop(s_boost_timer);
    portENTER_CRITICAL(&s_boost_mux);
    s_boost_hold_until = secs ? esp_timer_get_time() + (int64_t)secs * 1000000 : 0;
    boost_apply_locked();
    portEXIT_CRITICAL(&s_boost_mux);
    if (secs) esp_timer_start_once(s_boost_timer, (uint64_t)secs * 1000000);
    ESP_LOGW(TAG, "5 V drive-rail hold %s (%u s)", secs ? "on" : "off", (unsigned)secs);
}

uint32_t power_boost_hold_left(void)
{
    int64_t left = s_boost_hold_until - esp_timer_get_time();
    return left > 0 ? (uint32_t)((left + 999999) / 1000000) : 0;
}

// Tracked state: gpio_get_level() reads 0 on an output-only pin.
bool power_boost_is_on(void) { return s_boost_session || s_boost_usb || power_boost_hold_left() > 0; }

void power_kill(void)
{
    ESP_LOGW(TAG, "asserting LTC2954 KILL — powering off");
    lastoff_mark_kill();   // so the next boot can tell this from a power loss
    gpio_set_level(LTC_KILL_GPIO, 1);
}

int power_low_level(void)
{
    power_state_t ps;
    if (!power_get(&ps) || !ps.gauge_ok || ps.usb_present || ps.soc < 0) return 0;
    if (ps.soc <= BATT_CRIT_PCT) return 2;
    if (ps.soc <= BATT_LOW_PCT) return 1;
    return 0;
}
