// Battery power path: BQ25180 charger + MAX17048 gauge over I2C, plus the
// boost/kill GPIOs (docs/power-topology.md). If neither IC answers at boot the
// module reports "absent" and everything else still works.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

// BQ25180 STAT0 CHG_STAT field (bits 6:5).
typedef enum {
    CHG_NOT_CHARGING = 0,
    CHG_CONST_CURRENT = 1,
    CHG_CONST_VOLTAGE = 2,
    CHG_DONE = 3,
} chg_state_t;

// BQ25180 STAT1 TS_STAT field (bits 4:3): a JEITA zone, not a temperature.
typedef enum {
    TS_NORMAL = 0,     // within the normal charging window
    TS_SUSPENDED = 1,  // outside T1..T5 — charging suspended (too cold or too hot)
    TS_DERATE_A = 2,   // in a derated zone (reduced current or voltage)
    TS_DERATE_B = 3,   // the other derated zone
} ts_zone_t;

typedef struct {
    bool     present;       // at least one power-path IC answered at boot
    bool     gauge_ok;      // MAX17048 present
    bool     charger_ok;    // BQ25180 present
    int16_t  soc;           // battery charge 0..100 %, -1 = unknown
    uint16_t mv;            // battery millivolts, 0 = unknown
    bool     usb_present;   // VIN power-good (STAT0 bit 0)
    bool     charging;      // CHG_STAT says current is flowing into the cell
    uint8_t  chg_stat_raw;  // raw BQ25180 STAT0 byte

    // ---- gauge extras ----
    int16_t  crate_cpph;    // charge rate, hundredths of a percent per hour;
                            // negative = discharging
    bool     crate_valid;
    uint8_t  gauge_status;  // live STATUS high byte; normally 0 because
                            // service_alert() clears it
    bool     gauge_reset;   // RI: the gauge power-cycled and its model restarted

    // Every MAX17048 alert since boot (STATUS bit layout), kept because the
    // interesting ones are transient. Bit 1 = over-voltage (charger fault),
    // bit 2 = low voltage, bit 4 = low SOC.
    uint8_t  gauge_alerts;
    uint32_t alert_count;   // how many times ALRT has been serviced

    // ---- charger extras ----
    uint8_t  stat1_raw;
    uint8_t  flag0_raw;     // sticky: FLAG0 is read-to-clear, so bits accumulate
    uint8_t  boot_flag0;    // FLAG0 as first read this boot: faults from before
    bool     boot_flag0_valid; // it, e.g. bit 0 = BATOCP cut the power
    chg_state_t chg_state;
    ts_zone_t   ts_zone;
    bool     ts_open;       // thermistor not connected (STAT0 bit 7)
    bool     ts_fault;      // STICKY: a TS fault latched at least once
    bool     thermreg;      // charger is in thermal regulation (throttling itself)
    uint16_t ichg_ma;       // configured fast-charge current, mA (0 = unread)
    uint16_t vbatreg_mv;    // configured charge voltage, mV (0 = unread)
    bool     chg_disabled;  // ICHG_CTRL CHG_DIS bit

    // ---- charger configuration, re-read every poll ----
    // ilim_ma caps the whole board's draw from VIN, so the cell only gets what
    // is left; ichg_ma above that headroom can't be delivered.
    uint16_t ilim_ma;        // input current limit, mA (0 = unread)
    uint16_t vindpm_mv;      // input droop threshold, mV; 0 = DISABLED
    uint8_t  iterm_pct;      // termination current as % of ichg_ma; 0 = disabled
    uint16_t buvlo_mv;       // battery undervoltage lockout, mV
    uint16_t ibat_ocp_ma;    // discharge over-current limit, mA; 0 = disabled
    uint8_t  safety_timer_h; // fast-charge safety timer, hours; 0 = disabled
    uint8_t  treg_c;         // thermal regulation threshold, C; 0 = off/unknown
    bool     treg_known;     // false when the code is one the datasheet omits

    // JEITA window from TS_CONTROL. INT8_MIN = threshold disabled.
    int8_t   ts_cold_c;      // suspend below this
    int8_t   ts_cool_c;      // derate below this (INT8_MIN = disabled)
    int8_t   ts_warm_c;      // reduce target voltage above this (INT8_MIN = disabled)
    int8_t   ts_hot_c;       // suspend above this
    uint8_t  ts_derate_pct;  // charge current in the cool zone, % of ichg_ma
    uint8_t  ts_ctrl_raw;

    uint8_t  cc0_raw;        // raw CHARGECTRL0, CHARGECTRL1, IC_CTRL, TMR_ILIM
    uint8_t  cc1_raw;
    uint8_t  ic_ctrl_raw;
    uint8_t  tmr_ilim_raw;
} power_state_t;

// Human-readable forms for /api/status and the LCD.
const char *power_chg_state_str(chg_state_t s);
const char *power_ts_zone_str(ts_zone_t z);

// Bring up the I2C bus, probe the charger and gauge, and start the poll task
// if either answered. Safe on any board.
void power_init(void);

// Latest snapshot. False (and *out untouched) if no power-path IC is present.
bool power_get(power_state_t *out);
bool power_present(void);

// The I2C bus created by power_init(), shared with the RV-3028 RTC
// (wallclock.c). NULL if creation failed.
i2c_master_bus_handle_t power_i2c_bus(void);

// BOOST_EN and LTC_KILL low, interrupt lines as pulled-up inputs. Call once at
// boot, before ui_init() and the gw task.
void power_io_init(void);

// 5 V drive rail (TPS61022). On while any of the three reasons below holds.
void power_boost_5v(bool on);  // drive session (gw_drive_prepare/release)
void power_boost_usb(bool on); // USB host on the controller (gw_usb_watch)
// Manual hold for `secs` (0 = cancel): bench metering, or USB use with a
// controller too old to report it. POST /api/drive5v.
void     power_boost_hold(uint32_t secs);
uint32_t power_boost_hold_left(void);   // seconds, 0 = no hold
bool     power_boost_is_on(void);

// Battery warning level for the LCD and /api/status: 0 = fine, 1 = low
// (<= BATT_LOW_PCT), 2 = critical (<= BATT_CRIT_PCT). Always 0 on USB power
// or when the gauge is absent.
int power_low_level(void);

// Latch system power off via LTC2954 KILL. Caller checks power_present().
void power_kill(void);

// Bench aid: enable/disable charging (CHG_DIS). Safe from any task; applied by
// the power task. A disable reverts after 10 minutes or on reboot. See
// docs/power-topology.md.
void power_request_charge_enable(bool en);
