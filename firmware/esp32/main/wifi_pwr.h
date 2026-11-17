#pragma once
// WiFi power policy during drive sessions (brown-out A/B testing).
//
// The board has lost power mid-read with the web UI busy: drive motor + 5 V
// boost + WiFi TX bursts on a high-impedance battery path. This lets the
// radio's share of the peak be cut without reflashing. Saved in NVS
// ("wifipwr"), set with POST /api/wifipwr?mode=N[&dbm=D], shown in
// /api/status under "wifipwr".
//
//   0 NORMAL  - full TX power all the time (the old behaviour)
//   1 LOWTX   - TX power capped to `dbm` while the drive rail is on
//   2 OFF     - radio stopped for the whole drive session, reconnects after
//   3 SPINUP  - radio stopped only for motor spin-up, back on once it settles
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

enum { WIFIPWR_NORMAL = 0, WIFIPWR_LOWTX = 1, WIFIPWR_OFF = 2, WIFIPWR_SPINUP = 3, WIFIPWR_N };

// Load the saved policy. Call after nvs_flash_init().
void wifi_pwr_init(void);
// Call once the STA radio has been started (wifi_start); before that, and in
// the provisioning portal, every hook below is a no-op.
void wifi_pwr_sta_ready(void);

int      wifi_pwr_mode(void);
int      wifi_pwr_low_dbm(void);
bool     wifi_pwr_set(int mode, int low_dbm);    // persists; false = bad args

// Drive-session hooks (gw_drive_prepare / gw_drive_release).
void wifi_pwr_drive_begin(void);    // before the boost + motor come on
void wifi_pwr_spinup_done(void);    // after the spin-up settle
void wifi_pwr_drive_end(void);      // after the motor and boost are off

// True while this module has the radio stopped (the event handler must not
// try to reconnect then).
bool     wifi_pwr_radio_paused(void);
// Live TX power in dBm (from the driver), or -1 if unknown/radio off.
int      wifi_pwr_tx_dbm(void);
// RSSI of the AP, or 0 if not associated.
int      wifi_pwr_rssi(void);

// Associated AP: SSID, RSSI and channel. False if not associated.
bool     wifi_pwr_ap(char *ssid, size_t n, int *rssi, int *channel);

void wifi_pwr_json(char *buf, size_t len);
