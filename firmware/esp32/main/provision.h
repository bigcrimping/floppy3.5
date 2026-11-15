// WiFi provisioning via a SoftAP captive portal. When no STA credentials are
// stored, the device brings up an open access point + a captive-portal DNS
// responder + a one-page config form; submitting it writes the credentials to
// NVS (namespace "wifi") and reboots into station mode. See
// firmware/esp32/README.md ("WiFi setup").
#pragma once
#include <stdbool.h>

// True if a non-empty STA SSID is stored in NVS.
bool provision_have_creds(void);

// Erase the stored STA credentials (forces the portal on the next boot). Used
// for wrong-password recovery.
void provision_clear_creds(void);

// Bring up the SoftAP + captive portal and return; the HTTP + DNS servers run in
// the background. A valid form submission saves creds to NVS and reboots.
void provision_run_portal(void);
