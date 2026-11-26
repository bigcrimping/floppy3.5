// Hardware pin map and build configuration for the gw-esp32 PCB.
#pragma once

// ---- GWSPI link to the RP2354A controller (docs/spi-link-protocol.md) ----
// SPI3 master, mode 1: the PL022 slave needs CS held low across back-to-back
// bytes. The PL022 slave tops out at SSPCLK/12 = 12.5 MHz; 6 MHz is the
// qualified fallback.
#define GW_LINK_SPI_HOST    SPI3_HOST
#define GW_SPI_SCLK_GPIO    7       // -> RP2354 GPIO2 (SPI0 SCK)   [IC1-7 GW_SCLK]
#define GW_SPI_MOSI_GPIO    16      // -> RP2354 GPIO0 (SPI0 RX)    [IC1-9 GW_MOSI]
#define GW_SPI_MISO_GPIO    6       // <- RP2354 GPIO3 (SPI0 TX)    [IC1-6 GW_MISO]
#define GW_SPI_CS_GPIO      15      // -> RP2354 GPIO1 (SPI0 CSn)   [IC1-8 GW_CS]
#define GW_SPI_DRDY_GPIO    5       // <- RP2354 GPIO4: "slave has output" wake hint
#define GW_SPI_CLK_HZ       (12 * 1000 * 1000)

// Minimum gap between CS-up and the next CS-down: the slave validates its
// frame and re-arms DMA in it (protocol §1).
#define GW_SPI_GAP_US       100

// Frame geometry lives in firmware/common/gwspi_wire.h, shared with the RP.

// SYNC bring-up budget: besides a verb's own deadline, the only thing that
// declares the link hard-down (protocol §2.3).
#define GW_SYNC_TIMEOUT_MS  3000

// ---- Peer reset line (open-drain to RP RUN) ----
// Pulsed at init and by gw_relink so both ends restart in sync; also used by
// the SWD flasher.
#define GW_RESET_GPIO       41      // -> RP2354 RUN, active low [IC1-34 RP_RUN_N]
#define GW_RESET_LOW_MS     15
#define GW_BOOT_SETTLE_MS   300     // QSPI boot is 100-300 ms; SYNC polls after this

// ---- Controller flasher: SWD (rp_swd.c + rp_flash.c, /api/rpflash) ----
// SWD works on a blank or wedged chip and needs no BOOTSEL strap. Both nets
// reach the J12 debug header via 100R R59/R60 and are tristated except during
// a flash, so a probe on J12 still works; SWCLK has a 100k pull-down at the
// RP. See firmware/rp2354/docs/bringup.md.
#define RP_SWCLK_GPIO       21      // -> RP2354 SWCLK (also J12 pin 1) [IC1-23 RP_SWCLK]
#define RP_SWDIO_GPIO       14      // <-> RP2354 SWDIO (also J12 pin 3) [IC1-22 RP_SWDIO]
#define RP_FLASH_IMG_MAX    (512 * 1024) // firmware is ~100 KB; PSRAM-buffered

// ---- Drive / disk geometry: IBM 1.44 MB HD floppy ----
#define DISK_CYLS           80
#define DISK_HEADS          2
#define DISK_SECTORS        18      // per track
#define SECTOR_SIZE         512
#define DISK_IMAGE_BYTES    (DISK_CYLS * DISK_HEADS * DISK_SECTORS * SECTOR_SIZE) // 1.44 MB
#define TRACK_RETRIES       3       // attempts per track on any sector CRC fail

// Capture window per track. One revolution is 200 ms; the extra ~11 ms catches
// a sector straddling the random start point. Below ~215 ms sectors go missing.
// 230 (was 220) gives ~30 ms of overlap, so more sectors near the start point
// are captured twice and the decoder can use the better copy: a marginal
// sector on the test disk (c58h1) failed its first read ~1 in 4 at 220 once
// reads started streaming immediately (speed-up phase 3). Costs ~1.6 s a disk.
#define GW_READ_WINDOW_MS   230

// Disk test / health check grading (app_main.c), also served in /api/trackmap
// for the web page's margin view. Not calibrated yet: MARGIN_LOW is reported,
// but doesn't change a verdict.
#define TEST_WEAK_MARGINAL  4       // this many weak tracks (or more) = MARGINAL
#define MARGIN_LOW          25      // timing margin under this = a low-margin track

// Read cyl 0, parse the FAT, then read only the tracks in use. Falls back to a
// whole-disk read if the disk isn't FAT12. Set 0 for full-surface archival.
#define READ_FAT_AWARE      1

// Per-dump phase timing, logged and shown as the status detail. 0 = off.
#define GW_PROFILE          1

// ---- Drive control ----
// The controller drives a single SELECT line for units 0 and 1, so the unit
// number only matters to the protocol. Override with -DGW_DRIVE_UNIT=n.
#ifndef GW_DRIVE_UNIT
#define GW_DRIVE_UNIT       1
#endif
#define GW_BUS_SHUGART      2       // Sony MPF920 is Shugart bus; IBMPC=1

// ---- UI: S1/S2/S3 buttons (active-low, 10k pull-ups), WS2812 chain ----
// Functions follow the V3 housing legends, front -> back DISP / READ / SAVE /
// POWER, i.e. S3 / S2 / S1 / S5. The net names are older (S2 was SAVE, S1 BOOT).
// GPIO0 is the BOOT strap; it reads fine as a button after boot.
#define UI_BUTTON_GPIO      42      // READ -> full disk read           [IC1-35 SAVE / S2]
#define UI_BTN_SAVEALL_GPIO 0       // SAVE -> copy every file to SD    [IC1-27 BOOT_BTN / S1]
#define UI_BTN_DISPLAY_GPIO 4       // DISP: short = home screen / disk screen, long = LCD on/off [IC1-4 PWER_DISP / S3]
#define UI_LED_GPIO         12      // WS2812 data     -> R1 -> LED1..LED4 chain [IC1-20 RGB_LED]
#define UI_LED_IS_RGB       1       // chain of four WS2812B-B

// ---- microSD (SPI2, shared with the LCD) ----
#define SD_SPI_HOST         SPI2_HOST
#define SD_PIN_CLK          11      // shared SPI2 SCLK [IC1-19 SPI2_SCLK -> J7-5]
#define SD_PIN_MOSI         3       // shared SPI2 MOSI [IC1-15 SPI2_MOSI -> J7-3]
#define SD_PIN_MISO         10      // [IC1-18 SD_MISO <- J7-7]
#define SD_PIN_CS           47      // [IC1-24 SD_CS -> J7-2]
#ifndef SD_MOUNT_POINT              // the host tests point it at a folder
#define SD_MOUNT_POINT      "/sdcard"
#endif
// Card-detect switch closes to GND when a card is seated. Advisory only: it
// never blocks a mount. Comment out SD_DET_GPIO to fall back to mount state.
#define SD_DET_GPIO         48      // [IC1-25 SD_DET <- J7 card-detect switch]
#define SD_DET_ACTIVE_LOW   1

// ---- 1.9" ST7789V LCD (170x320) on J2, sharing SPI2 with the SD card ----
// Whichever of sd_mount()/display_init() runs first initializes the bus, so
// the two bus configs must stay identical. J2 has no backlight pin (BLK is
// tied high), so "display off" sleeps the panel but leaves the backlight on.
#define LCD_SPI_HOST        SPI2_HOST
#define LCD_PIN_SCLK        11      // = SD_PIN_CLK (shared bus) [J2-4]
#define LCD_PIN_MOSI        3       // = SD_PIN_MOSI; aka SDA / DIN [J2-3]
#define LCD_PIN_CS          8       // [IC1-12 OLED_CS -> J2-5]
#define LCD_PIN_DC          18      // [IC1-11 OLED_DC -> J2-6]
#define LCD_PIN_RST         17      // [IC1-10 OLED_RST -> J2-7]
#define LCD_W               170     // portrait: the long side runs front -> back
#define LCD_H               320
// 26 MHz left the panel blank; SCLK/MOSI go through the GPIO matrix.
#define LCD_SPI_HZ          (10 * 1000 * 1000)
// Bring-up: show R/G/B/W bars for this many ms after init. 0 = off.
#define LCD_BOOT_TEST       0
// 240x320 controller RAM; the 170-column glass is at columns 35..204. MADCTL
// 0x00 is portrait in the panel's native scan; 0xC0 turns it 180 degrees.
#define LCD_MADCTL          0x00
#define LCD_X_OFS           35
#define LCD_Y_OFS           0
#define LCD_INVERT          1       // IPS ST7789 modules need INVON for correct colours

// ---- Power path: BQ25180 charger + MAX17048 gauge + RV-3028 RTC on I2C ----
// See power.c and docs/power-topology.md.
#define PWR_I2C_PORT        0
#define PWR_I2C_SDA_GPIO    2       // [IC1-38 SDA]  5.11k pull-up R61; also RV-3028 RTC (Y2)
#define PWR_I2C_SCL_GPIO    1       // [IC1-39 SCL]  5.11k pull-up R62; also RV-3028 RTC (Y2)
#define PWR_I2C_FREQ_HZ     400000
#define MAX17048_I2C_ADDR   0x36    // fuel gauge (SoC %, VCELL)
#define BQ25180_I2C_ADDR    0x6A    // charger (STAT0)

// Fast-charge current. Provisional: the BQ25180 is a linear charger, so this
// needs bench validation of case temperature and `thermreg` before raising.
// See docs/power-topology.md.
#define BQ25180_ICHG_TARGET_MA  600

// Input current limit. This is what really gates charging: it caps the whole
// board's draw from VIN. 1100 mA (the next step) would sag VBUS below the cell
// on the measured 0.845 ohm supply path. Only safe to raise with VINDPM on.
#define BQ25180_ILIM_TARGET_MA  700

// Input droop threshold (4200/4500/4700, 0 = off): the charger backs off to
// hold VIN above it. 4500 would throttle at currents this board already
// sustains (VBUS measured 4.62 V at 442 mA).
#define BQ25180_VINDPM_MV       4200

// Discharge over-current limit (500/1000/1500, 0 = off). The 1000 mA default
// tripped when the drive spun up on battery, and after repeated trips the
// BATFET stays off until USB is connected (datasheet 8.3.7.5). 1500 is the
// highest real limit, kept on as short-circuit protection.
#define BQ25180_IBAT_OCP_MA     1500

// Power-path GPIOs. The LTC2954 EN output drives the load-switch gate
// directly, not an ESP pin.
#define BOOST_EN_GPIO       9    // -> TPS61022 EN, 5 V drive rail, high = on [IC1-17].
                                 //    Needs an external pull-down (off at reset).
#define LTC_KILL_GPIO       38   // -> LTC2954 KILL via Q2, high = power off [IC1-31]
#define LTC_INT_GPIO        40   // <- LTC2954 INT, active-low shutdown request [IC1-33]
// Both ICs are also polled every 2 s; ALRT is edge-triggered too. Comment out
// on a board that doesn't fit them.
#define BQ_INT_GPIO         13   // <- BQ25180 /INT, active-low [IC1-21]
#define MAX_ALRT_GPIO       39   // <- MAX17048 ALRT, active-low, R19 pull-up [IC1-32]

// ---- MAX17048 alert thresholds (ALRT latches until CONFIG.ALRT is cleared) ----
// VALRT has 20 mV resolution; values are rounded down.

// Above the true cutoff because the cell sags well below its resting voltage
// under drive load. 3.40 V resting is roughly 10-15 % on this pack.
#define MAX17048_VALRT_MIN_MV   3400

// Above VBATREG (4.20 V) means a charger fault, not a full battery. This is
// the only independent check on the charger.
#define MAX17048_VALRT_MAX_MV   4300

// Empty alert in percent (1..32), stored as ATHD = 32 - pct. Generous margin
// because SOC on an uncharacterised pack is the least reliable reading.
#define MAX17048_ATHD_PCT       10

// Pin-map notes (ESP32-S3-WROOM-1-N16R8, 16 MB flash, octal PSRAM):
// GPIO 33..37 belong to the PSRAM; 19/20 are native USB; 43/44 (UART0) go to
// J3. GPIO3 and 45/46 are strapping pins: 45/46 are unconnected and GPIO3
// (SD/LCD MOSI) is high-Z at boot. Every usable pin is assigned.

// ---- WiFi ----
// Credentials come from NVS (namespace "wifi"); these are only the fallback.
// Keep them as placeholders so no secret lands in git.
#define WIFI_SSID           "CHANGEME"
#define WIFI_PASS           "CHANGEME"

// Open SoftAP for the captive setup portal when no WiFi is stored.
#define PROVISION_AP_SSID   "Floppy-Setup"
// Battery warnings (gauge state of charge, on battery only): power_low_level().
#define BATT_LOW_PCT        15
#define BATT_CRIT_PCT       5

// On battery, switch off after this long with no button press, web action or
// drive work; the LCD counts down the last AUTO_OFF_WARN_S.
#define AUTO_OFF_IDLE_S     300
#define AUTO_OFF_WARN_S     30

// Hold SAVE + DISP this long to forget the WiFi network and reboot to setup.
#define WIFI_RESET_HOLD_MS  5000

#define WIFI_CONNECT_TIMEOUT_MS 15000      // don't block the read path on WiFi
#define MDNS_HOSTNAME       "floppy"       // -> floppy.local
#define MDNS_INSTANCE       "Floppy Bridge"
