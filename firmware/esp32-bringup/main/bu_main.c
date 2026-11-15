// gw-esp32 PCB assembly bring-up — ESP32-S3 side.
//
// Boots, touches every peripheral wired to the ESP (see README.md for the
// list and the pass criteria), prints a PASS/FAIL table on both consoles,
// shows the outcome on the four LEDs and the OLED, then sits in an idle loop
// where the buttons and the power button can be exercised by hand:
//   S1 (BOOT)      re-run the whole sequence
//   S2 (SAVE)      toggle the 5 V drive rail (BOOST_EN) — meter J1 pins 1/3/5
//   S3 (PWER_DISP) invert the OLED
//   S5 (power)     LTC2954 INT -> firmware asserts KILL -> board powers off
//
// LED legend after a run:  LED1 = RP2354 (SWD/link)   LED2 = microSD
//                          LED3 = I2C parts           LED4 = ESP (flash/PSRAM/WiFi)
//   green PASS · red FAIL · amber WARN · blue LOOK · dim white not run
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "app_config.h"
#include "bu_report.h"
#include "bu_tests.h"
#include "bu_oled.h"
#include "bu_leds.h"

static const char *TAG = "bringup";

static void banner(void)
{
    printf("\n\n==== gw-esp32 BRING-UP  (ESP32-S3)  built %s %s  IDF %s ====\n", __DATE__, __TIME__, IDF_VER);
    printf("Pins under test (app_config.h):\n");
    printf("  GWSPI  SCLK %d MOSI %d MISO %d CS %d DRDY %d   RUN %d   SWCLK %d SWDIO %d\n",
           GW_SPI_SCLK_GPIO, GW_SPI_MOSI_GPIO, GW_SPI_MISO_GPIO, GW_SPI_CS_GPIO, GW_SPI_DRDY_GPIO,
           GW_RESET_GPIO, RP_SWCLK_GPIO, RP_SWDIO_GPIO);
    printf("  SPI2   CLK %d MOSI %d MISO %d | SD CS %d DET %d | OLED CS %d DC %d RST %d\n",
           SD_PIN_CLK, SD_PIN_MOSI, SD_PIN_MISO, SD_PIN_CS, SD_DET_GPIO, OLED_PIN_CS, OLED_PIN_DC, OLED_PIN_RST);
    printf("  I2C    SDA %d SCL %d (MAX17048 0x%02X, BQ25180 0x%02X, RV-3028 0x52)\n",
           PWR_I2C_SDA_GPIO, PWR_I2C_SCL_GPIO, MAX17048_I2C_ADDR, BQ25180_I2C_ADDR);
    printf("  UI     S1 %d S2 %d S3 %d | WS2812 x4 on %d\n",
           UI_BUTTON_GPIO, UI_BTN_SAVEALL_GPIO, UI_BTN_DISPLAY_GPIO, UI_LED_GPIO);
    printf("  POWER  BOOST_EN %d LTC_KILL %d LTC_INT %d BQ_INT %d MAX_ALRT %d\n\n",
           BOOST_EN_GPIO, LTC_KILL_GPIO, LTC_INT_GPIO, BQ_INT_GPIO, MAX_ALRT_GPIO);
}

// SPI2 carries the SD card and the OLED (same config as the production
// storage.c / display.c). Initialised once here; both users share it.
static void spi2_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = SD_PIN_MOSI,
        .miso_io_num     = SD_PIN_MISO,
        .sclk_io_num     = SD_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t e = spi_bus_initialize(SD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE)
        ESP_LOGE(TAG, "SPI2 init failed: %s", esp_err_to_name(e));
}

static void show_results(void)
{
    bu_report_summary();
    bu_leds_status(bu_report_worst("RP"), bu_report_worst("SD"),
                   bu_report_worst("I2C"), bu_report_worst("ESP"));

    char l[24];
    bu_oled_clear();
    bu_oled_text(0, 0, "BRINGUP RESULT");
    snprintf(l, sizeof(l), "RP2354  %s", bu_status_str(bu_report_worst("RP")));   bu_oled_text(0, 12, l);
    snprintf(l, sizeof(l), "SD CARD %s", bu_status_str(bu_report_worst("SD")));   bu_oled_text(0, 22, l);
    snprintf(l, sizeof(l), "I2C     %s", bu_status_str(bu_report_worst("I2C")));  bu_oled_text(0, 32, l);
    snprintf(l, sizeof(l), "ESP     %s", bu_status_str(bu_report_worst("ESP")));  bu_oled_text(0, 42, l);
    snprintf(l, sizeof(l), "PASS %d FAIL %d", bu_report_count(BU_PASS), bu_report_count(BU_FAIL));
    bu_oled_text(0, 54, l);
    bu_oled_flush();
}

static void run_all(void)
{
    bu_report_reset();
    printf("\n---- running tests ----\n");

    bu_leds_chase();
    bu_report("LEDs WS2812 x4", BU_VISUAL, "each LED walked R,G,B in chain order, then all white");

    bu_oled_test_pattern("PATTERN CHECK");
    bu_report("OLED SSD1309", BU_VISUAL, "border + text + checkerboard visible? (write-only panel)");

    bu_test_esp();
    bu_test_power_levels();
    bu_test_i2c();
    bu_test_sd();
    bu_test_rp();
    bu_test_link();

    // 5 V drive rail: on for 3 s with the LEDs blue so the moment is obvious.
    printf("  BOOST_EN high for 3 s — meter 5 V on J1 pins 1/3/5 (S2 toggles it later)\n");
    bu_leds_all(0, 60, 255); bu_leds_flush();
    bu_boost_5v(true);
    vTaskDelay(pdMS_TO_TICKS(3000));
    bu_boost_5v(false);

    show_results();
}

// ---- idle loop: buttons, boost toggle, LTC2954 power-button path ----

typedef struct { const char *name; int gpio; int stable; int pressed; } btn_t;

static void idle_loop(void)
{
    btn_t b[3] = {
        { "S1/BOOT", UI_BUTTON_GPIO, 0, 0 },
        { "S2/SAVE", UI_BTN_SAVEALL_GPIO, 0, 0 },
        { "S3/PWER_DISP", UI_BTN_DISPLAY_GPIO, 0, 0 },
    };
    bool boost = false, inv = false;
    int ltc_low = 0, last_bq = -1, last_mx = -1;
    int64_t last_beat = esp_timer_get_time();

    printf("idle: S1 = re-run, S2 = 5 V rail on/off, S3 = OLED invert, S5 (power button) = KILL test\n");
    for (;;) {
        for (int i = 0; i < 3; i++) {
            if (gpio_get_level(b[i].gpio) == 0) {
                if (b[i].stable < 1000) b[i].stable++;
                if (b[i].stable == 3 && !b[i].pressed) {
                    b[i].pressed = 1;
                    printf("  button %s PRESSED (GPIO%d reads 0)\n", b[i].name, b[i].gpio);
                }
            } else {
                if (b[i].pressed) {
                    printf("  button %s released\n", b[i].name);
                    switch (i) {
                    case 0: run_all(); break;
                    case 1:
                        boost = !boost;
                        bu_boost_5v(boost);
                        printf("  BOOST_EN (GPIO%d) -> %s: meter J1 pins 1/3/5 for 5 V\n", BOOST_EN_GPIO, boost ? "ON" : "off");
                        if (boost) { bu_leds_all(0, 60, 255); bu_leds_flush(); }
                        else       show_results();
                        break;
                    case 2:
                        inv = !inv;
                        bu_oled_invert(inv);
                        printf("  OLED invert -> %s\n", inv ? "on" : "off");
                        break;
                    }
                }
                b[i].stable = 0;
                b[i].pressed = 0;
            }
        }

        // Power button (S5 -> LTC2954 PB -> INT low). Mirror production: ask
        // the LTC to cut power via KILL. If we are still alive 3 s later the
        // KILL path (GPIO38 -> R8 -> IC9 -> IC6 KILL) is broken.
        if (gpio_get_level(LTC_INT_GPIO) == 0) {
            if (++ltc_low == 3) {
                printf("  LTC2954 INT asserted (S5 pressed) -> asserting KILL in 2 s; board should power off\n");
                bu_oled_clear(); bu_oled_text(0, 20, "POWER OFF..."); bu_oled_flush();
                bu_leds_all(255, 0, 0); bu_leds_flush();
                vTaskDelay(pdMS_TO_TICKS(2000));
                bu_ltc_kill();
                vTaskDelay(pdMS_TO_TICKS(3000));
                printf("  [FAIL] LTC2954 KILL             still running 3 s after KILL — GPIO38/R8/IC9/IC6 path\n");
                gpio_set_level(LTC_KILL_GPIO, 0);
                show_results();
            }
        } else {
            ltc_low = 0;
        }

        int bq = gpio_get_level(BQ_INT_GPIO), mx = gpio_get_level(MAX_ALRT_GPIO);
        if (bq != last_bq) { if (last_bq >= 0) printf("  BQ25180 /INT -> %d\n", bq); last_bq = bq; }
        if (mx != last_mx) { if (last_mx >= 0) printf("  MAX17048 ALRT -> %d\n", mx); last_mx = mx; }

        int64_t now = esp_timer_get_time();
        if (now - last_beat > 15 * 1000000LL) {
            last_beat = now;
            printf("  alive %llds  PASS %d FAIL %d WARN %d\n", (long long)(now / 1000000),
                   bu_report_count(BU_PASS), bu_report_count(BU_FAIL), bu_report_count(BU_WARN));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void app_main(void)
{
    bu_test_power_io_init();      // safe outputs first: BOOST off, KILL deasserted
    banner();
    bu_leds_init();
    spi2_init();
    bu_oled_init();
    run_all();
    idle_loop();
}
