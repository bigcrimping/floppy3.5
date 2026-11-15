// Power-path and UI GPIOs. Nothing here talks I2C (see bu_test_i2c.c); this is
// the plain-GPIO side: BOOST_EN, LTC2954 KILL/INT, the two optional IRQ
// lines, and the three buttons' idle levels.
#include "bu_tests.h"
#include "bu_report.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "app_config.h"

static const char *TAG = "t_pwr";

void bu_test_power_io_init(void)
{
    // Outputs first, driven to their safe state in the same call: BOOST off,
    // KILL deasserted. (R18 100k pull-down holds BOOST_EN low before this.)
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << BOOST_EN_GPIO) | (1ULL << LTC_KILL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);
    gpio_set_level(BOOST_EN_GPIO, 0);
    gpio_set_level(LTC_KILL_GPIO, 0);

    uint64_t in_mask = (1ULL << LTC_INT_GPIO)
#ifdef BQ_INT_GPIO
                     | (1ULL << BQ_INT_GPIO)
#endif
#ifdef MAX_ALRT_GPIO
                     | (1ULL << MAX_ALRT_GPIO)
#endif
                     | (1ULL << UI_BUTTON_GPIO) | (1ULL << UI_BTN_SAVEALL_GPIO)
                     | (1ULL << UI_BTN_DISPLAY_GPIO);
    gpio_config_t in = {
        .pin_bit_mask = in_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in);
}

void bu_test_power_levels(void)
{
    ESP_LOGI(TAG, "--- power-path / UI GPIO idle levels ---");

    int ltc = gpio_get_level(LTC_INT_GPIO);
    bu_report("LTC2954 INT (GPIO40)", ltc ? BU_PASS : BU_WARN,
              ltc ? "high (idle; goes low while S5 is pressed)" : "LOW at idle — S5 stuck, or IC6 INT pin fault");

#ifdef BQ_INT_GPIO
    int bq = gpio_get_level(BQ_INT_GPIO);
    bu_report("BQ25180 /INT (GPIO13)", BU_INFO, "%s (open-drain, pulses low on charger events)",
              bq ? "high" : "LOW");
#endif
#ifdef MAX_ALRT_GPIO
    int mx = gpio_get_level(MAX_ALRT_GPIO);
    bu_report("MAX17048 ALRT (GPIO39)", BU_INFO, "%s (open-drain; low = low-SoC alert)",
              mx ? "high" : "LOW");
#endif

    int s1 = gpio_get_level(UI_BUTTON_GPIO);
    int s2 = gpio_get_level(UI_BTN_SAVEALL_GPIO);
    int s3 = gpio_get_level(UI_BTN_DISPLAY_GPIO);
    bu_report("Buttons idle", (s1 && s2 && s3) ? BU_PASS : BU_FAIL,
              "S1/BOOT=%d S2/SAVE=%d S3/PWER_DISP=%d (all 1 expected; 0 = stuck/shorted)", s1, s2, s3);

    bu_report("BOOST_EN (GPIO9)", BU_VISUAL,
              "5 V rail pulsed for 3 s after the tests; S2 toggles it in idle — meter J1 pins 1/3/5");
    bu_report("LTC2954 KILL (GPIO38)", BU_SKIP,
              "tap S5 (power button) in idle: INT -> KILL -> board powers off; S5 again to restart");
}

void bu_boost_5v(bool on) { gpio_set_level(BOOST_EN_GPIO, on ? 1 : 0); }

void bu_ltc_kill(void)
{
    ESP_LOGW(TAG, "asserting LTC2954 KILL — power should drop now");
    gpio_set_level(LTC_KILL_GPIO, 1);
}
