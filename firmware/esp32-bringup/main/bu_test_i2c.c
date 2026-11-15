// I2C bus on SDA=GPIO2 / SCL=GPIO1 (5.11k pull-ups R61/R62). Three parts hang
// on it: MAX17048 fuel gauge (0x36, powered from VBAT — silent with no
// battery), BQ25180 charger (0x6A, powered from VBUS or VBAT), RV-3028-C7 RTC
// (0x52, 3V3). Identity registers are read and the RTC is watched ticking.
#include "bu_tests.h"
#include "bu_report.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "app_config.h"

static const char *TAG = "t_i2c";

#define RV3028_I2C_ADDR   0x52
#define I2C_BRINGUP_HZ    100000     // conservative for first power-on

static i2c_master_bus_handle_t s_bus;

static bool rd(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, n, 100) == ESP_OK;
}

static i2c_master_dev_handle_t add(uint16_t addr)
{
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = I2C_BRINGUP_HZ,
    };
    i2c_master_dev_handle_t d = NULL;
    if (i2c_master_bus_add_device(s_bus, &dc, &d) != ESP_OK) return NULL;
    return d;
}

static void idle_levels(void)
{
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PWR_I2C_SDA_GPIO) | (1ULL << PWR_I2C_SCL_GPIO),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&in);
    int sda = gpio_get_level(PWR_I2C_SDA_GPIO), scl = gpio_get_level(PWR_I2C_SCL_GPIO);
    bu_report("I2C idle levels", (sda && scl) ? BU_PASS : BU_FAIL,
              "SDA=%d SCL=%d (both 1 via R61/R62; 0 = short or a part holding the line)", sda, scl);
}

static void max17048(void)
{
    i2c_master_dev_handle_t d = add(MAX17048_I2C_ADDR);
    uint8_t b[2];
    if (!d || !rd(d, 0x08, b, 2)) {
        bu_report("I2C MAX17048 (0x36)", BU_WARN, "no ACK — expected with NO battery (IC8 is VBAT-powered)");
        if (d) i2c_master_bus_rm_device(d);
        return;
    }
    uint16_t ver = (b[0] << 8) | b[1];
    uint16_t vcell = 0, soc = 0;
    if (rd(d, 0x02, b, 2)) vcell = (b[0] << 8) | b[1];
    if (rd(d, 0x04, b, 2)) soc   = (b[0] << 8) | b[1];
    // Judge the part on the measurements it exists to make: a sane cell voltage
    // and SoC prove the gauge is alive and reading VBAT. VERSION varies across
    // die revisions (0x0010..0x001F documented, other values seen in the wild),
    // so it is reported but not used as the pass criterion.
    unsigned mv = (unsigned)((uint32_t)vcell * 5 / 64);
    unsigned pct = soc >> 8;
    bool sane = (mv >= 2500 && mv <= 4500 && pct <= 100);
    bu_report("I2C MAX17048 (0x36)", sane ? BU_PASS : BU_WARN,
              "VERSION 0x%04X, VCELL %u mV, SoC %u%%%s", ver, mv, pct,
              sane ? "" : " — readings out of range for a 1S LiPo");
    i2c_master_bus_rm_device(d);
}

static void bq25180(void)
{
    i2c_master_dev_handle_t d = add(BQ25180_I2C_ADDR);
    uint8_t st0 = 0, st1 = 0, id = 0;
    if (!d || !rd(d, 0x00, &st0, 1)) {
        bu_report("I2C BQ25180 (0x6A)", BU_FAIL, "no ACK (IC5 is powered from VBUS or VBAT — should answer on USB)");
        if (d) i2c_master_bus_rm_device(d);
        return;
    }
    rd(d, 0x01, &st1, 1);
    rd(d, 0x0C, &id, 1);
    bu_report("I2C BQ25180 (0x6A)", BU_PASS, "STAT0 0x%02X STAT1 0x%02X MASK_ID 0x%02X", st0, st1, id);
    i2c_master_bus_rm_device(d);
}

static void rv3028(void)
{
    i2c_master_dev_handle_t d = add(RV3028_I2C_ADDR);
    uint8_t s0 = 0, s1 = 0, status = 0, id = 0;
    if (!d || !rd(d, 0x00, &s0, 1)) {
        bu_report("I2C RV-3028 RTC (0x52)", BU_FAIL, "no ACK (Y2)");
        if (d) i2c_master_bus_rm_device(d);
        return;
    }
    rd(d, 0x0E, &status, 1);
    rd(d, 0x28, &id, 1);
    vTaskDelay(pdMS_TO_TICKS(1200));
    rd(d, 0x00, &s1, 1);
    bool ticking = s1 != s0;
    bu_report("I2C RV-3028 RTC (0x52)", ticking ? BU_PASS : BU_FAIL,
              "ID 0x%02X STATUS 0x%02X%s, seconds %02X -> %02X (%s)", id, status,
              (status & 0x01) ? " [PORF: fresh power-on]" : "", s0, s1,
              ticking ? "32 kHz crystal running" : "NOT ticking — crystal/oscillator?");
    i2c_master_bus_rm_device(d);
}

void bu_test_i2c(void)
{
    ESP_LOGI(TAG, "--- I2C (SDA=%d SCL=%d) ---", PWR_I2C_SDA_GPIO, PWR_I2C_SCL_GPIO);
    idle_levels();

    i2c_master_bus_config_t bc = {
        .i2c_port                     = PWR_I2C_PORT,
        .sda_io_num                   = PWR_I2C_SDA_GPIO,
        .scl_io_num                   = PWR_I2C_SCL_GPIO,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bc, &s_bus) != ESP_OK) {
        bu_report("I2C bus", BU_FAIL, "i2c_new_master_bus failed");
        return;
    }

    // No 0x08..0x77 sweep: on this board every address reports an ACK, via
    // i2c_master_probe() and via a real addressed write alike, so the sweep
    // only ever produced a 112-device lie. Worse, probing by writing touches
    // real parts. Probe just the three addresses the board actually has,
    // read-only, and let the per-part tests below do the real work.
    static const struct { uint16_t addr; const char *name; } expect[] = {
        { MAX17048_I2C_ADDR, "MAX17048" },
        { BQ25180_I2C_ADDR,  "BQ25180"  },
        { RV3028_I2C_ADDR,   "RV-3028"  },
    };
    char found[96] = "";
    int n = 0;
    for (size_t i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
        if (i2c_master_probe(s_bus, expect[i].addr, 50) == ESP_OK) {
            size_t l = strlen(found);
            snprintf(found + l, sizeof(found) - l, "%s%s@0x%02X",
                     n ? " " : "", expect[i].name, expect[i].addr);
            n++;
        }
    }
    bu_report("I2C expected parts", n == 3 ? BU_PASS : BU_WARN,
              "%d/3 ACK: %s%s", n, n ? found : "none",
              n == 3 ? "" : " (probe ACK detection is unreliable here — trust the per-part reads below)");

    max17048();
    bq25180();
    rv3028();

    i2c_del_master_bus(s_bus);
    s_bus = NULL;
}
