#include "bu_oled.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "app_config.h"     // OLED_* pins, OLED_W/H
#include "font8x8.h"

static const char *TAG = "oled";

static spi_device_handle_t s_spi;
static bool    s_ok;
static uint8_t s_fb[OLED_W * OLED_H / 8];

static void send(bool data, const uint8_t *buf, size_t len)
{
    if (!s_ok) return;
    gpio_set_level(OLED_PIN_DC, data ? 1 : 0);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = buf };
    spi_device_polling_transmit(s_spi, &t);
}
static void cmd(uint8_t c) { send(false, &c, 1); }

// Same sequence as the production display.c (SSD1306-compatible init that the
// SSD1309 accepts).
static const uint8_t INIT_SEQ[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
    0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF,
};

bool bu_oled_init(void)
{
    if (s_ok) return true;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << OLED_PIN_DC) | (1ULL << OLED_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = OLED_PIN_CS,
        .queue_size     = 1,
    };
    if (spi_bus_add_device(OLED_SPI_HOST, &dev, &s_spi) != ESP_OK) {
        ESP_LOGW(TAG, "spi_bus_add_device failed");
        return false;
    }
    gpio_set_level(OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    s_ok = true;
    for (size_t i = 0; i < sizeof(INIT_SEQ); i++) cmd(INIT_SEQ[i]);
    return true;
}

static inline void px(int x, int y, int on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    uint32_t idx = (uint32_t)x + (uint32_t)(y >> 3) * OLED_W;
    uint8_t  m = 1 << (y & 7);
    if (on) s_fb[idx] |= m; else s_fb[idx] &= ~m;
}

void bu_oled_clear(void) { memset(s_fb, 0, sizeof(s_fb)); }

void bu_oled_rect(int x, int y, int w, int h, int on)
{
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++) px(xx, yy, on);
}

void bu_oled_text(int x, int y, const char *s)
{
    for (; *s; s++, x += 8) {
        const uint8_t *g = font8x8_basic[(uint8_t)*s & 0x7F];
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 8; c++)
                if (g[r] & (1 << c)) px(x + c, y + r, 1);
    }
}

void bu_oled_flush(void)
{
    cmd(0x21); cmd(0); cmd(OLED_W - 1);
    cmd(0x22); cmd(0); cmd(OLED_H / 8 - 1);
    send(true, s_fb, sizeof(s_fb));
}

void bu_oled_invert(bool inv) { cmd(inv ? 0xA7 : 0xA6); }

// Border (edge-pixel check), a checkerboard block (pixel pitch / contrast),
// and two text lines. If any part is missing or smeared, the panel, its
// ribbon, or the SPI2 lines are suspect.
void bu_oled_test_pattern(const char *line2)
{
    bu_oled_clear();
    bu_oled_rect(0, 0, OLED_W, 1, 1);
    bu_oled_rect(0, OLED_H - 1, OLED_W, 1, 1);
    bu_oled_rect(0, 0, 1, OLED_H, 1);
    bu_oled_rect(OLED_W - 1, 0, 1, OLED_H, 1);
    for (int y = 0; y < 16; y++)
        for (int x = 0; x < 32; x++)
            px(96 + x, 40 + y, ((x >> 2) + (y >> 2)) & 1);
    bu_oled_text(4, 4,  "GW-ESP32 BRINGUP");
    bu_oled_text(4, 16, line2);
    bu_oled_flush();
}
