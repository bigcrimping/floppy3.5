#include "bu_leds.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "app_config.h"     // UI_LED_GPIO

static const char *TAG = "leds";
#define LED_N   4
#define LED_MAX 52          // ~20 %: same cap as the production build

static led_strip_handle_t s_strip;
static bool s_ok;

bool bu_leds_init(void)
{
    if (s_ok) return true;
    led_strip_config_t cfg = {
        .strip_gpio_num   = UI_LED_GPIO,
        .max_leds         = LED_N,
        .led_model        = LED_MODEL_WS2812,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    if (led_strip_new_rmt_device(&cfg, &rmt, &s_strip) != ESP_OK) {
        ESP_LOGW(TAG, "led_strip init failed");
        return false;
    }
    led_strip_clear(s_strip);
    s_ok = true;
    return true;
}

static inline uint8_t cap(uint8_t v) { return (uint8_t)((uint32_t)v * LED_MAX / 255u); }

void bu_leds_set(int i, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_ok || i < 0 || i >= LED_N) return;
    led_strip_set_pixel(s_strip, i, cap(r), cap(g), cap(b));
}

void bu_leds_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_N; i++) bu_leds_set(i, r, g, b);
}

void bu_leds_flush(void) { if (s_ok) led_strip_refresh(s_strip); }

void bu_leds_chase(void)
{
    if (!s_ok) return;
    static const uint8_t C[3][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 } };
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < LED_N; i++) {
            bu_leds_all(0, 0, 0);
            bu_leds_set(i, C[c][0], C[c][1], C[c][2]);
            bu_leds_flush();
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }
    bu_leds_all(255, 255, 255);   // all-on white: current draw / common-supply check
    bu_leds_flush();
    vTaskDelay(pdMS_TO_TICKS(300));
    bu_leds_all(0, 0, 0);
    bu_leds_flush();
}

static void colour(int i, bu_status_t st)
{
    switch (st) {
    case BU_PASS:   bu_leds_set(i, 0, 255, 40);    break;   // green
    case BU_FAIL:   bu_leds_set(i, 255, 0, 0);     break;   // red
    case BU_WARN:   bu_leds_set(i, 255, 140, 0);   break;   // amber
    case BU_VISUAL: bu_leds_set(i, 0, 60, 255);    break;   // blue
    default:        bu_leds_set(i, 40, 40, 40);    break;   // dim white
    }
}

void bu_leds_status(bu_status_t l1, bu_status_t l2, bu_status_t l3, bu_status_t l4)
{
    colour(0, l1); colour(1, l2); colour(2, l3); colour(3, l4);
    bu_leds_flush();
}
