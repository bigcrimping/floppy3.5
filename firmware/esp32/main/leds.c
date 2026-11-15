// led_strip over RMT; one task owns the strip. Brightness is capped (LED_MAX)
// because four WS2812s at full white draw ~240 mA.
#include "leds.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"

#include "app_config.h"
#include "disk_image.h"
#include "power.h"

static const char *TAG = "leds";

#define LED_N 4
#define LED_MAX 52          // ~20% of 255: plenty indoors, ~50 mA all-on
#define FRAME_MS 40

typedef struct { uint8_t r, g, b; } rgb_t;

static const rgb_t C_OFF    = {   0,   0,   0 };
static const rgb_t C_IDLE   = {  40,  40,  40 };   // dim white
static const rgb_t C_BUSY   = {   0,  60, 255 };   // blue, progress bar
static const rgb_t C_OK     = {   0, 255,  40 };   // green
static const rgb_t C_DIRTY  = { 255, 140,   0 };   // amber: bad sectors
static const rgb_t C_ERR    = { 255,   0,   0 };   // red
static const rgb_t C_USB    = {   0, 120, 200 };   // powered, no battery data

static led_strip_handle_t s_strip;
static bool s_ok;                       // strip initialised
static volatile bool s_enabled = true;
static volatile int  s_state = DISK_EMPTY;
static volatile int64_t s_batt_until_us;   // battery gauge shown until this time

static inline uint8_t scale(uint8_t v)
{
    return (uint8_t)((uint32_t)v * LED_MAX / 255u);
}

// Triangle wave for the pulsing pixel.
static uint8_t pulse_level(void)
{
    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000) % 1600u;
    uint32_t up = ms < 800u ? ms : 1600u - ms;      // 0..800..0
    return (uint8_t)(40u + up * 215u / 800u);       // never fully dark
}

static void put(int i, rgb_t c, uint8_t level /*0..255*/)
{
    uint32_t r = (uint32_t)scale(c.r) * level / 255u;
    uint32_t g = (uint32_t)scale(c.g) * level / 255u;
    uint32_t b = (uint32_t)scale(c.b) * level / 255u;
    led_strip_set_pixel(s_strip, i, r, g, b);
}

// ---- the two display modes ----

// 4-segment bar: full pixels solid, the one in progress pulsing.
static void draw_progress(int pct)
{
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    int filled = pct * LED_N / 100;             // 0..4 whole segments
    for (int i = 0; i < LED_N; i++) {
        if (i < filled)        put(i, C_BUSY, 255);
        else if (i == filled)  put(i, C_BUSY, pulse_level());
        else                   put(i, C_OFF, 0);
    }
}

static void draw_state(int st)
{
    rgb_t c;
    switch (st) {
    case DISK_READY:       c = C_OK;    break;
    case DISK_READY_DIRTY: c = C_DIRTY; break;
    case DISK_ERROR:       c = C_ERR;   break;
    case DISK_READING:     c = C_BUSY;  break;
    default:               c = C_IDLE;  break;
    }
    for (int i = 0; i < LED_N; i++)
        put(i, c, 255);
}

// One pixel per 25%, the next one pulsing while charging. No gauge reading:
// all pixels in the USB colour.
static void draw_battery(void)
{
    power_state_t ps;
    if (!power_get(&ps) || ps.soc < 0) {
        for (int i = 0; i < LED_N; i++)
            put(i, C_USB, 255);
        return;
    }

    int soc = ps.soc;
    if (soc > 100) soc = 100;
    if (soc < 0)   soc = 0;
    rgb_t c = (soc >= 50) ? C_OK : (soc >= 20 ? C_DIRTY : C_ERR);

    // Round up: an empty strip should mean flat, not under 25%.
    int lit = (soc * LED_N + (100 / LED_N) - 1) / 100;
    if (lit < 1 && soc > 0) lit = 1;

    for (int i = 0; i < LED_N; i++) {
        if (i < lit)                              put(i, c, 255);
        else if (ps.charging && i == lit)         put(i, c, pulse_level());
        else                                      put(i, C_OFF, 0);
    }
}

static void led_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!s_enabled) {
            led_strip_clear(s_strip);
        } else if (esp_timer_get_time() < s_batt_until_us) {
            draw_battery();
            led_strip_refresh(s_strip);
        } else {
            if (s_state == DISK_READING)
                draw_progress((int)(disk_sectors_done() * 100u / disk_sectors_total()));
            else
                draw_state(s_state);
            led_strip_refresh(s_strip);
        }
        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}

// ---- public API ----

void leds_init(void)
{
    led_strip_config_t cfg = {
        .strip_gpio_num = UI_LED_GPIO,
        .max_leds = LED_N,
        .led_model = LED_MODEL_WS2812,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    if (led_strip_new_rmt_device(&cfg, &rmt, &s_strip) != ESP_OK) {
        ESP_LOGW(TAG, "WS2812 init failed — status LEDs disabled");
        return;
    }
    led_strip_clear(s_strip);
    s_ok = true;
    xTaskCreatePinnedToCore(led_task, "leds", 2560, NULL, 2, NULL, 0);
    ESP_LOGI(TAG, "%d status LEDs on GPIO%d", LED_N, UI_LED_GPIO);
}

void leds_set_state(int disk_state)
{
    if (!s_ok) return;
    s_state = disk_state;
}

void leds_show_battery(void)
{
    if (!s_ok) return;
    s_batt_until_us = esp_timer_get_time() + (int64_t)LED_BATTERY_MS * 1000;
}

void leds_set_enabled(bool on)
{
    s_enabled = on;
}

bool leds_enabled(void) { return s_enabled; }
