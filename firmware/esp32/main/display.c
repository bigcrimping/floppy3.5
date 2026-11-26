#include "display.h"
#include "app_config.h"
#include "disk_image.h"
#include "netinfo.h"
#include "storage.h"
#include "power.h"
#include "ota.h"
#include "rp_flash.h"
#include "ui.h"
#include "font8x8.h"
#include "wifi_pwr.h"
#include "lastoff.h"
#include "serialno.h"
#include "temps.h"
#include "sd_index.h"
#include "esp_app_desc.h"
#include <time.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "qrcode.h"

static const char *TAG = "lcd";

// ---------------- panel driver (ST7789V, RGB565, portrait) ----------------
//
// A 106 KB PSRAM framebuffer, pushed through a small DMA buffer with CS held
// for the whole RAMWR (~90 ms at 10 MHz), so the SD card on the same bus
// waits rather than splitting it. Unchanged frames aren't sent.

#define LCD_CHUNK 4096                 // = the shared bus max_transfer_sz

static spi_device_handle_t s_spi;
static bool                 s_ok;       // panel brought up
static uint16_t            *s_fb;       // pixels in wire (big-endian) order
static uint8_t             *s_dma;      // LCD_CHUNK bytes, internal + DMA-capable
static uint32_t             s_last_sum; // checksum of the last frame sent

static void lcd_xfer(bool is_data, const void *buf, size_t len, bool keep_cs)
{
    gpio_set_level(LCD_PIN_DC, is_data ? 1 : 0);
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = buf,
        .flags     = keep_cs ? SPI_TRANS_CS_KEEP_ACTIVE : 0,
    };
    spi_device_polling_transmit(s_spi, &t);
}

// One command plus its parameter bytes, as a single CS-low burst.
static void lcd_cmd(uint8_t c, const uint8_t *p, size_t n)
{
    if (!s_ok) return;
    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    lcd_xfer(false, &c, 1, n > 0);
    if (n) lcd_xfer(true, p, n, false);
    spi_device_release_bus(s_spi);
}

static void lcd_flush(void)
{
    if (!s_ok || !s_fb) return;

    // FNV-1a, to skip identical frames.
    uint32_t sum = 2166136261u;
    const uint32_t *w = (const uint32_t *)s_fb;
    for (size_t i = 0; i < (size_t)LCD_W * LCD_H / 2; i++) sum = (sum ^ w[i]) * 16777619u;
    if (sum == s_last_sum) return;
    s_last_sum = sum;

    const uint16_t x0 = LCD_X_OFS, x1 = LCD_X_OFS + LCD_W - 1;
    const uint16_t y0 = LCD_Y_OFS, y1 = LCD_Y_OFS + LCD_H - 1;
    const uint8_t caset[4] = { x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF };
    const uint8_t raset[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };
    lcd_cmd(0x2A, caset, 4);   // CASET
    lcd_cmd(0x2B, raset, 4);   // RASET

    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    const uint8_t ramwr = 0x2C;
    lcd_xfer(false, &ramwr, 1, true);
    const uint8_t *src = (const uint8_t *)s_fb;
    size_t left = (size_t)LCD_W * LCD_H * 2;
    while (left) {
        size_t n = left > LCD_CHUNK ? LCD_CHUNK : left;
        memcpy(s_dma, src, n);             // PSRAM -> internal DMA buffer
        left -= n; src += n;
        lcd_xfer(true, s_dma, n, left > 0);
    }
    spi_device_release_bus(s_spi);
}

static bool lcd_hw_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    s_fb  = heap_caps_malloc((size_t)LCD_W * LCD_H * 2, MALLOC_CAP_SPIRAM);
    if (!s_fb) s_fb = heap_caps_malloc((size_t)LCD_W * LCD_H * 2, MALLOC_CAP_8BIT);
    s_dma = heap_caps_malloc(LCD_CHUNK, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_fb || !s_dma) {
        ESP_LOGW(TAG, "LCD framebuffer alloc failed (fb %p dma %p)", s_fb, s_dma);
        return false;
    }

    // Shared with the SD card, which normally initializes it first. Must stay
    // identical to storage.c's config so either init works.
    spi_bus_config_t bus = {
        .mosi_io_num     = LCD_PIN_MOSI,
        .miso_io_num     = SD_PIN_MISO,     // the LCD has none
        .sclk_io_num     = LCD_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t e = spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {   // already initialized
        ESP_LOGW(TAG, "LCD SPI bus init failed: %s", esp_err_to_name(e));
        return false;
    }
    spi_device_interface_config_t dev = {
        .clock_speed_hz = LCD_SPI_HZ,
        .mode           = 0,
        .spics_io_num   = LCD_PIN_CS,
        .queue_size     = 1,
    };
    if (spi_bus_add_device(LCD_SPI_HOST, &dev, &s_spi) != ESP_OK) {
        ESP_LOGW(TAG, "LCD SPI add device failed");
        return false;
    }

    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    s_ok = true;                               // enable lcd_cmd before init
    lcd_cmd(0x01, NULL, 0);                    // SWRESET
    vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(0x11, NULL, 0);                    // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(120));
    const uint8_t colmod = 0x55;               // 16 bpp RGB565
    lcd_cmd(0x3A, &colmod, 1);
    const uint8_t madctl = LCD_MADCTL;
    lcd_cmd(0x36, &madctl, 1);
    lcd_cmd(LCD_INVERT ? 0x21 : 0x20, NULL, 0); // INVON / INVOFF
    lcd_cmd(0x13, NULL, 0);                    // NORON
    lcd_cmd(0x29, NULL, 0);                    // DISPON
    ESP_LOGI(TAG, "ST7789 init sent (SCLK %d MOSI %d CS %d DC %d RST %d, %d Hz, MADCTL 0x%02X, ofs %d,%d)",
             LCD_PIN_SCLK, LCD_PIN_MOSI, LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_RST, LCD_SPI_HZ,
             LCD_MADCTL, LCD_X_OFS, LCD_Y_OFS);

#if LCD_BOOT_TEST
    // Fill the whole 240x320 RAM with R/G/B/W bars, visible even with wrong
    // offsets. Black = no valid commands; wrong hues = flip LCD_INVERT or the
    // MADCTL BGR bit (0x08); bars cut off = adjust the offsets.
    static const uint16_t bars[4] = {
        0x00F8 /* red */, 0xE007 /* green */, 0x1F00 /* blue */, 0xFFFF /* white */ };
    const uint8_t cas[4] = { 0, 0, 0, 239 };
    const uint8_t ras[4] = { 0, 0, 319 >> 8, 319 & 0xFF };
    lcd_cmd(0x2A, cas, 4);
    lcd_cmd(0x2B, ras, 4);
    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    const uint8_t wr = 0x2C;
    lcd_xfer(false, &wr, 1, true);
    uint16_t *line = (uint16_t *)s_dma;        // one 240-px row = 480 B
    for (int x = 0; x < 240; x++) line[x] = bars[x / 60];
    for (int y = 0; y < 320; y++) lcd_xfer(true, line, 480, y < 319);
    spi_device_release_bus(s_spi);
    ESP_LOGI(TAG, "colour-bar test drawn; holding %d ms", LCD_BOOT_TEST);
    vTaskDelay(pdMS_TO_TICKS(LCD_BOOT_TEST));
#endif
    return true;
}

// ---------------- colours ----------------
// "Walkman LCD": grey-green field with dark and ghost segments. Flat fills
// only, since gradients band in RGB565.

#define RGB(r, g, b)  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define SW(c)         ((uint16_t)(((c) >> 8) | ((c) << 8)))   // wire (big-endian) order

enum {
    C_LCD      = SW(RGB(0xB7, 0xBF, 0xA3)),
    C_INK      = SW(RGB(0x1E, 0x25, 0x19)),
    C_DIM      = SW(RGB(0x55, 0x60, 0x4A)),
    C_GHOST    = SW(RGB(0xA9, 0xB1, 0x97)),
};

// ---------------- framebuffer drawing ----------------
//
// Portrait, 170 x 320, top = the back of the box. The LCD field fills the
// glass; the button names are printed on the housing beside it.

#define FX        6                    // content left edge
#define FR        (LCD_W - 6)          // content right edge
#define FW        (FR - FX)            // 158 px = 19 characters at sx=1

static inline void px(int x, int y, uint16_t c)
{
    if ((unsigned)x < LCD_W && (unsigned)y < LCD_H) s_fb[y * LCD_W + x] = c;
}

static void fill(int x, int y, int w, int h, uint16_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    for (int yy = y; yy < y + h; yy++) {
        uint16_t *row = s_fb + yy * LCD_W + x;
        for (int i = 0; i < w; i++) row[i] = c;
    }
}

// font8x8 with separate x/y scale; sx=1, sy=2 gives the tall dot-matrix look
// and fits long strings on a line.
static void text(int x, int y, const char *s, int sx, int sy, uint16_t c)
{
    for (; *s; s++, x += 8 * sx) {
        const uint8_t *g = font8x8_basic[(uint8_t)*s & 0x7F];
        for (int r = 0; r < 8; r++)
            for (int k = 0; k < 8; k++)
                if (g[r] & (1 << k))                       // bit0 = leftmost
                    fill(x + k * sx, y + r * sy, sx, sy, c);
    }
}
static int text_w(const char *s, int sx) { return (int)strlen(s) * 8 * sx; }

// Left edge that centres something `w` px wide across the content width.
static int cx(int w) { return FX + (FW - w) / 2; }

static void text_c(int y, const char *s, int sx, int sy, uint16_t c)
{
    text(cx(text_w(s, sx)), y, s, sx, sy, c);
}

// Wrap into centred lines of at most `cols` characters, breaking at spaces
// where it can; returns the y after the last line.
static int text_wrap(int y, int cols, const char *s, int sx, int sy, int max_lines, uint16_t c)
{
    int len = (int)strlen(s), i = 0;
    for (int line = 0; line < max_lines && i < len; line++) {
        char buf[48];
        int k = len - i;
        if (k > cols) {
            k = cols;
            for (int j = cols; j > cols / 2; j--)
                if (s[i + j] == ' ') { k = j; break; }
        }
        if (k > (int)sizeof(buf) - 1) k = sizeof(buf) - 1;
        memcpy(buf, s + i, k);
        buf[k] = '\0';
        i += k;
        text_c(y, buf, sx, sy, c);
        y += 8 * sy + 3;
        while (i < len && s[i] == ' ') i++;
    }
    return y;
}

// ---- italic 7-segment digits with ghost segments ----
// Width is 0.55 h; segments are hex-ended and sheared 1 px per 6 rows.
static const uint8_t SEG7[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };  // gfedcba

static void seg_h(int x, int y, int w, int t, int base, uint16_t c)
{
    for (int r = 0; r < t; r++) {
        int d = abs(2 * r - (t - 1)) / 2, yy = y + r, sk = (base - yy) / 6;
        fill(x + d + sk, yy, w - 2 * d, 1, c);
    }
}
static void seg_v(int x, int y, int h, int t, int base, uint16_t c)
{
    for (int yy = y; yy < y + h; yy++) {
        int sk = (base - yy) / 6;
        for (int k = 0; k < t; k++) {
            int d = abs(2 * k - (t - 1)) / 2;
            if (yy >= y + d && yy < y + h - d) px(x + k + sk, yy, c);
        }
    }
}
static int digit_w(int h) { return h * 55 / 100; }

static void digit(int x, int y, int h, int v, uint16_t on)
{
    int w = digit_w(h), t = h / 8 < 2 ? 2 : h / 8, g = 1, half = h / 2, base = y + h;
    uint8_t m = (v >= 0 && v <= 9) ? SEG7[v] : (v == '-' ? 0x40 : 0);
    struct { int horiz, sx, sy, len; } S[7] = {
        { 1, x + t / 2 + g, y,                 w - t - 2 * g },          // a
        { 0, x + w - t,     y + t / 2 + g,     half - t - 2 * g + t / 2 },// b
        { 0, x + w - t,     y + half + g,      half - t - 2 * g + t / 2 },// c
        { 1, x + t / 2 + g, y + h - t,         w - t - 2 * g },          // d
        { 0, x,             y + half + g,      half - t - 2 * g + t / 2 },// e
        { 0, x,             y + t / 2 + g,     half - t - 2 * g + t / 2 },// f
        { 1, x + t / 2 + g, y + half - t / 2,  w - t - 2 * g },          // g
    };
    for (int i = 0; i < 7; i++) {
        uint16_t c = (m & (1 << i)) ? on : C_GHOST;
        if (S[i].horiz) seg_h(S[i].sx, S[i].sy, S[i].len, t, base, c);
        else            seg_v(S[i].sx, S[i].sy, S[i].len, t, base, c);
    }
}

// Draw a string of digits / '-' / ' ' / ':'; returns the x after it.
static int digits(int x, int y, int h, const char *s, uint16_t on)
{
    int adv = digit_w(h) + h / 6;
    for (; *s; s++) {
        if (*s == ':') {
            int d = h / 9 < 2 ? 2 : h / 9, sk = h / 12;
            fill(x + sk + 1, y + h / 3 - d / 2, d, d, on);
            fill(x + 1, y + 2 * h / 3 - d / 2, d, d, on);
            x += d + h / 8;
            continue;
        }
        digit(x, y, h, *s >= '0' && *s <= '9' ? *s - '0' : *s, on);
        x += adv;
    }
    return x;
}

// Width digits() will advance for `s` (includes the italic lean).
static int digits_w(int h, const char *s)
{
    int w = 0;
    for (; *s; s++) {
        if (*s == ':') w += (h / 9 < 2 ? 2 : h / 9) + h / 8;
        else           w += digit_w(h) + h / 6;
    }
    return w;
}

// ---- printed icons: always visible as ghosts, switched on in ink ----
// 16x11 bitmaps, '#' = segment.
static const char *const IC_WIFI[11] = {
    "                ", "   ##########   ", " ##          ## ", "#    ######    #", "   ##      ##   ",
    "  #   ####   #  ", "     #    #     ", "       ##       ", "      ####      ", "       ##       ", "                " };
static const char *const IC_PLAY[11] = {
    "   #            ", "   ###          ", "   #####        ", "   #######      ", "   #########    ",
    "   ##########   ", "   #########    ", "   #######      ", "   #####        ", "   ###          ", "   #            " };
static const char *const IC_DISK[11] = {
    " ############   ", " ##  ####  ###  ", " ##  ## #  #### ", " ##  ####  #### ", " ############## ",
    " ############## ", " ##          ## ", " ## ######## ## ", " ##          ## ", " ## ######## ## ", " ############## " };
static const char *const IC_SD[11] = {
    "   ##########   ", "   # # # # ###  ", "   # # # # #### ", "   ########### #", "   #          ##",
    "   #   SD     ##", "   #          ##", "   #          ##", "   #          ##", "   #          ##", "   #############" };
static const char *const IC_WARN[11] = {
    "       ##       ", "      ####      ", "      ####      ", "     ##  ##     ", "     ## ###     ",
    "    ### ####    ", "    ### ####    ", "   ##########   ", "   #### #####   ", "  ############  ", "  ############  " };

static void icon(int x, int y, const char *const bmp[11], bool lit)
{
    uint16_t c = lit ? C_INK : C_GHOST;
    for (int r = 0; r < 11; r++)
        for (int k = 0; k < 16 && bmp[r][k]; k++)
            if (bmp[r][k] == '#') px(x + k, y + r, c);
}

// Charge bolt, printed left of the battery; lit while USB power is in.
static const char *const IC_BOLT[11] = {
    "    ###", "   ### ", "   ##  ", "  ##   ", " ######", " ######",
    "   ##  ", "  ##   ", "  #    ", " #     ", "#      " };

typedef enum { BATT_DISCHARGE, BATT_CHARGING, BATT_FULL, BATT_USB_IDLE } batt_mode_t;

static unsigned s_frame;               // render count, drives the charge animation

static void battery(int x, int y, int soc, batt_mode_t mode)
{
    icon(x - 10, y, IC_BOLT, mode != BATT_DISCHARGE);

    // outline 23x11 + nub (ends at x+24); four bars with a 1 px gap each
    // side, ghosted when empty
    fill(x, y, 23, 1, C_INK); fill(x, y + 10, 23, 1, C_INK);
    fill(x, y, 1, 11, C_INK); fill(x + 22, y, 1, 11, C_INK);
    fill(x + 23, y + 3, 2, 5, C_INK);
    int bars = soc < 0 ? 0 : (soc + 12) / 25;
    if (bars > 4) bars = 4;
    if (mode == BATT_FULL) bars = 4;
    if (mode == BATT_CHARGING) {
        // Fill from the current level up to full, one bar per frame, and
        // repeat. Capped at 3 so a nearly full cell still visibly moves.
        int base = bars > 3 ? 3 : bars;
        bars = base + (int)(s_frame % (unsigned)(5 - base));
    }
    for (int i = 0; i < 4; i++) fill(x + 2 + i * 5, y + 2, 4, 7, i < bars ? C_INK : C_GHOST);
}

// ---------------- QR (via espressif/qrcode) ----------------

#define QR_MAXSZ 41                    // version 6
static struct { int size; uint8_t on[QR_MAXSZ][QR_MAXSZ]; } s_qr;
static char s_qr_text[80];             // regenerate only when this changes

static void qr_capture(esp_qrcode_handle_t qr)
{
    int n = esp_qrcode_get_size(qr);
    if (n > QR_MAXSZ) n = QR_MAXSZ;
    s_qr.size = n;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
            s_qr.on[y][x] = esp_qrcode_get_module(qr, x, y) ? 1 : 0;
}

static void qr_ensure(const char *t)
{
    if (strcmp(t, s_qr_text) == 0 && s_qr.size > 0) return;
    strlcpy(s_qr_text, t, sizeof(s_qr_text));
    s_qr.size = 0;
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func       = qr_capture;
    cfg.max_qrcode_version = 6;
    cfg.qrcode_ecc_level   = ESP_QRCODE_ECC_LOW;
    if (esp_qrcode_generate(&cfg, t) != ESP_OK)
        ESP_LOGW(TAG, "QR generate failed for '%s'", t);
}

// Centred across the content width at the largest whole scale that fits
// `box` px tall. Drawn straight on the LCD field, which is light enough to
// act as the quiet zone. Returns the y past its bottom edge.
static int qr_draw(int y, int box)
{
    int n = s_qr.size;
    if (n <= 0) return y;
    const int quiet = 2;
    int lim = box < FW ? box : FW;
    int scale = lim / (n + quiet * 2);
    if (scale < 1) scale = 1;
    int side = (n + quiet * 2) * scale;
    int ox = FX + (FW - side) / 2;
    for (int r = 0; r < n; r++)
        for (int k = 0; k < n; k++)
            if (s_qr.on[r][k])
                fill(ox + (k + quiet) * scale, y + (r + quiet) * scale, scale, scale, C_INK);
    return y + side;
}

// ---------------- shared chrome: icon row, track map ----------------

static volatile display_mode_t s_mode = DISPLAY_PROVISIONING;
static volatile int            s_files = -1;
static char                    s_note[64];       // result-screen note; byte 63 stays 0
static bool                    s_blink;          // toggles each frame: bad tracks blink
static bool                    s_online;         // station has an address
static TaskHandle_t            s_task;
static int64_t                 s_read_t0, s_read_ms = -1;
static const char *volatile    s_job_verb;       // display_set_job_pos()
static volatile int            s_job_track = -1;
static disk_state_t            s_last_state = DISK_EMPTY;

// DISP steps through the pages; page 0 is the state-driven disk screen. Back
// to it when the disk state changes or PAGE_TIMEOUT_US after the last press.
enum { PAGE_DISK = 0, PAGE_WIFI, PAGE_POWER, PAGE_SD, PAGE_SYSTEM, PAGE_N };
#define PAGE_TIMEOUT_US (120LL * 1000000)
static volatile int  s_page;
static int64_t       s_page_us;
static char          s_saved_dir[16];   // last SAVE folder, for the SD page

static void icon_row(bool online, bool reading, bool loaded, bool warn)
{
    bool sd = sd_present();
    icon(FX,      4, IC_WIFI, online);
    icon(FX + 22, 4, IC_PLAY, reading);
    icon(FX + 44, 4, IC_DISK, loaded);
    icon(FX + 66, 4, IC_SD,   sd);
    icon(FX + 88, 4, IC_WARN, warn);

    power_state_t ps;
    bool got = power_get(&ps);
    bool have = got && ps.gauge_ok;
    batt_mode_t bm = BATT_DISCHARGE;
    if (got && ps.charger_ok && ps.usb_present)
        bm = power_is_charged(&ps)    ? BATT_FULL
           : ps.charging              ? BATT_CHARGING
                                      : BATT_USB_IDLE;   // e.g. too hot/cold to charge
    battery(FR - 24, 4, have ? ps.soc : -1, bm);

    int low = power_low_level();
    if (low) {
        // Battery warning replaces the SD line on every screen; critical blinks.
        char w[24];
        snprintf(w, sizeof w, low == 2 ? "BATTERY CRITICAL %d%%" : "LOW BATTERY %d%%",
                 have ? ps.soc : 0);
        if (low == 1 || s_blink) {
            fill(FX, 17, FW, 12, C_INK);
            text_c(19, w, 1, 1, C_LCD);
        } else {
            text_c(19, w, 1, 1, C_INK);
        }
        icon(FX + 88, 4, IC_WARN, true);
        fill(FX, 30, FW, 1, C_GHOST);
        return;
    }

    char b[16];
    uint64_t total, freeb;
    if (sd && sd_usage(&total, &freeb))
        snprintf(b, sizeof(b), "SD %.1fG FREE", (double)freeb / 1073741824.0);
    else
        snprintf(b, sizeof(b), sd ? "SD OK" : "NO SD CARD");
    text(FR - text_w(b, 1), 19, b, 1, 1, sd ? C_DIM : C_INK);
    fill(FX, 30, FW, 1, C_GHOST);
}

// The 80 cylinders as 4 rows of 20; each cell is head 0 over head 1.
// Retried = lower half only; bad = whole cell, blinking.
#define TMAP_COLS   20
#define TMAP_PITCH  7                  // 6 px cell + 1 gap -> 140 px
#define TMAP_ROW    19                 // two 7 px heads + gaps
#define TMAP_H      (DISK_CYLS / TMAP_COLS * TMAP_ROW)

static void track_map(int y)
{
    const uint8_t *m = disk_trackmap();
    int x0 = FX + (FW - TMAP_COLS * TMAP_PITCH) / 2;
    text(x0, y, "TRACKS", 1, 1, C_DIM);
    text(x0 + TMAP_COLS * TMAP_PITCH - 1 - text_w("CYL 0-79", 1), y, "CYL 0-79", 1, 1, C_DIM);
    y += 12;
    for (int c = 0; c < DISK_CYLS; c++) {
        int x = x0 + (c % TMAP_COLS) * TMAP_PITCH, ry = y + (c / TMAP_COLS) * TMAP_ROW;
        for (int h = 0; h < DISK_HEADS; h++) {
            int hy = ry + h * 8;
            switch (m[c * DISK_HEADS + h] & 3) {
            case TRK_GOOD:  fill(x, hy, 6, 7, C_INK); break;
            case TRK_RETRY: fill(x, hy, 6, 7, C_GHOST); fill(x, hy + 4, 6, 3, C_INK); break;
            case TRK_BAD:   fill(x - 1, hy - 1, 8, 9, s_blink ? C_INK : C_GHOST); break;
            default:        fill(x, hy, 6, 7, C_GHOST); break;
            }
        }
    }
}

static void begin(void)
{
    fill(0, 0, LCD_W, LCD_H, C_LCD);
}

// ---------------- screens ----------------

static void screen_provisioning(void)
{
    qr_ensure("WIFI:T:nopass;S:" PROVISION_AP_SSID ";;");
    begin();
    icon_row(false, false, false, false);
    text_c(38, "WI-FI SETUP", 1, 2, C_INK);
    int y = qr_draw(60, 140) + 6;
    text_c(y,      "1 SCAN THE CODE", 1, 1, C_INK);
    text_c(y + 14, "2 JOIN", 1, 1, C_INK);
    text_c(y + 26, PROVISION_AP_SSID, 1, 2, C_INK);
    text_c(y + 50, "3 PICK YOUR", 1, 1, C_INK);
    text_c(y + 62, "NETWORK", 1, 1, C_INK);
    text_c(y + 82, "OPEN, NO PASSWORD", 1, 1, C_DIM);
}

// `loaded`: a decoded disk is behind this screen.
static void screen_idle(const char *ip, bool loaded)
{
    char url[40];
    snprintf(url, sizeof(url), "http://%s/", ip);
    qr_ensure(url);
    begin();
    icon_row(true, false, loaded, false);
    int x = cx(digits_w(26, "--") + 6 + text_w("READY", 2));
    x = digits(x, 40, 26, "--", C_INK);
    text(x + 6, 45, "READY", 2, 2, C_INK);
    int y = qr_draw(74, 130) + 6;
    text_c(y,      MDNS_HOSTNAME ".local", 1, 2, C_INK);
    text_c(y + 22, ip, 1, 2, C_INK);
    if (loaded) {
        text_c(y + 50, "DISK IN MEMORY", 1, 1, C_INK);
        text_c(y + 62, "DISP: BACK TO DISK", 1, 1, C_DIM);
    } else {
        text_c(y + 50, "INSERT DISK", 1, 1, C_INK);
        text_c(y + 62, "PRESS READ", 1, 1, C_DIM);
    }
}

static void screen_connecting(void)
{
    begin();
    icon_row(false, false, false, false);
    digits(cx(digits_w(80, "--")), 60, 80, "--", C_INK);
    text_c(170, "WI-FI", 2, 2, C_INK);
    text_c(196, "CONNECTING", 1, 2, C_INK);
    text_c(226, "THE WEB PAGE OPENS", 1, 1, C_DIM);
    text_c(238, "WHEN THIS IS DONE", 1, 1, C_DIM);
}

// m:ss
static void fmt_elapsed(char *b, size_t n, int64_t ms)
{
    if (ms < 0) { snprintf(b, n, "-:--"); return; }
    int s = (int)(ms / 1000);
    if (s > 99 * 60 + 59) s = 99 * 60 + 59;
    snprintf(b, n, "%d:%02d", s / 60, s % 60);
}

// "TIME m:ss", centred as one group; y is the top of the digits.
static void time_row(int y, int h, const char *t)
{
    int x = cx(40 + digits_w(h, t));
    text(x, y + 6, "TIME", 1, 1, C_DIM);
    digits(x + 40, y, h, t, C_INK);
}

static void screen_reading(void)
{
    uint32_t done = disk_sectors_done(), tot = disk_sectors_total();
    int pct = tot ? (int)((uint64_t)done * 100 / tot) : 0;
    if (pct > 99) pct = 99;
    begin();
    icon_row(s_online, true, false, false);

    char b[24];
    snprintf(b, sizeof(b), "%2d", pct);
    int x = cx(digits_w(66, b) - 2 + text_w("%", 2));
    x = digits(x, 40, 66, b, C_INK);
    text(x - 2, 90, "%", 2, 2, C_INK);

    const int n = 20;
    int lit = pct * n / 100, bx = cx(n * 7 - 2);
    for (int i = 0; i < n; i++) {
        int h = 5 + i * 10 / (n - 1);
        fill(bx + i * 7, 132 - h, 5, h, i < lit ? C_INK : C_GHOST);
    }
    snprintf(b, sizeof(b), "%u/%u SEC", (unsigned)done, (unsigned)tot);
    text_c(140, b, 1, 2, C_INK);

    // Furthest track touched so far.
    const uint8_t *m = disk_trackmap();
    int last = -1;
    for (int i = 0; i < TOTAL_TRACKS; i++) if (m[i] & 3) last = i;
    // The same screen serves a retry of the bad tracks, a disk write and the
    // disk test (which names its pass and track itself).
    int op = ui_running_op();
    const char *verb = op == UI_OP_WRITE_DISK ? "WRITE" : op == UI_OP_RETRY ? "RETRY" : "CYL";
    const char *jv = s_job_verb;
    if (jv && s_job_track >= 0) { verb = jv; last = s_job_track; }
    if (last >= 0) snprintf(b, sizeof(b), "%s %02d HD%d", verb, last / DISK_HEADS, last % DISK_HEADS);
    else           snprintf(b, sizeof(b), "SEEKING");
    text_c(162, b, 1, 2, C_INK);

    fmt_elapsed(b, sizeof(b), esp_timer_get_time() / 1000 - s_read_t0);
    time_row(186, 20, b);

    track_map(LCD_H - TMAP_H - 18);
}

static void screen_result(bool dirty)
{
    begin();
    icon_row(s_online, false, true, dirty);

    char b[32];
    // Three digits only fit beside the label at a smaller size.
    int files = s_files, dh = files > 99 ? 48 : 66;
    if (files >= 0) snprintf(b, sizeof(b), "%2d", files > 999 ? 999 : files);
    else            snprintf(b, sizeof(b), "--");
    int x = cx(digits_w(dh, b) + 2 + text_w("FILES", 1));
    x = digits(x, 40 + 66 - dh, dh, b, C_INK);
    text(x + 2, 90, "FILES", 1, 2, C_INK);

    uint32_t bad = disk_bad_sector_count();
    if (dirty) {
        int bw = text_w("PARTIAL", 2) + 8, bx = cx(bw);
        fill(bx, 116, bw, 20, C_INK);
        text(bx + 4, 118, "PARTIAL", 2, 2, C_LCD);
        snprintf(b, sizeof(b), "%u BAD SEC", (unsigned)bad);
        text_c(142, b, 1, 2, C_INK);
    } else {
        text_c(118, "ALL CLEAN", 2, 2, C_INK);
        text_c(142, "NO BAD SECTORS", 1, 2, C_INK);
    }

    char t[8];
    fmt_elapsed(t, sizeof(t), s_read_ms);
    time_row(166, 18, t);

    const char *hint = s_note[0] ? s_note
                     : dirty     ? "HOLD READ TO RETRY THE BAD TRACKS"
                                 : "PRESS SAVE TO ARCHIVE";
    text_wrap(192, FW / 8, hint, 1, 1, 3, C_INK);

    track_map(LCD_H - TMAP_H - 18);
}

static void screen_error(void)
{
    begin();
    icon_row(s_online, false, false, true);
    // Warning triangle (~44 px wide, centred on +20) beside the title,
    // centred as one group.
    int x = cx(4 + 52 + text_w("READ", 2)) + 4;   // triangle starts at x-4
    for (int r = 0; r < 40; r++) fill(x + 20 - r * 6 / 10, 40 + r, 1 + r * 12 / 10, 1, C_INK);
    fill(x + 18, 52, 5, 16, C_LCD);
    fill(x + 18, 72, 5, 5, C_LCD);
    // A failed disk write says so in its detail line (app_main.c).
    const char *d = disk_get_detail();
    bool test = d && (strncmp(d, "test", 4) == 0 || strncmp(d, "quick test", 10) == 0);
    bool hlth = d && strncmp(d, "health", 6) == 0;
    bool wr = test || (d && strncmp(d, "write", 5) == 0);
    text(x + 52, 44, test ? "TEST" : hlth ? "CHECK" : wr ? "WRITE" : "READ", 2, 2, C_INK);
    text(x + 52, 64, "FAILED", 1, 2, C_INK);
    if (d && d[0]) text_wrap(100, FW / 8, d, 1, 2, 5, C_INK);
    text_c(220, "CHECK THE DISK IS", 1, 1, C_INK);
    text_c(232, wr ? "SEATED AND NOT" : "SEATED, THEN", 1, 1, C_INK);
    text_c(244, wr ? "WRITE-PROTECTED" : "PRESS READ", 1, 1, wr ? C_INK : C_DIM);
}

// Firmware update in progress. Neither kind can brick the board (the ESP32
// keeps its old slot; the controller can be flashed again), but switching
// off mid-way wastes the update, and a half-written controller won't read
// disks until it is redone.
static void screen_update(const char *what, int pct, bool restarting)
{
    begin();
    icon_row(s_online, false, false, false);
    text_c(38, "UPDATING", 2, 2, C_INK);

    char b[16];
    snprintf(b, sizeof(b), "%2d", pct > 99 ? 99 : pct);
    int x = cx(digits_w(66, b) - 2 + text_w("%", 2));
    x = digits(x, 66, 66, b, C_INK);
    text(x - 2, 116, "%", 2, 2, C_INK);

    const int n = 20;
    int lit = pct * n / 100, bx = cx(n * 7 - 2);
    for (int i = 0; i < n; i++)
        fill(bx + i * 7, 146, 5, 10, i < lit ? C_INK : C_GHOST);

    text_c(168, what, 1, 2, C_INK);
    text_c(188, restarting ? "RESTARTING" : "FIRMWARE", 1, 2, C_INK);

    fill(FX, 222, FW, 50, C_INK);
    text_c(228, "DON'T", 2, 2, C_LCD);
    text_c(250, "SWITCH OFF", 1, 2, C_LCD);
}

// ---------------- DISP pages ----------------

static void upcase(char *s) { for (; *s; s++) *s = (char)toupper((unsigned char)*s); }

// Which page this is, as dots along the bottom edge.
static void page_dots(int page)
{
    int x = cx(PAGE_N * 12 - 6);
    for (int i = 0; i < PAGE_N; i++)
        fill(x + i * 12, LCD_H - 10, 6, 6, i == page ? C_INK : C_GHOST);
}

// "LABEL      VALUE" on one 16 px row, value right-aligned; a value too long
// for the row drops to the next one. Returns the y for the next row.
static int kv(int y, const char *label, const char *value)
{
    char v[24];
    snprintf(v, sizeof v, "%s", value);
    upcase(v);
    text(FX, y, label, 1, 2, C_DIM);
    if (text_w(label, 1) + 8 + text_w(v, 1) > FW) y += 17;
    text(FR - text_w(v, 1), y, v, 1, 2, C_INK);
    return y + 19;
}

static void fmt_hours(char *b, size_t n, float h)
{
    if (!(h > 0) || h > 999) { snprintf(b, n, "--"); return; }
    int m = (int)(h * 60 + 0.5f);
    if (m < 60)       snprintf(b, n, "%dM", m < 1 ? 1 : m);
    else if (h >= 10) snprintf(b, n, "%dH", (int)(h + 0.5f));
    else              snprintf(b, n, "%dH %02dM", m / 60, m % 60);
}

// A 20-segment meter, 0..100.
static void meter(int y, int pct)
{
    const int n = 20;
    int lit = (pct * n + 50) / 100, bx = cx(n * 7 - 2);
    for (int i = 0; i < n; i++) fill(bx + i * 7, y, 5, 10, i < lit ? C_INK : C_GHOST);
}

static void screen_wifi(const char *ip, bool online)
{
    begin();
    icon_row(online, false, false, false);
    text_c(38, "WI-FI", 2, 2, C_INK);
    int y = 60;
    char b[40];
    if (online) {
        snprintf(b, sizeof b, "http://%s/", ip);
        qr_ensure(b);
        y = qr_draw(y, 104) + 2;
        text_c(y, ip, 1, 2, C_INK);
        y += 22;
        char ssid[33] = "";
        int rssi = 0, ch = 0;
        if (wifi_pwr_ap(ssid, sizeof ssid, &rssi, &ch)) {
            ssid[19] = '\0';
            upcase(ssid);
            text_c(y, ssid, 1, 2, C_INK);
            y += 22;
            // Signal: four rising bars beside the dBm figure.
            int bars = rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : rssi > -85 ? 1 : 0;
            snprintf(b, sizeof b, "%d DBM CH%d", rssi, ch);
            int w = 4 * 6 + 6 + text_w(b, 1), x = cx(w);
            for (int i = 0; i < 4; i++)
                fill(x + i * 6, y + 12 - (i + 1) * 3, 4, (i + 1) * 3, i < bars ? C_INK : C_GHOST);
            text(x + 30, y + 4, b, 1, 1, C_INK);
            y += 20;
        }
    } else {
        text_c(y + 30, "CONNECTING", 1, 2, C_INK);
        text_c(y + 56, "OR NO NETWORK", 1, 1, C_DIM);
        y += 120;
    }
    text_c(LCD_H - 40, "HOLD SAVE+DISP 5S", 1, 1, C_DIM);
    text_c(LCD_H - 28, "TO FORGET NETWORK", 1, 1, C_DIM);
    page_dots(PAGE_WIFI);
}

static void screen_power(void)
{
    begin();
    icon_row(s_online, false, false, false);
    text_c(38, "BATTERY", 2, 2, C_INK);
    power_state_t ps;
    if (!power_get(&ps) || !ps.gauge_ok) {
        digits(cx(digits_w(66, "--")), 66, 66, "--", C_INK);
        text_c(150, "NO GAUGE", 1, 2, C_INK);
        page_dots(PAGE_POWER);
        return;
    }
    char b[32], t[16];
    int soc = ps.soc < 0 ? 0 : ps.soc > 99 ? 99 : ps.soc;
    snprintf(b, sizeof b, "%2d", soc);
    int x = cx(digits_w(66, b) - 2 + text_w("%", 2));
    x = digits(x, 60, 66, b, C_INK);
    text(x - 2, 110, "%", 2, 2, C_INK);
    meter(134, ps.soc < 0 ? 0 : ps.soc);
    snprintf(b, sizeof b, "%u.%02u V", ps.mv / 1000, (ps.mv % 1000) / 10);
    text_c(152, b, 1, 2, C_INK);

    // Status and time: to full while charging, to empty on battery.
    float rate = ps.crate_valid ? ps.crate_cpph / 100.0f : 0;
    const char *st;
    if (ps.usb_present)
        st = power_is_charged(&ps)        ? "CHARGED"
           : ps.charging                  ? "CHARGING"
           : ps.tmr_fault                 ? "CHG TIMEOUT"
           : ps.ts_zone == TS_SUSPENDED   ? "TOO HOT/COLD"
           : ps.chg_disabled              ? "CHG PAUSED"
                                          : "ON USB";
    else
        st = "ON BATTERY";
    text_c(176, st, 1, 2, C_INK);
    b[0] = '\0';
    if (ps.charging && rate > 0.1f) {
        fmt_hours(t, sizeof t, (100 - ps.soc) / rate);
        snprintf(b, sizeof b, "FULL IN %s", t);
    } else if (!ps.usb_present && rate < -0.1f && !ps.gauge_reset) {
        fmt_hours(t, sizeof t, ps.soc / -rate);
        snprintf(b, sizeof b, "%s LEFT", t);
    } else if (!ps.usb_present) {
        snprintf(b, sizeof b, ps.gauge_reset ? "GAUGE SETTLING" : "TIME LEFT --");
    }
    if (b[0]) text_c(198, b, 1, 2, C_INK);

    int y = 226;
    if (ps.charging) {
        snprintf(b, sizeof b, "%u MA", ps.ichg_ma);
        y = kv(y, "CHARGE", b);
    }
    temps_t tp;
    temps_get(&tp);
    if (tp.batt_zone_valid)
        y = kv(y, "BATT TEMP", tp.batt_ts_open ? "NO SENSOR" : power_ts_zone_str(tp.batt_zone));
    int left = ui_autooff_left_s();
    if (left >= 0) {
        snprintf(b, sizeof b, "%d:%02d", left / 60, left % 60);
        y = kv(y, "AUTO-OFF", b);
    }
    page_dots(PAGE_POWER);
}

static void screen_sd(void)
{
    begin();
    icon_row(s_online, false, false, false);
    text_c(38, "SD CARD", 2, 2, C_INK);
    uint64_t total = 0, freeb = 0;
    if (!sd_present() || !sd_usage(&total, &freeb) || !total) {
        text_c(90, sd_present() ? "CARD ERROR" : "NO CARD", 2, 2, C_INK);
        text_c(124, "INSERT A MICROSD", 1, 1, C_DIM);
        text_c(136, "CARD TO ARCHIVE", 1, 1, C_DIM);
        page_dots(PAGE_SD);
        return;
    }
    char b[32];
    double gb = (double)freeb / 1073741824.0;
    bool in_gb = gb >= 1;
    snprintf(b, sizeof b, "%2d", in_gb ? (gb > 99 ? 99 : (int)(gb + 0.5))
                                       : (int)((double)freeb / 1048576.0) % 1000);
    int x = cx(digits_w(66, b) + 4 + text_w("FREE", 1));
    x = digits(x, 60, 66, b, C_INK);
    text(x + 4, 92, in_gb ? "GB" : "MB", 1, 2, C_INK);
    text(x + 4, 112, "FREE", 1, 1, C_INK);
    int used_pct = (int)((total - freeb) * 100 / total);
    meter(134, used_pct);
    double used_mb = (double)(total - freeb) / 1048576.0;
    if (used_mb < 1000) snprintf(b, sizeof b, "%d MB OF %.0f GB", (int)used_mb, total / 1073741824.0);
    else                snprintf(b, sizeof b, "%.1f OF %.0f GB", used_mb / 1024.0, total / 1073741824.0);
    text_c(152, b, 1, 2, C_INK);

    int y = 182;
    bool idx = sd_index_is_loaded();
    snprintf(b, sizeof b, "%d", sd_index_disk_count());
    y = kv(y, "DISKS SAVED", idx ? b : "--");
    snprintf(b, sizeof b, "%d", sd_index_count());
    y = kv(y, "FILES", idx ? b : "--");
    y = kv(y, "LAST SAVE", s_saved_dir[0] ? s_saved_dir : "--");
    text_c(LCD_H - 28, "SAVE: ARCHIVE DISK", 1, 1, C_DIM);
    page_dots(PAGE_SD);
}

static void screen_system(void)
{
    begin();
    icon_row(s_online, false, false, false);
    text_c(38, "SYSTEM", 2, 2, C_INK);
    const esp_app_desc_t *app = esp_app_get_description();
    char b[32];
    int y = 60;
    snprintf(b, sizeof b, "%.31s", app->version);
    char *dirty = strstr(b, "-dirty");      // uncommitted build: hash + "+"
    if (dirty) strcpy(dirty, "+");
    b[11] = 0;
    y = kv(y, "FIRMWARE", b);
    snprintf(b, sizeof b, "%.6s %.5s", app->date, app->time);
    y = kv(y, "BUILT", b);
    y = kv(y, "SLOT", ota_running_label());
    y = kv(y, "SERIAL", serialno_get());
    int64_t up = esp_timer_get_time() / 1000000;
    snprintf(b, sizeof b, "%d:%02d:%02d", (int)(up / 3600), (int)(up / 60 % 60), (int)(up % 60));
    y = kv(y, "UPTIME", b);
    time_t now = time(NULL);
    if (now > 1600000000) {
        struct tm tm;
        gmtime_r(&now, &tm);
        snprintf(b, sizeof b, "%02d:%02d UTC", tm.tm_hour, tm.tm_min);
    } else {
        snprintf(b, sizeof b, "NOT SET");
    }
    y = kv(y, "CLOCK", b);
    temps_t tp;
    temps_get(&tp);
    if (tp.esp_valid && tp.rp_valid) snprintf(b, sizeof b, "%dC/%dC", (int)tp.esp_c, (int)tp.rp_c);
    else if (tp.esp_valid)           snprintf(b, sizeof b, "%dC/--", (int)tp.esp_c);
    else                             snprintf(b, sizeof b, "--");
    y = kv(y, "ESP/RP", b);
    text(FX, y, "LAST OFF", 1, 2, C_DIM);
    y += 17;
    text(FR - text_w(lastoff_prev_short(), 1), y, lastoff_prev_short(), 1, 2, C_INK);
    page_dots(PAGE_SYSTEM);
}

// SAVE + DISP held: WiFi reset countdown. Takes the screen over.
static void screen_chord(int left)
{
    begin();
    icon_row(s_online, false, false, true);
    text_c(44, "WI-FI", 2, 2, C_INK);
    text_c(66, "RESET", 2, 2, C_INK);
    if (ui_chord_fired()) {
        fill(FX, 120, FW, 60, C_INK);
        text_c(128, "CLEARED", 2, 2, C_LCD);
        text_c(154, "RESTARTING", 1, 2, C_LCD);
        text_c(206, "THEN JOIN", 1, 1, C_INK);
        text_c(218, PROVISION_AP_SSID, 1, 2, C_INK);
        text_c(240, "TO SET IT UP AGAIN", 1, 1, C_INK);
        return;
    }
    char b[12];
    snprintf(b, sizeof b, "%d", left < 0 ? 0 : left > 9 ? 9 : left);
    digits(cx(digits_w(90, b)), 104, 90, b, C_INK);
    text_c(214, "KEEP HOLDING", 1, 2, C_INK);
    text_c(236, "SAVE + DISP", 1, 2, C_INK);
    text_c(270, "LET GO TO CANCEL", 1, 1, C_DIM);
}

// Idle on battery: the last AUTO_OFF_WARN_S before switching off.
static void screen_autooff(int left)
{
    begin();
    icon_row(s_online, false, false, true);
    text_c(44, "POWER OFF", 2, 2, C_INK);
    char b[12];
    snprintf(b, sizeof b, "%2d", left > 99 ? 99 : left);
    digits(cx(digits_w(90, b)), 84, 90, b, C_INK);
    text_c(190, "IDLE ON BATTERY", 1, 2, C_INK);
    fill(FX, 224, FW, 44, C_INK);
    text_c(230, "PRESS ANY BUTTON", 1, 2, C_LCD);
    text_c(250, "TO STAY ON", 1, 2, C_LCD);
}

// What is being updated, or NULL. ESP32 first: its 100 % stays up until the
// reboot.
static const char *update_state(int *pct)
{
    int p = ota_progress_pct();
    if (p >= 0) { *pct = p; return "ESP32"; }
    p = rp_flash_progress_pct();
    if (p >= 0) { *pct = p; return "DRIVE CONTROLLER"; }
    return NULL;
}

// Read timing and the home-screen latch. Runs every tick, also while the
// panel sleeps, so a read done with the LCD off is still timed.
static disk_state_t track_state(void)
{
    disk_state_t st = disk_get_state();
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (st == DISK_READING && s_last_state != DISK_READING) s_read_t0 = now_ms;
    if (st != DISK_READING && s_last_state == DISK_READING) s_read_ms = now_ms - s_read_t0;
    if (st == DISK_EMPTY) s_read_ms = -1;
    if (st != s_last_state) s_page = PAGE_DISK;
    if (s_page != PAGE_DISK && esp_timer_get_time() - s_page_us > PAGE_TIMEOUT_US)
        s_page = PAGE_DISK;
    s_last_state = st;
    return st;
}

static void render(disk_state_t st)
{
    s_blink = !s_blink;
    s_frame++;
    int upct;
    const char *upd = update_state(&upct);
    if (upd) {
        screen_update(upd, upct, upd[0] == 'E' && upct >= 100);
        lcd_flush();
        return;
    }
    char ip[16];
    net_sta_ip_str(ip, sizeof(ip));
    bool online = s_online = strcmp(ip, "0.0.0.0") != 0;

    int left = ui_chord_secs_left();
    if (left >= 0) { screen_chord(left); lcd_flush(); return; }
    left = ui_autooff_secs_left();
    if (left >= 0) { screen_autooff(left); lcd_flush(); return; }

    if (s_mode == DISPLAY_PROVISIONING) { screen_provisioning(); lcd_flush(); return; }

    switch (s_page) {
    case PAGE_WIFI:   screen_wifi(ip, online); lcd_flush(); return;
    case PAGE_POWER:  screen_power();          lcd_flush(); return;
    case PAGE_SD:     screen_sd();             lcd_flush(); return;
    case PAGE_SYSTEM: screen_system();         lcd_flush(); return;
    default: break;
    }

    switch (st) {
    case DISK_READING:     screen_reading();             break;
    case DISK_READY:       screen_result(false);         break;
    case DISK_READY_DIRTY: screen_result(true);          break;
    case DISK_ERROR:       screen_error();               break;
    case DISK_EMPTY:
    default:
        if (online) screen_idle(ip, false);
        else        screen_connecting();
        break;
    }
    lcd_flush();
}

// Panel sleep, requested from other tasks and applied by display_task, which
// owns the SPI device.
static volatile bool s_power = true;
static volatile bool s_power_dirty = false;

static void display_task(void *arg)
{
    (void)arg;
    for (;;) {
        // An update shows even if the panel was put to sleep.
        int upct;
        bool updating = update_state(&upct) != NULL;
        // Updates and the WiFi-reset / auto-off countdowns show even if the
        // panel was put to sleep.
        bool overlay = ui_chord_secs_left() >= 0 || ui_autooff_secs_left() >= 0;
        if ((updating || overlay) && !s_power) { s_power = true; s_power_dirty = true; }
        if (s_power_dirty) {
            s_power_dirty = false;
            if (s_power) {
                lcd_cmd(0x11, NULL, 0);            // SLPOUT
                vTaskDelay(pdMS_TO_TICKS(120));
                lcd_cmd(0x29, NULL, 0);            // DISPON
                s_last_sum = 0;                    // force a full frame
            } else {
                lcd_cmd(0x28, NULL, 0);            // DISPOFF
                lcd_cmd(0x10, NULL, 0);            // SLPIN
            }
        }
        disk_state_t st = track_state();
        if (s_power && s_ok) render(st);
        // A button press (home / power) redraws at once. Faster during an
        // update: an ESP32 one only lasts a few seconds.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(updating ? 150 : overlay ? 250 : 400));
    }
}

// ---------------- public API ----------------

void display_init(void)
{
    if (!lcd_hw_init()) {
        ESP_LOGW(TAG, "LCD not initialised — status display disabled");
    }
    xTaskCreatePinnedToCore(display_task, "lcd", 4096, NULL, 2, &s_task, 0);
    ESP_LOGI(TAG, "display task started (panel %s)", s_ok ? "up" : "absent");
}

void display_set_mode(display_mode_t m) { s_mode = m; }
void display_set_files(int n)           { s_files = n; }

void display_set_job_pos(const char *verb, int track)
{
    s_job_track = track;
    s_job_verb  = verb;
}

void display_note(const char *text)
{
    snprintf(s_note, sizeof(s_note), "%s", text ? text : "");
}

void display_note_saved(const char *name)
{
    if (!name || !name[0]) { s_note[0] = '\0'; return; }
    snprintf(s_note, sizeof(s_note), "SAVED TO %.12s", name);
    snprintf(s_saved_dir, sizeof(s_saved_dir), "%.12s", name);
}

static void wake(void) { if (s_task) xTaskNotifyGive(s_task); }

void display_next_page(void)
{
    s_page = (s_page + 1) % PAGE_N;
    s_page_us = esp_timer_get_time();
    wake();
}
void display_set_power(bool on)    { s_power = on; s_power_dirty = true; wake(); }
void display_toggle_power(void)    { display_set_power(!s_power); }
