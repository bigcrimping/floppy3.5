#include "ui.h"
#include "app_config.h"
#include "disk_image.h"
#include "display.h"
#include "leds.h"
#include "power.h"
#include "storage.h"
#include "lastoff.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "op_queue.h"
#include "esp_log.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "ota.h"
#include "rp_flash.h"
#include "gw_client.h"
#include "provision.h"

static const char *TAG = "ui";
// Ops waiting for the gw task (op_queue.h), under s_ops_mux; s_wake wakes it.
static opq_t             s_ops;
static SemaphoreHandle_t s_wake;
static portMUX_TYPE      s_ops_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int      s_running = -1;
static volatile bool     s_cancel;         // ui_cancel(): stop the running op

// Active-low buttons. Short actions fire on release (so a long press doesn't
// also fire one); long actions fire as soon as the hold threshold passes.
enum { BTN_LOAD = 0, BTN_SAVEALL, BTN_DISPLAY, BTN_N };
#define POLL_MS 30
#define DEBOUNCE_TICKS 3                       // ~90 ms
#define LONG_TICKS (1500 / POLL_MS)            // 1.5 s hold = long press

static struct {
    int gpio;
    int stable;     // consecutive polls seen low
    int pressed;    // debounced press latched
    int longfired;  // long action already fired for this press
    int swallow;    // this press does nothing (chord / auto-off cancel)
} s_btn[BTN_N] = {
    { UI_BUTTON_GPIO,      0, 0, 0, 0 },
    { UI_BTN_SAVEALL_GPIO, 0, 0, 0, 0 },
    { UI_BTN_DISPLAY_GPIO, 0, 0, 0, 0 },
};

// SAVE + DISP chord: WiFi reset. s_chord_t0 = 0 when not held.
static int64_t s_chord_t0;
static volatile int s_chord_left = -1;      // seconds, for the LCD
static volatile bool s_chord_done;

// Idle auto-off (on battery only).
static int64_t s_activity_us;
static bool    s_autooff_en = true;      // runtime: off during the setup portal
static bool    s_autooff_user = true;    // the user's setting, NVS "ui"/"autooff"
static volatile int s_autooff_left = -1;    // seconds, only in the warning window
static volatile int s_autooff_total = -1;   // seconds until off, -1 = not running

void ui_activity(void) { s_activity_us = esp_timer_get_time(); }
void ui_set_autooff_enabled(bool en) { s_autooff_en = en; ui_activity(); }

#define UI_NVS_NS "ui"
static void autooff_user_load(void)
{
    nvs_handle_t h;
    uint8_t v = 1;
    if (nvs_open(UI_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;   // never set: on
    if (nvs_get_u8(h, "autooff", &v) == ESP_OK) s_autooff_user = v != 0;
    nvs_close(h);
}

void ui_set_autooff_user(bool on)
{
    s_autooff_user = on;
    ui_activity();                      // the 5 minutes restart if it's back on
    nvs_handle_t h;
    if (nvs_open(UI_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "autooff", on ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "auto-off %s", on ? "enabled" : "disabled");
}
bool ui_autooff_user(void) { return s_autooff_user; }
int  ui_chord_secs_left(void)   { return s_chord_done ? 0 : s_chord_left; }
bool ui_chord_fired(void)       { return s_chord_done; }
int  ui_autooff_secs_left(void) { return s_autooff_left; }
int  ui_autooff_left_s(void)    { return s_autooff_total; }

// Anything that should hold the power on regardless of buttons.
static bool busy(void)
{
    return disk_get_state() == DISK_READING || ui_running_op() >= 0 ||
           ui_queued_mask() != 0 || ota_in_progress() ||
           rp_flash_progress_pct() >= 0 || gw_usb_active() ||
           power_boost_hold_left() > 0;
}

static void autooff_poll(void)
{
    int64_t now = esp_timer_get_time();
    power_state_t ps;
    bool on_batt = power_get(&ps) && !ps.usb_present;
    if (!s_autooff_en || !s_autooff_user || !on_batt || busy() || !s_activity_us) {
        s_activity_us = now;                // the 5 minutes start from here
        s_autooff_left = -1;
        s_autooff_total = -1;
        return;
    }
    int idle_s = (int)((now - s_activity_us) / 1000000);
    int left = AUTO_OFF_IDLE_S - idle_s;
    if (left <= 0) {
        ESP_LOGW(TAG, "idle %d s on battery — auto power-off", idle_s);
        lastoff_set_auto();
        power_kill();
        return;
    }
    s_autooff_total = left;
    s_autooff_left = left <= AUTO_OFF_WARN_S ? left : -1;
}

// Both SAVE and DISP held: count down, then forget WiFi and reboot to setup.
static void chord_poll(void)
{
    bool both = s_btn[BTN_SAVEALL].pressed && s_btn[BTN_DISPLAY].pressed;
    int64_t now = esp_timer_get_time();
    if (!both) { s_chord_t0 = 0; s_chord_left = -1; return; }
    // Never in the middle of writing a disk.
    int op = ui_running_op();
    if (op == UI_OP_WRITE_DISK || op == UI_OP_COMMIT || op == UI_OP_WTEST ||
        op == UI_OP_TEST || op == UI_OP_TEST_QUICK) return;
    s_btn[BTN_SAVEALL].swallow = s_btn[BTN_DISPLAY].swallow = 1;
    if (!s_chord_t0) {
        s_chord_t0 = now;
        ESP_LOGI(TAG, "SAVE+DISP held: WiFi reset in %d ms", WIFI_RESET_HOLD_MS);
    }
    int64_t held_ms = (now - s_chord_t0) / 1000;
    s_chord_left = (int)((WIFI_RESET_HOLD_MS - held_ms + 999) / 1000);
    if (held_ms >= WIFI_RESET_HOLD_MS && !s_chord_done) {
        s_chord_done = true;
        ESP_LOGW(TAG, "WiFi credentials cleared by SAVE+DISP — rebooting into setup");
        provision_clear_creds();
        vTaskDelay(pdMS_TO_TICKS(2000));    // LCD shows "WI-FI CLEARED"
        esp_restart();
    }
}

static void on_short(int i)
{
    switch (i) {
    case BTN_LOAD:
        ui_trigger();
        ESP_LOGI(TAG, "button: read disc");
        break;
    case BTN_SAVEALL:
        ui_trigger_op(UI_OP_SAVEALL);
        ESP_LOGI(TAG, "button: save to SD");
        break;
    case BTN_DISPLAY:
        // Next LCD page, plus the battery gauge on the LEDs.
        display_next_page();
        leds_show_battery();
        ESP_LOGI(TAG, "button: home / battery level");
        break;
    }
}

static void on_long(int i)
{
    switch (i) {
    case BTN_LOAD:
        // A partial read: go back for the bad tracks only. Otherwise the
        // same as a short press.
        if (disk_get_state() == DISK_READY_DIRTY) {
            ui_trigger_op(UI_OP_RETRY);
            ESP_LOGI(TAG, "button: retry bad tracks");
        } else {
            ui_trigger();
            ESP_LOGI(TAG, "button: read disc");
        }
        break;
    case BTN_SAVEALL:
        ui_trigger_op(UI_OP_SAVEALL_FORCE);
        ESP_LOGI(TAG, "button: save to SD (forced)");
        break;
    case BTN_DISPLAY:
        // LCD and LEDs off.
        display_toggle_power();
        leds_set_enabled(!leds_enabled());
        ESP_LOGI(TAG, "button: display power");
        break;
    }
}

// Logs state changes; the LEDs themselves are driven by leds_set_state().
static void indicate(disk_state_t st)
{
    static disk_state_t last = -1;
    if (st == last) return;
    last = st;
    switch (st) {
    case DISK_EMPTY:       ESP_LOGI(TAG, "LED: idle (white)"); break;
    case DISK_READING:     ESP_LOGI(TAG, "LED: reading (pulsing blue)"); break;
    case DISK_READY:       ESP_LOGI(TAG, "LED: done (green)"); break;
    case DISK_READY_DIRTY: ESP_LOGI(TAG, "LED: done w/ bad sectors (amber)"); break;
    case DISK_ERROR:       ESP_LOGI(TAG, "LED: error (red blink)"); break;
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    int off_stable = 0;
    while (1) {
        for (int i = 0; i < BTN_N; i++) {
            if (gpio_get_level(s_btn[i].gpio) == 0) {
                if (s_btn[i].stable < 1000) s_btn[i].stable++;
                if (s_btn[i].stable == DEBOUNCE_TICKS) {
                    s_btn[i].pressed = 1;
                    // A press during the auto-off countdown only cancels it.
                    if (s_autooff_left >= 0) { s_btn[i].swallow = 1; s_autooff_left = -1; }
                    ui_activity();
                }
                if (s_btn[i].pressed && !s_btn[i].longfired &&
                    s_btn[i].stable >= LONG_TICKS) {
                    s_btn[i].longfired = 1;
                    if (!s_btn[i].swallow) on_long(i);
                }
            } else {
                if (s_btn[i].pressed && !s_btn[i].longfired && !s_btn[i].swallow)
                    on_short(i);
                s_btn[i].stable = 0;
                s_btn[i].pressed = 0;
                s_btn[i].longfired = 0;
                s_btn[i].swallow = 0;
            }
        }
        chord_poll();
        autooff_poll();
        leds_set_state((int)disk_get_state());
        sd_detect_poll();
        // LTC2954 shutdown request. TODO: let an in-flight write finish before
        // asserting KILL.
        if (power_present() && gpio_get_level(LTC_INT_GPIO) == 0) {
            if (++off_stable == 3) { off_stable = 0; power_kill(); }
        } else {
            off_stable = 0;
        }
        lastoff_tick();
        indicate(disk_get_state());
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

void ui_init(void)
{
    opq_init(&s_ops, UI_OP_N);
    autooff_user_load();
    s_wake = xSemaphoreCreateBinary();
    lastoff_boot();     // read + clear why the board last went off

    uint64_t mask = 0;
    for (int i = 0; i < BTN_N; i++) mask |= 1ULL << s_btn[i].gpio;
    gpio_config_t btn = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);

    xTaskCreatePinnedToCore(ui_task, "ui", 3072, NULL, 3, NULL, 0);
}

void ui_trigger(void) { ui_trigger_op(UI_OP_READ); }

void ui_trigger_op(int op)
{
    if (!s_wake) return;
    // A delete-commit goes first: a read queued ahead of it would reload the
    // image and drop the delete.
    portENTER_CRITICAL(&s_ops_mux);
    bool added = opq_push(&s_ops, op, op == UI_OP_COMMIT);
    portEXIT_CRITICAL(&s_ops_mux);
    if (added) xSemaphoreGive(s_wake);
}

// An op leaves the queue as its job starts, so a trigger during the job
// queues another.
bool ui_wait_for_op(int *op, TickType_t timeout)
{
    for (;;) {
        portENTER_CRITICAL(&s_ops_mux);
        bool got = opq_pop(&s_ops, op);
        if (got) { s_running = *op; s_cancel = false; }
        portEXIT_CRITICAL(&s_ops_mux);
        if (got) return true;
        if (xSemaphoreTake(s_wake, timeout) != pdTRUE) return false;
    }
}

void ui_op_done(void) { s_running = -1; s_cancel = false; }
void ui_cancel(void)   { if (s_running >= 0) s_cancel = true; }
bool ui_cancelled(void) { return s_cancel; }
int  ui_running_op(void) { return s_running; }

uint32_t ui_queued_mask(void)
{
    portENTER_CRITICAL(&s_ops_mux);
    uint32_t m = opq_mask(&s_ops);
    portEXIT_CRITICAL(&s_ops_mux);
    return m;
}

const char *ui_op_name(int op)
{
    static const char *const names[UI_OP_N] = {
        [UI_OP_READ] = "read",       [UI_OP_WTEST] = "wtest",
        [UI_OP_COMMIT] = "commit",   [UI_OP_SAVEALL] = "save",
        [UI_OP_SAVEALL_FORCE] = "save", [UI_OP_RETRY] = "retry",
        [UI_OP_WRITE_DISK] = "write", [UI_OP_TEST] = "test",
        [UI_OP_TEST_QUICK] = "test", [UI_OP_HEALTH] = "health",
    };
    return (op >= 0 && op < UI_OP_N) ? names[op] : "";
}
