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

static const char *TAG = "ui";
// Ops waiting for the gw task (op_queue.h), under s_ops_mux; s_wake wakes it.
static opq_t             s_ops;
static SemaphoreHandle_t s_wake;
static portMUX_TYPE      s_ops_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int      s_running = -1;

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
} s_btn[BTN_N] = {
    { UI_BUTTON_GPIO,      0, 0, 0 },
    { UI_BTN_SAVEALL_GPIO, 0, 0, 0 },
    { UI_BTN_DISPLAY_GPIO, 0, 0, 0 },
};

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
        // Home screen toggle, plus the battery gauge on the LEDs.
        display_toggle_home();
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
                if (s_btn[i].stable == DEBOUNCE_TICKS)
                    s_btn[i].pressed = 1;
                if (s_btn[i].pressed && !s_btn[i].longfired &&
                    s_btn[i].stable >= LONG_TICKS) {
                    s_btn[i].longfired = 1;
                    on_long(i);
                }
            } else {
                if (s_btn[i].pressed && !s_btn[i].longfired)
                    on_short(i);
                s_btn[i].stable = 0;
                s_btn[i].pressed = 0;
                s_btn[i].longfired = 0;
            }
        }
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
        if (got) s_running = *op;
        portEXIT_CRITICAL(&s_ops_mux);
        if (got) return true;
        if (xSemaphoreTake(s_wake, timeout) != pdTRUE) return false;
    }
}

void ui_op_done(void) { s_running = -1; }
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
        [UI_OP_WRITE_DISK] = "write",
    };
    return (op >= 0 && op < UI_OP_N) ? names[op] : "";
}
