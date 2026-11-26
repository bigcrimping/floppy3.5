// SPDX-License-Identifier: Unlicense
// Shugart bus: every line active-low, inputs open-collector with pull-ups.
#include "floppy_drive.h"

#include <string.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "board_rp2354.h"

static const gw_delay_t factory_delays = {
    .select_delay = DELAY_SELECT_US,
    .step_delay = DELAY_STEP_US,
    .seek_settle = DELAY_SETTLE_MS,
    .motor_delay = DELAY_MOTOR_MS,
    .watchdog = DELAY_WATCHDOG_MS,
};
static gw_delay_t delays;

static int cur_cyl = -1; // -1 = unknown, next seek recalibrates
static bool motor_on;

// Recalibration only needs to find TRK0, so it steps faster than step_delay.
#define RECAL_STEP_MS 3
#define RECAL_MAX_STEPS 100 // 83 cylinders + margin

// Shugart minimums are 0.8 us step pulse and 1 us direction setup.
#define STEP_PULSE_US 4
#define DIR_SETUP_US 2

static void out_pin(uint pin, bool level) {
  gpio_init(pin);
  gpio_put(pin, level); // before enabling the output, so no glitch
  gpio_set_dir(pin, GPIO_OUT);
}

static void in_pin(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_pull_up(pin);
}

void drive_init(void) {
  // Outputs idle high (deasserted). flux_pio.c returns WRGATE/WRDATA to this
  // state after every write.
  out_pin(PIN_SELECT, 1);
  out_pin(PIN_MOTOR, 1);
  out_pin(PIN_DIR, 1);
  out_pin(PIN_STEP, 1);
  out_pin(PIN_SIDE, 1); // side 0
  out_pin(PIN_DENSITY, 1);
  out_pin(PIN_WRGATE, 1);
  out_pin(PIN_WRDATA, 1);

  in_pin(PIN_TRK0);
  in_pin(PIN_PROT);
  in_pin(PIN_INDEX);
  in_pin(PIN_RDDATA);
#ifdef PIN_DSKCHG
  in_pin(PIN_DSKCHG);
#endif
#ifdef PIN_DENS
  in_pin(PIN_DENS);
#endif
#ifdef PIN_RDY
  in_pin(PIN_RDY);
#endif

  delays = factory_delays;
  cur_cyl = -1;
  motor_on = false;
}

gw_delay_t *drive_delays(void) { return &delays; }

void drive_select(bool on) {
  gpio_put(PIN_SELECT, on ? 0 : 1);
  if (on)
    sleep_us(delays.select_delay);
}

void drive_motor(bool on) {
  if (motor_on == on)
    return;
  gpio_put(PIN_MOTOR, on ? 0 : 1);
  motor_on = on;
  // Block so the command's ack means "spun up".
  if (on)
    sleep_ms(delays.motor_delay);
}

// DIR low = step inward (higher cylinders).
static void set_direction(bool toward_trk0) {
  gpio_put(PIN_DIR, toward_trk0 ? 1 : 0);
  sleep_us(DIR_SETUP_US);
}

static void step_pulse(void) {
  gpio_put(PIN_STEP, 0);
  sleep_us(STEP_PULSE_US);
  gpio_put(PIN_STEP, 1);
}

#define RECAL_EXTRA_SETTLE_MS 100

uint8_t drive_seek(int cyl) {
  if (cyl < 0 || cyl > 83)
    return ACK_BAD_CYLINDER;

  if (cur_cyl < 0) {
    // Recalibrate. Bounded so a dead drive fails in ~300 ms.
    set_direction(true);
    int budget = RECAL_MAX_STEPS;
    while (!drive_trk0()) {
      if (!budget--)
        return ACK_NO_TRK0;
      step_pulse();
      sleep_ms(RECAL_STEP_MS);
    }
    cur_cyl = 0;
    // A recalibration follows a drive power-up (the ESP switches the 5 V rail
    // per job). The MPF920 then keeps RDDATA silent ~30 ms past the normal
    // settle, which cost the first track 2 sectors once reads started
    // streaming immediately (speed-up phase 3). Once per job, so be generous.
    sleep_ms(RECAL_EXTRA_SETTLE_MS);
  }

  if (cur_cyl != cyl) {
    set_direction(cyl < cur_cyl);
    int dir = (cyl < cur_cyl) ? -1 : 1;
    while (cur_cyl != cyl) {
      step_pulse();
      sleep_us(delays.step_delay);
      cur_cyl += dir;
    }
  }
  sleep_ms(delays.seek_settle);
  return ACK_OKAY;
}

void drive_head(int head) {
  gpio_put(PIN_SIDE, head ? 0 : 1); // low = head 1
}

void drive_reset(void) {
  drive_motor(false);
  drive_select(false);
  delays = factory_delays;
  cur_cyl = -1;
}

void drive_forget_position(void) { cur_cyl = -1; }

// Logical (asserted = true) views of the active-low inputs.
bool drive_wrprot(void) { return !gpio_get(PIN_PROT); }
bool drive_trk0(void) { return !gpio_get(PIN_TRK0); }
bool drive_index(void) { return !gpio_get(PIN_INDEX); }

#ifdef PIN_DSKCHG
bool drive_dskchg(void) { return !gpio_get(PIN_DSKCHG); }
bool drive_has_dskchg(void) { return true; }
#else
bool drive_dskchg(void) { return false; }
bool drive_has_dskchg(void) { return false; }
#endif
#ifdef PIN_DENS
bool drive_dens_sense(void) { return !gpio_get(PIN_DENS); }
bool drive_has_dens_sense(void) { return true; }
#else
bool drive_dens_sense(void) { return false; }
bool drive_has_dens_sense(void) { return false; }
#endif
#ifdef PIN_RDY
bool drive_rdy(void) { return !gpio_get(PIN_RDY); }
bool drive_has_rdy(void) { return true; }
#else
bool drive_rdy(void) { return false; }
bool drive_has_rdy(void) { return false; }
#endif

void drive_set_density(bool high) {
  gpio_put(PIN_DENSITY, high ? 0 : 1);
}
