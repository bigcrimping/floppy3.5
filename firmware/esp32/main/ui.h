// Buttons, LED state and the trigger that hands work to the gw task (shared by
// the buttons and the web API).
#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"

void ui_init(void);

//   READ          - full disk read
//   WTEST         - write self-test (rewrite cyl0 unchanged)
//   COMMIT        - write a delete back to the disk (cyl0)
//   SAVEALL       - archive to SD, reading the disk first if needed
//   SAVEALL_FORCE - same, even if the disk is already on the card
//   RETRY         - re-read only the tracks that came out bad, into the same
//                   image; a full read if there is nothing to merge into
//   WRITE_DISK    - write every track from the staged image
//                   (disk_stage_set()), or from the loaded one if none is staged
enum {
    UI_OP_READ = 0,
    UI_OP_WTEST = 1,
    UI_OP_COMMIT = 2,
    UI_OP_SAVEALL = 3,
    UI_OP_SAVEALL_FORCE = 4,
    UI_OP_RETRY = 5,
    UI_OP_WRITE_DISK = 6,
    UI_OP_N
};

// Queue an op for the gw task. Ops run in the order triggered, except that a
// COMMIT runs first; an op type already waiting is not queued twice.
void ui_trigger(void);           // UI_OP_READ
void ui_trigger_op(int op);

// The gw task blocks here for the next op, and calls ui_op_done() when it has
// finished it.
bool ui_wait_for_op(int *op, TickType_t timeout);
void ui_op_done(void);

// For /api/status and the LCD: the op being run (-1 = none), a bit per op
// type still waiting, and a short name for an op.
int         ui_running_op(void);
uint32_t    ui_queued_mask(void);
const char *ui_op_name(int op);
