// SPDX-License-Identifier: Unlicense
// Greaseweazle command engine.
#pragma once

void gw_floppy_init(void);
// One pass; blocks only inside flux captures/writes. Call every superloop pass.
void gw_floppy_process(void);
