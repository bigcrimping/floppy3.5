// SPDX-License-Identifier: Unlicense
// Drive control: select, motor, seek, side and the status inputs.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "gw_proto.h"

void drive_init(void);          // GPIO setup, factory delays
gw_delay_t *drive_delays(void); // SET_PARAMS/GET_PARAMS target

void drive_select(bool on); // select waits select_delay
// On blocks for motor_delay before returning; off is immediate.
void drive_motor(bool on);

// Seek to cylinder 0-83, recalibrating first if the position is unknown.
// Returns ACK_OKAY / ACK_NO_TRK0 / ACK_BAD_CYLINDER.
uint8_t drive_seek(int cyl);
void drive_head(int head);        // 0/1
void drive_reset(void);           // motor off, deselect, factory delays
void drive_forget_position(void); // drive lost power

bool drive_wrprot(void);
bool drive_trk0(void);
bool drive_index(void);
void drive_set_density(bool high); // density SELECT output

// Optional inputs; a board defines PIN_DSKCHG / PIN_DENS / PIN_RDY only if it
// routes them. Each returns true when asserted, false if not routed. Nothing
// gates drive operations on them.
bool drive_dskchg(void);     // disk change (ribbon pin 34)
bool drive_dens_sense(void); // media density sense (slimline J1 pin 9)
bool drive_rdy(void);        // drive ready (slimline J1 pin 8)
bool drive_has_dskchg(void);
bool drive_has_dens_sense(void);
bool drive_has_rdy(void);
