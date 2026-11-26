// The order of ops waiting for the gw task. No locking and no ESP-IDF
// dependencies (ui.c wraps it in a critical section), so the host tests cover
// it.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define OPQ_MAX 16  // op types, numbered 0..OPQ_MAX-1 (<= 32: opq_mask)

typedef struct {
    uint8_t ops[OPQ_MAX];   // oldest first
    uint8_t n;
    uint8_t n_types;
} opq_t;

void opq_init(opq_t *q, int n_types);

// Queue `op` at the back, or at the front with `front`. False, and no change,
// if that op type is already waiting or `op` is out of range. A slot per op
// type, so it can't be full.
bool opq_push(opq_t *q, int op, bool front);

// Take the oldest op. False if none is waiting.
bool opq_pop(opq_t *q, int *op);

// Bit per op type that is waiting.
uint32_t opq_mask(const opq_t *q);
