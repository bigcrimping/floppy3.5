#include "op_queue.h"
#include <string.h>

void opq_init(opq_t *q, int n_types)
{
    memset(q, 0, sizeof *q);
    q->n_types = (uint8_t)(n_types > OPQ_MAX ? OPQ_MAX : n_types);
}

uint32_t opq_mask(const opq_t *q)
{
    uint32_t m = 0;
    for (int i = 0; i < q->n; i++) m |= 1u << q->ops[i];
    return m;
}

bool opq_push(opq_t *q, int op, bool front)
{
    if (op < 0 || op >= q->n_types) return false;
    if (opq_mask(q) & (1u << op)) return false;
    if (front) {
        memmove(q->ops + 1, q->ops, q->n);
        q->ops[0] = (uint8_t)op;
    } else {
        q->ops[q->n] = (uint8_t)op;
    }
    q->n++;
    return true;
}

bool opq_pop(opq_t *q, int *op)
{
    if (!q->n) return false;
    *op = q->ops[0];
    q->n--;
    memmove(q->ops, q->ops + 1, q->n);
    return true;
}
