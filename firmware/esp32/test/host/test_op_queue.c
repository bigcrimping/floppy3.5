// Host tests for op_queue.c: the order in which the gw task gets its ops.
#include <stdio.h>
#include "op_queue.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { \
    g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

// The ui.h numbering, so the cases read like the real ones.
enum { READ = 0, WTEST, COMMIT, SAVE, SAVE_FORCE, RETRY, WRITE, N_OPS };

static int pop(opq_t *q)
{
    int op = -1;
    return opq_pop(q, &op) ? op : -1;
}

int main(void)
{
    printf("test_op_queue\n");
    opq_t q;

    // Empty.
    opq_init(&q, N_OPS);
    CHECK(pop(&q) == -1, "pop from an empty queue");
    CHECK(opq_mask(&q) == 0, "empty mask");

    // Ops come out in the order triggered.
    CHECK(opq_push(&q, SAVE, false) && opq_push(&q, READ, false), "two pushes");
    CHECK(opq_mask(&q) == ((1u << SAVE) | (1u << READ)), "mask has both");
    CHECK(pop(&q) == SAVE, "first in, first out");
    CHECK(pop(&q) == READ, "second out");
    CHECK(pop(&q) == -1, "then empty");

    // The same op type is queued once.
    opq_init(&q, N_OPS);
    CHECK(opq_push(&q, READ, false), "read queued");
    CHECK(!opq_push(&q, READ, false), "second read refused");
    CHECK(pop(&q) == READ && pop(&q) == -1, "only one read comes out");
    CHECK(opq_push(&q, READ, false), "and it can be queued again once taken");

    // A delete-commit jumps a read that was queued first.
    opq_init(&q, N_OPS);
    opq_push(&q, READ, false);
    opq_push(&q, SAVE, false);
    CHECK(opq_push(&q, COMMIT, true), "commit to the front");
    CHECK(pop(&q) == COMMIT, "commit runs first");
    CHECK(pop(&q) == READ && pop(&q) == SAVE, "the rest keep their order");

    // A second commit while one waits is the same commit.
    opq_init(&q, N_OPS);
    opq_push(&q, COMMIT, true);
    opq_push(&q, READ, false);
    CHECK(!opq_push(&q, COMMIT, true), "second commit refused");
    CHECK(pop(&q) == COMMIT && pop(&q) == READ && pop(&q) == -1, "one commit, then the read");

    // Every op type at once fits, and nothing else does.
    opq_init(&q, N_OPS);
    int pushed = 0;
    for (int op = 0; op < N_OPS; op++) pushed += opq_push(&q, op, false);
    CHECK(pushed == N_OPS, "one of each fits (%d)", pushed);
    CHECK(opq_mask(&q) == (1u << N_OPS) - 1, "mask is full");
    for (int op = 0; op < N_OPS; op++) CHECK(pop(&q) == op, "op %d in order", op);

    // Out of range.
    opq_init(&q, N_OPS);
    CHECK(!opq_push(&q, -1, false), "negative op refused");
    CHECK(!opq_push(&q, N_OPS, false), "op past the last refused");
    CHECK(!opq_push(&q, 31, true), "wild op refused");
    CHECK(pop(&q) == -1, "queue still empty");
    opq_init(&q, 100);
    CHECK(!opq_push(&q, OPQ_MAX, false), "n_types is clamped to OPQ_MAX");

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    if (g_fail == 0) printf("ALL PASS\n");
    return g_fail ? 1 : 0;
}
