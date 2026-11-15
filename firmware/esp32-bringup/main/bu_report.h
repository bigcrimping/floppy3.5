// Result table for the bring-up run. Every test files one or more lines here;
// the sequencer prints the summary and maps it onto the LEDs / OLED.
#pragma once
#include <stdbool.h>

typedef enum {
    BU_PASS = 0,   // measured, correct
    BU_FAIL,       // measured, wrong (or expected part did not answer)
    BU_WARN,       // measured, unexpected but plausibly a bench condition (no card, no battery)
    BU_VISUAL,     // cannot be read back — the assembler must look / measure
    BU_INFO,       // identity / measurement, no pass criterion
    BU_SKIP,       // not run (needs a manual action — see the detail)
} bu_status_t;

void        bu_report_reset(void);
void        bu_report(const char *item, bu_status_t st, const char *fmt, ...)
                __attribute__((format(printf, 3, 4)));
void        bu_report_summary(void);
int         bu_report_count(bu_status_t st);
// Worst status among items whose name starts with `prefix` (BU_SKIP if none).
bu_status_t bu_report_worst(const char *prefix);
const char *bu_status_str(bu_status_t st);
