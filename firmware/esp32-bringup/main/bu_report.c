#include "bu_report.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define MAX_ITEMS 48

typedef struct {
    char        item[28];
    bu_status_t st;
    char        detail[96];
} entry_t;

static entry_t s_items[MAX_ITEMS];
static int     s_n;

const char *bu_status_str(bu_status_t st)
{
    switch (st) {
    case BU_PASS:   return "PASS";
    case BU_FAIL:   return "FAIL";
    case BU_WARN:   return "WARN";
    case BU_VISUAL: return "LOOK";
    case BU_INFO:   return "INFO";
    default:        return "SKIP";
    }
}

// Severity order for "worst of": FAIL > WARN > VISUAL > PASS > INFO > SKIP.
static int rank(bu_status_t st)
{
    switch (st) {
    case BU_FAIL:   return 5;
    case BU_WARN:   return 4;
    case BU_VISUAL: return 3;
    case BU_PASS:   return 2;
    case BU_INFO:   return 1;
    default:        return 0;
    }
}

void bu_report_reset(void) { s_n = 0; }

void bu_report(const char *item, bu_status_t st, const char *fmt, ...)
{
    entry_t tmp, *e = (s_n < MAX_ITEMS) ? &s_items[s_n++] : &tmp;
    strlcpy(e->item, item, sizeof(e->item));
    e->st = st;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(e->detail, sizeof(e->detail), fmt, ap);
    va_end(ap);
    // Plain printf (not ESP_LOG) so the line is clean on both consoles.
    printf("  [%s] %-26s %s\n", bu_status_str(st), e->item, e->detail);
}

int bu_report_count(bu_status_t st)
{
    int n = 0;
    for (int i = 0; i < s_n; i++) if (s_items[i].st == st) n++;
    return n;
}

bu_status_t bu_report_worst(const char *prefix)
{
    bu_status_t w = BU_SKIP;
    size_t pl = strlen(prefix);
    for (int i = 0; i < s_n; i++) {
        if (strncmp(s_items[i].item, prefix, pl) != 0) continue;
        if (rank(s_items[i].st) > rank(w)) w = s_items[i].st;
    }
    return w;
}

void bu_report_summary(void)
{
    printf("\n================= BRING-UP SUMMARY =================\n");
    for (int i = 0; i < s_n; i++)
        printf("  [%s] %-26s %s\n", bu_status_str(s_items[i].st), s_items[i].item,
               s_items[i].detail);
    printf("----------------------------------------------------\n");
    printf("  PASS %d   FAIL %d   WARN %d   LOOK %d   SKIP %d\n",
           bu_report_count(BU_PASS), bu_report_count(BU_FAIL), bu_report_count(BU_WARN),
           bu_report_count(BU_VISUAL), bu_report_count(BU_SKIP));
    printf("====================================================\n\n");
}
