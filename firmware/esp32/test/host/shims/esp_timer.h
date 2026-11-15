// Host-test shim for esp_timer.h — mfm_decode.c uses esp_timer_get_time() only
// for GW_PROFILE accumulation; a coarse monotonic microsecond clock is fine.
#pragma once
#include <stdint.h>
#include <time.h>

static inline int64_t esp_timer_get_time(void)
{
    return (int64_t)clock() * 1000000LL / CLOCKS_PER_SEC;
}
