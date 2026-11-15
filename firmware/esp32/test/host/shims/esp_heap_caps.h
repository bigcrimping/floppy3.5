// Host-test shim for esp_heap_caps.h — maps PSRAM allocs to plain malloc.
#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM   0
#define MALLOC_CAP_INTERNAL 0
#define MALLOC_CAP_8BIT     0
#define MALLOC_CAP_DEFAULT  0
static inline void *heap_caps_malloc(size_t sz, int caps) { (void)caps; return malloc(sz); }
static inline void  heap_caps_free(void *p) { free(p); }
