// Force-included (-include) into the test_sd_index build: the POSIX bits
// sd_index.c uses that the Windows C runtime lacks, and a card "mount point"
// that is a plain folder beside the test binary.
#pragma once
#include <stddef.h>
#include <string.h>

#define SD_MOUNT_POINT "sdcard_host"

static size_t strlcpy(char *dst, const char *src, size_t n)
{
    size_t len = strlen(src);
    if (n) {
        size_t c = len < n - 1 ? len : n - 1;
        memcpy(dst, src, c);
        dst[c] = '\0';
    }
    return len;
}

int _stricmp(const char *a, const char *b);
int _strnicmp(const char *a, const char *b, size_t n);
int _commit(int fd);
#define strcasecmp  _stricmp
#define strncasecmp _strnicmp
#define fsync       _commit
