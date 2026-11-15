#include "web_util.h"
#include <string.h>

// strcasecmp isn't in every host libc.
static bool ci_eq(const char *a, const char *b)
{
    for (; *a && *b; a++, b++) {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca += 32;
        if (cb >= 'A' && cb <= 'Z') cb += 32;
        if (ca != cb) return false;
    }
    return *a == *b;
}

const char *web_mime_for(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (!dot) return "application/octet-stream";
    if (ci_eq(dot, ".jpg") || ci_eq(dot, ".jpeg")) return "image/jpeg";
    if (ci_eq(dot, ".png")) return "image/png";
    if (ci_eq(dot, ".gif")) return "image/gif";
    if (ci_eq(dot, ".txt")) return "text/plain";
    return "application/octet-stream";
}

bool web_name_is_safe(const char *name)
{
    if (!name || name[0] == '\0') return false;
    if (strstr(name, "..")) return false;
    if (strchr(name, '/') || strchr(name, '\\')) return false;
    return true;
}

bool web_thumb_for(const char *name, char *out, size_t out_sz)
{
    const char *dot = strrchr(name, '.');
    if (!dot || dot == name || ci_eq(dot, ".411")) return false;
    size_t stem = (size_t)(dot - name);
    if (stem + 5 > out_sz) return false;          // ".411" + NUL
    memcpy(out, name, stem);
    memcpy(out + stem, ".411", 5);
    return true;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool web_url_decode(char *s)
{
    char *w = s;
    for (const char *r = s; *r; r++) {
        int hi, lo;
        if (*r == '%' && (hi = hexval(r[1])) >= 0 && (lo = hexval(r[2])) >= 0) {
            char c = (char)(hi * 16 + lo);
            if (c == '\0') { *w = '\0'; return false; }
            *w++ = c;
            r += 2;
        } else {
            *w++ = (*r == '+') ? ' ' : *r;
        }
    }
    *w = '\0';
    return true;
}
