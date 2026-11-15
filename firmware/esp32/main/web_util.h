// Web helpers with no ESP-IDF dependencies, so the host tests can cover them.
#pragma once
#include <stdbool.h>
#include <stddef.h>

// By extension, case-insensitive; application/octet-stream if unknown.
const char *web_mime_for(const char *name);

// A non-empty flat name: no "..", '/' or '\'.
bool web_name_is_safe(const char *name);

// The Mavica thumbnail name for a picture (MVC-001S.JPG -> MVC-001S.411).
// False if there is no extension, it is already a .411, or out is too small.
bool web_thumb_for(const char *name, char *out, size_t out_sz);

// Percent-decode in place (%XX, '+' -> space); malformed escapes are kept.
// Decode before the safety checks so an encoded ".." is caught. False if the
// result would contain a NUL.
bool web_url_decode(char *s);
