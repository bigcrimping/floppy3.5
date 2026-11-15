// Host tests for web.c's pure helpers (MIME lookup + filename safety).
#include <stdio.h>
#include <string.h>
#include "web_util.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { \
    g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define MIME_IS(name, want) \
    CHECK(strcmp(web_mime_for(name), want) == 0, \
          "mime(\"%s\")=\"%s\" want \"%s\"", name, web_mime_for(name), want)

int main(void)
{
    printf("test_web_util\n");

    // --- web_mime_for: extension mapping is case-insensitive ---
    MIME_IS("DSC00001.JPG", "image/jpeg");
    MIME_IS("photo.jpg",    "image/jpeg");
    MIME_IS("photo.jpeg",   "image/jpeg");
    MIME_IS("MiXeD.JpG",    "image/jpeg");
    MIME_IS("icon.PNG",     "image/png");
    MIME_IS("anim.gif",     "image/gif");
    MIME_IS("notes.TXT",    "text/plain");
    MIME_IS("firmware.bin", "application/octet-stream");   // unknown ext
    MIME_IS("README",       "application/octet-stream");   // no extension
    MIME_IS("archive.tar.gz","application/octet-stream");  // last ext only
    MIME_IS("weird.jpgx",   "application/octet-stream");   // not exactly .jpg

    // --- web_name_is_safe ---
    CHECK(web_name_is_safe("DSC00001.JPG"),  "flat name is safe");
    CHECK(web_name_is_safe("MVC-0001.JPG"),  "dash name is safe");
    CHECK(!web_name_is_safe(""),             "empty rejected");
    CHECK(!web_name_is_safe(NULL),           "NULL rejected");
    CHECK(!web_name_is_safe(".."),           "'..' rejected");
    CHECK(!web_name_is_safe("../secret"),    "parent traversal rejected");
    CHECK(!web_name_is_safe("a/b.jpg"),      "forward slash rejected");
    CHECK(!web_name_is_safe("a\\b.jpg"),     "back slash rejected");
    CHECK(!web_name_is_safe("x..y"),         "embedded '..' rejected (conservative)");

    // --- Mavica thumbnail companion (deleted along with its picture) ---
    char th[16];
    CHECK(web_thumb_for("MVC-034S.JPG", th, sizeof th) && strcmp(th, "MVC-034S.411") == 0,
          "MVC-034S.JPG -> MVC-034S.411");
    CHECK(web_thumb_for("MVC-001F.jpg", th, sizeof th) && strcmp(th, "MVC-001F.411") == 0,
          "lower-case extension");
    CHECK(!web_thumb_for("MVC-034S.411", th, sizeof th), ".411 itself has no companion");
    CHECK(!web_thumb_for("mvc-034s.411", th, sizeof th), ".411 match is case-insensitive");
    CHECK(!web_thumb_for("README", th, sizeof th), "no extension -> no companion");
    CHECK(!web_thumb_for(".JPG", th, sizeof th), "empty stem -> no companion");
    CHECK(!web_thumb_for("MVC-034S.JPG", th, 12), "too-small buffer refused");
    CHECK(web_thumb_for("MVC-034S.JPG", th, 13), "exact-fit buffer accepted");

    // --- query-value percent-decoding (the SD thumbnail 404) ---
    char q[64];
    strcpy(q, "DISK0003%2FMVC-007S.JPG");
    CHECK(web_url_decode(q) && strcmp(q, "DISK0003/MVC-007S.JPG") == 0, "%%2F -> '/'");
    strcpy(q, "Get_started_with_GoPro.url");
    CHECK(web_url_decode(q) && strcmp(q, "Get_started_with_GoPro.url") == 0, "plain name unchanged");
    strcpy(q, "a%20b+c%2b");
    CHECK(web_url_decode(q) && strcmp(q, "a b c+") == 0, "%%20 and '+' -> space, %%2b -> '+'");
    strcpy(q, "%2e%2e%2Fsecret");
    CHECK(web_url_decode(q) && strcmp(q, "../secret") == 0 && !web_name_is_safe(q),
          "encoded traversal decodes and is still rejected");
    strcpy(q, "bad%zzesc%4");
    CHECK(web_url_decode(q) && strcmp(q, "bad%zzesc%4") == 0, "malformed escapes kept literally");
    strcpy(q, "x%00y");
    CHECK(!web_url_decode(q), "encoded NUL refused");

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    if (g_fail == 0) printf("ALL PASS\n");
    return g_fail ? 1 : 0;
}
