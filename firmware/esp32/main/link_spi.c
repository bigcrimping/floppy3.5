// SPDX-License-Identifier: Unlicense
// GWSPI v1 master (docs/spi-link-protocol.md). One CS-low window carries one
// frame each way; the slave's frame holds what it had queued beforehand, not
// a reply to ours. Reliability is seq/ack with a window of 2: go-back-N
// replays the recorded slices byte-for-byte from the caller's buffer.
// Transient frame failures are normal; the link is declared down only by the
// SYNC timeout or a verb exhausting its deadline.
#include "link.h"
#include "app_config.h"
#include "crc16.h"
#include "power.h"
#include "../../common/gwspi_wire.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "link";

#define RX_RING_SZ (16 * 1024)

#define SYNC_POLL_MS   50
#define BACKOFF_MAX_MS 10 // transient retries back off 1 ms doubling to this

// Backstop for a missed DRDY edge while idle. Long on purpose: every poll is
// an interrupt on the slave, landing on its flux engine during a write.
#define DRDY_BACKSTOP_MS 50

// ---- state ----

static spi_device_handle_t s_dev;
static bool s_up;             // SYNC handshake completed
static bool s_bus_live;       // we own an spi_bus_initialize to undo
static bool s_isr_live;       // DRDY handler installed
static uint8_t *s_txf, *s_rxf; // DMA frames in internal RAM
static uint8_t *s_ring;        // received payload bytes awaiting link_read
static size_t s_rhead, s_rtail, s_rcount;

// Only one task may be inside a verb at a time (asserted, not locked); the
// DRDY ISR notifies it. s_task is cleared on exit because app_main's TCB is
// freed when it returns.
static TaskHandle_t volatile s_task;
static volatile bool  s_in_verb;

static uint8_t  s_tx_seq;       // seq of the next new payload frame
static uint8_t  s_rx_last;      // last in-order slave seq (0xFF after SYNC)
static uint16_t s_peer_avail;   // slave's queued output, from its last header
static uint16_t s_peer_credit;  // slave's rx-ring free space, ditto
static int64_t  s_cs_up_us;     // when the previous transaction's CS rose

// In-flight tx window, used only inside link_write. A seq must always map to
// the same byte range because the slave detects duplicates by seq alone.
typedef struct { uint8_t seq; size_t off; uint16_t len; } slice_t;
static slice_t s_fly[2];
static int     s_nfly;
static size_t  s_wacked;        // bytes of the current write acked so far

static struct {
    uint32_t exchanges, bytes_tx, bytes_rx;
    uint32_t hcrc_fail, pcrc_fail, retransmits, dups, seq_err;
    uint32_t sync_resets, drdy_wakes, credit_stalls;
} s_st;

// ---- small helpers ----

static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline uint16_t get16(const uint8_t *p)   { return (uint16_t)(p[0] | (p[1] << 8)); }

static void verb_enter(void)
{
    assert(!s_in_verb);
    s_in_verb = true;
    s_task = xTaskGetCurrentTaskHandle();
}
static void verb_exit(void)
{
    s_task = NULL;
    s_in_verb = false;
}

static size_t ring_free(void) { return RX_RING_SZ - s_rcount; }

static void ring_push(const uint8_t *p, size_t n)
{
    size_t first = RX_RING_SZ - s_rhead;
    if (first > n) first = n;
    memcpy(s_ring + s_rhead, p, first);
    memcpy(s_ring, p + first, n - first);
    s_rhead = (s_rhead + n) % RX_RING_SZ;
    s_rcount += n;
}

static size_t ring_pop(uint8_t *dst, size_t n)
{
    if (n > s_rcount) n = s_rcount;
    size_t first = RX_RING_SZ - s_rtail;
    if (first > n) first = n;
    memcpy(dst, s_ring + s_rtail, first);
    memcpy(dst + first, s_ring, n - first);
    s_rtail = (s_rtail + n) % RX_RING_SZ;
    s_rcount -= n;
    return n;
}

static void IRAM_ATTR drdy_isr(void *arg)
{
    (void)arg;
    s_st.drdy_wakes++;
    BaseType_t hpw = pdFALSE;
    TaskHandle_t t = s_task;
    if (t) vTaskNotifyGiveFromISR(t, &hpw);
    portYIELD_FROM_ISR(hpw);
}

// Enforce GW_SPI_GAP_US between CS-up and the next CS-down.
static void gap_wait(void)
{
    for (;;) {
        int64_t rem = (s_cs_up_us + GW_SPI_GAP_US) - esp_timer_get_time();
        if (rem <= 0) return;
        if (rem > 1000) vTaskDelay(pdMS_TO_TICKS((uint32_t)(rem / 1000)));
        else            taskYIELD();
    }
}

// ---- frame codec ----

typedef struct {
    bool hdr_ok;   // header valid; control fields were used
    bool sync;
    bool usb;      // a USB host has the controller
    int  popped;   // tx-window slices retired by this frame's ack
} rxinfo_t;

// Header fields are used whenever the header CRC is valid; payload only when
// its CRC and the seq check also pass (§2.1-2.3).
static void parse_frame(const uint8_t *f, size_t clocked, rxinfo_t *ri)
{
    if (f[GWSPI_OFF_MAGIC] != GWSPI_MAGIC_S2M ||
        (f[GWSPI_OFF_FLAGS] & 0x0F) != GWSPI_VERSION ||
        get16(f + GWSPI_OFF_HCRC) != crc16_ccitt_tab(0xFFFF, f, GWSPI_OFF_HCRC)) {
        s_st.hcrc_fail++;               // also: slave dead, in reset or busy
        return;
    }
    ri->hdr_ok = true;
    ri->sync   = (f[GWSPI_OFF_FLAGS] & GWSPI_FLAG_SYNC) != 0;
    ri->usb    = (f[GWSPI_OFF_FLAGS] & GWSPI_FLAG_USB) != 0;
    uint8_t  ack = f[GWSPI_OFF_ACK];
    uint16_t len = get16(f + GWSPI_OFF_LEN);
    s_peer_avail  = get16(f + GWSPI_OFF_AVAIL);
    s_peer_credit = get16(f + GWSPI_OFF_CREDIT);

    // ack == s0 retires one slice, s0+1 both; anything else is stale.
    if (s_nfly) {
        uint8_t d = (uint8_t)(ack - s_fly[0].seq);
        if (d < (uint8_t)s_nfly) {
            int pop = (int)d + 1;
            for (int i = 0; i < pop; i++)
                s_wacked = s_fly[i].off + s_fly[i].len;
            s_nfly -= pop;
            if (s_nfly) s_fly[0] = s_fly[pop];
            ri->popped = pop;
        }
    }

    if (len == 0) return;
    if (len > GWSPI_PAYLOAD_MAX || GWSPI_HDR_LEN + (size_t)len + 2 > clocked) {
        s_st.pcrc_fail++;               // slave sent past what we clocked
        return;
    }
    const uint8_t *pl = f + GWSPI_HDR_LEN;
    if (get16(pl + len) != crc16_ccitt_tab(0xFFFF, pl, len)) {
        s_st.pcrc_fail++;
        return;
    }
    uint8_t seq = f[GWSPI_OFF_SEQ];
    uint8_t exp = (uint8_t)(s_rx_last + 1);
    if (seq == exp) {
        if (ring_free() >= len) {       // our advertised credit guarantees this
            ring_push(pl, len);
            s_rx_last = exp;
            s_st.bytes_rx += len;
        } else {
            s_st.credit_stalls++;       // not acked, so the slave resends
        }
    } else if (seq == s_rx_last || seq == (uint8_t)(s_rx_last - 1)) {
        s_st.dups++;
    } else {
        s_st.seq_err++;                 // treated as a duplicate
    }
}

// One full-duplex transaction. Returns the SPI driver status; ri->hdr_ok says
// whether a valid frame came back.
static esp_err_t do_exchange(const uint8_t *pl, uint16_t plen, uint8_t seq,
                             uint16_t avail_after, bool sync, rxinfo_t *ri)
{
    memset(ri, 0, sizeof *ri);

    // Grant rule (§2.1): the slave sends at most the avail it last advertised,
    // so clock 12 + max(our len, grant) (+CRC), rounded up to 4.
    uint16_t grant = s_peer_avail > GWSPI_PAYLOAD_MAX ? GWSPI_PAYLOAD_MAX
                                                      : s_peer_avail;
    uint16_t L = plen > grant ? plen : grant;
    size_t used = GWSPI_HDR_LEN + plen + (plen ? 2u : 0u);
    size_t T = GWSPI_HDR_LEN + L + (L ? 2u : 0u);
    T = (T + 3u) & ~(size_t)3u;

    uint8_t *f = s_txf;
    f[GWSPI_OFF_MAGIC] = GWSPI_MAGIC_M2S;
    f[GWSPI_OFF_FLAGS] = GWSPI_VERSION | (sync ? GWSPI_FLAG_SYNC : 0) |
                         (power_boost_is_on() ? GWSPI_FLAG_RAIL : 0);
    f[GWSPI_OFF_SEQ]   = seq;
    f[GWSPI_OFF_ACK]   = s_rx_last;
    put16(f + GWSPI_OFF_LEN, plen);
    put16(f + GWSPI_OFF_AVAIL, avail_after);
    size_t cf = ring_free();
    put16(f + GWSPI_OFF_CREDIT, cf > 0xFFFF ? 0xFFFF : (uint16_t)cf);
    put16(f + GWSPI_OFF_HCRC, crc16_ccitt_tab(0xFFFF, f, GWSPI_OFF_HCRC));
    if (plen) {
        memcpy(f + GWSPI_HDR_LEN, pl, plen);
        put16(f + GWSPI_HDR_LEN + plen, crc16_ccitt_tab(0xFFFF, f + GWSPI_HDR_LEN, plen));
    }
    if (T > used) memset(f + used, 0, T - used);

    gap_wait();
    spi_transaction_t t = {
        .length    = T * 8,
        .tx_buffer = s_txf,
        .rx_buffer = s_rxf,
    };
    // Poll short transactions; a 4 ms flux frame is too long to spin on.
    esp_err_t e = (T <= 64) ? spi_device_polling_transmit(s_dev, &t)
                            : spi_device_transmit(s_dev, &t);
    s_cs_up_us = esp_timer_get_time();
    if (e != ESP_OK) return e;
    s_st.exchanges++;
    s_st.bytes_tx += plen;

    parse_frame(s_rxf, T, ri);
    return ESP_OK;
}

// seq for payload-less frames (ignored by the receiver).
static inline uint8_t poll_seq(void) { return (uint8_t)(s_tx_seq - 1); }

// ---- SYNC (§2.4) ----

// Reset both ends and poll with SYNC set until the slave answers with SYNC.
static esp_err_t link_sync(void)
{
    s_tx_seq = 0;
    s_rx_last = 0xFF;
    s_nfly = 0;
    s_peer_avail = 0;                   // first data exchange follows a
    s_peer_credit = 0;                  // header-only poll
    s_rhead = s_rtail = s_rcount = 0;

    int64_t dl = esp_timer_get_time() + (int64_t)GW_SYNC_TIMEOUT_MS * 1000;
    for (;;) {
        rxinfo_t ri;
        if (do_exchange(NULL, 0, 0, 0, true, &ri) == ESP_OK &&
            ri.hdr_ok && ri.sync) {
            // Reset again: an earlier attempt may have ingested a stale
            // pre-SYNC resend with seq 0, which would make the real first
            // payload look like a duplicate. The SYNC reply itself has no
            // payload.
            s_rx_last = 0xFF;
            s_rhead = s_rtail = s_rcount = 0;
            s_st.sync_resets++;
            return ESP_OK;
        }
        if (esp_timer_get_time() >= dl) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(SYNC_POLL_MS));
    }
}

// ---- bring-up / teardown ----

static void teardown(void)
{
    if (s_isr_live) {
        gpio_isr_handler_remove(GW_SPI_DRDY_GPIO);
        s_isr_live = false;
    }
    if (s_dev) {
        spi_bus_remove_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus_live) {
        spi_bus_free(GW_LINK_SPI_HOST);
        s_bus_live = false;
    }
    s_up = false;
    s_rhead = s_rtail = s_rcount = 0;
    s_nfly = 0;
}

static esp_err_t hw_init(void)
{
    if (!s_txf) {
        s_txf  = heap_caps_malloc(GWSPI_FRAME_MAX, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        s_rxf  = heap_caps_malloc(GWSPI_FRAME_MAX, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        s_ring = heap_caps_malloc(RX_RING_SZ, MALLOC_CAP_INTERNAL);
        if (!s_txf || !s_rxf || !s_ring) return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus = {
        .mosi_io_num     = GW_SPI_MOSI_GPIO,
        .miso_io_num     = GW_SPI_MISO_GPIO,
        .sclk_io_num     = GW_SPI_SCLK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = GWSPI_FRAME_MAX,
    };
    esp_err_t e = spi_bus_initialize(GW_LINK_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e == ESP_OK) s_bus_live = true;
    else if (e != ESP_ERR_INVALID_STATE) return e;   // left over from a partial teardown

    spi_device_interface_config_t dev = {
        .mode           = 1,             // see app_config.h
        .clock_speed_hz = GW_SPI_CLK_HZ,
        .spics_io_num   = GW_SPI_CS_GPIO,
        .queue_size     = 1,
        .input_delay_ns = 30,            // MISO round-trip delay
    };
    e = spi_bus_add_device(GW_LINK_SPI_HOST, &dev, &s_dev);
    if (e != ESP_OK) { teardown(); return e; }

    gpio_config_t drdy = {
        .pin_bit_mask = 1ULL << GW_SPI_DRDY_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // idles low with the peer unpowered
        .intr_type    = GPIO_INTR_ANYEDGE,     // edges only wake; the level decides
    };
    gpio_config(&drdy);
    e = gpio_install_isr_service(0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) { teardown(); return e; }
    gpio_isr_handler_add(GW_SPI_DRDY_GPIO, drdy_isr, NULL);
    s_isr_live = true;

    s_cs_up_us = esp_timer_get_time();
    return ESP_OK;
}

esp_err_t link_up(void)
{
    verb_enter();
    if (s_up) { verb_exit(); return ESP_OK; }
    esp_err_t e = hw_init();
    if (e == ESP_OK) {
        e = link_sync();
        if (e != ESP_OK) teardown();
    }
    s_up = (e == ESP_OK);
    verb_exit();
    return e;
}

void link_down(void)
{
    verb_enter();
    teardown();
    verb_exit();
}

bool link_is_up(void) { return s_up; }

void link_peer_reset(void)
{
    // Open-drain: never drive RUN high.
    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << GW_RESET_GPIO,
        .mode         = GPIO_MODE_OUTPUT_OD,
    };
    gpio_config(&rst);
    gpio_set_level(GW_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(GW_RESET_LOW_MS));
    gpio_set_level(GW_RESET_GPIO, 1);              // high-Z via OD
    vTaskDelay(pdMS_TO_TICKS(GW_BOOT_SETTLE_MS));
}

// ---- data verbs ----

int link_read(uint8_t *dst, size_t want, int to_ms)
{
    verb_enter();
    if (!s_up) { verb_exit(); return -1; }
    int64_t dl = esp_timer_get_time() + (int64_t)to_ms * 1000;
    size_t got = ring_pop(dst, want);
    uint32_t backoff_ms = 1;
    while (got < want) {
        int64_t now = esp_timer_get_time();
        if (now >= dl) break;            // a short read at the deadline is normal
        // Nothing advertised and DRDY low: sleep until the DRDY edge rather
        // than polling, which would interrupt the slave's flux engine.
        if (s_peer_avail == 0 && !gpio_get_level(GW_SPI_DRDY_GPIO)) {
            int64_t rem_ms = (dl - now + 999) / 1000;
            TickType_t tk = pdMS_TO_TICKS(rem_ms > DRDY_BACKSTOP_MS ? DRDY_BACKSTOP_MS
                                                                   : rem_ms);
            ulTaskNotifyTake(pdTRUE, tk ? tk : 1);
        }
        rxinfo_t ri;
        if (do_exchange(NULL, 0, poll_seq(), 0, false, &ri) == ESP_OK && ri.hdr_ok) {
            backoff_ms = 1;
            got += ring_pop(dst + got, want - got);
        } else {
            // Transient: back off and retry within the deadline.
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = backoff_ms * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : backoff_ms * 2;
        }
    }
    verb_exit();
    return (int)got;
}

int link_write(const uint8_t *src, size_t len)
{
    verb_enter();
    if (!s_up) { verb_exit(); return -1; }
    if (len == 0) { verb_exit(); return 0; }

    // 3 s covers the slave's longest stall (a ~2.2 s flux op), plus wire time.
    int64_t dl = esp_timer_get_time() + ((int64_t)3000 + (int64_t)len / 100) * 1000;

    size_t next = 0;                     // next never-sent source byte
    int    resend = 0;                   // s_fly[0,resend) sent since the last
                                         // rewind; the rest await replay
    s_wacked = 0;
    uint32_t backoff_ms = 1;
    int no_news = 0;                     // valid frames since the window last moved

    while (s_wacked < len) {
        if (esp_timer_get_time() >= dl) {
            // Link down: every verb fails fast until gw_relink().
            ESP_LOGE(TAG, "link_write deadline: %u/%u acked — link down (%s)",
                     (unsigned)s_wacked, (unsigned)len, link_stats_str());
            s_nfly = 0;
            s_up = false;
            verb_exit();
            return -2;
        }

        const uint8_t *pl = NULL;
        uint16_t plen = 0;
        uint8_t  seq  = poll_seq();
        size_t   tail = next;            // source bytes queued after this frame
        if (resend < s_nfly) {
            // Replay the recorded slice exactly; it was credited when first cut.
            slice_t *w = &s_fly[resend++];
            pl   = src + w->off;
            plen = w->len;
            seq  = w->seq;
            tail = w->off + w->len;
            s_st.retransmits++;
        } else if (s_nfly < 2 && next < len) {
            size_t sl = len - next;
            if (sl > GWSPI_PAYLOAD_MAX) sl = GWSPI_PAYLOAD_MAX;
            // The slave's credit predates our unacked bytes (its frames are
            // prebuilt), so subtract them or we'd overrun its ring.
            size_t infl = next - s_wacked;
            size_t cred = s_peer_credit > infl ? s_peer_credit - infl : 0;
            if (sl > cred) sl = cred;
            if (sl == 0) s_st.credit_stalls++;                 // poll until credit returns
            else {
                pl = src + next; plen = (uint16_t)sl; seq = s_tx_seq;
                s_fly[s_nfly++] = (slice_t){ .seq = seq, .off = next, .len = plen };
                resend = s_nfly;
                next += plen;
                tail  = next;
                s_tx_seq++;
            }
        }
        size_t rem = len - tail;                               // queued after this frame
        rxinfo_t ri;
        esp_err_t e = do_exchange(pl, plen, seq,
                                  rem > 0xFFFF ? 0xFFFF : (uint16_t)rem, false, &ri);
        // Acked slices left the front of s_fly; rebase the resend cursor.
        if (ri.popped >= resend) resend = 0;
        else                     resend -= ri.popped;
        if (e == ESP_OK && ri.hdr_ok) {
            if (ri.popped) { no_news = 0; backoff_ms = 1; }
            else if (s_nfly && !plen && ++no_news >= 2) {
                // Acks lag one transaction, so two with no progress means a
                // lost slice: go back N.
                resend  = 0;
                no_news = 0;
            }
        } else {
            if (s_nfly) resend = 0;
            no_news = 0;
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = backoff_ms * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : backoff_ms * 2;
        }
    }
    verb_exit();
    return (int)len;
}

bool link_wait_tx_done(int to_ms)
{
    // link_write already waited for the peer's ack.
    (void)to_ms;
    return true;
}

int link_probe_peer_usb(void)
{
    verb_enter();
    if (!s_up) { verb_exit(); return -1; }
    // The first reply was prebuilt before this call and may be stale; the
    // second is current.
    rxinfo_t ri = {0};
    int r = -1;
    for (int i = 0; i < 2; i++) {
        if (do_exchange(NULL, 0, poll_seq(), 0, false, &ri) == ESP_OK && ri.hdr_ok)
            r = ri.usb ? 1 : 0;
        else
            r = -1;
    }
    verb_exit();
    return r;
}

void link_flush_rx(void)
{
    verb_enter();
    s_rhead = s_rtail = s_rcount = 0;
    verb_exit();
}

const char *link_stats_str(void)
{
    static char b[176];
    snprintf(b, sizeof b,
             "ex=%u tx=%u rx=%u crc=%u/%u rt=%u dup=%u seq=%u sync=%u drdy=%u cst=%u",
             (unsigned)s_st.exchanges, (unsigned)s_st.bytes_tx, (unsigned)s_st.bytes_rx,
             (unsigned)s_st.hcrc_fail, (unsigned)s_st.pcrc_fail,
             (unsigned)s_st.retransmits, (unsigned)s_st.dups, (unsigned)s_st.seq_err,
             (unsigned)s_st.sync_resets, (unsigned)s_st.drdy_wakes,
             (unsigned)s_st.credit_stalls);
    return b;
}

// ---- loopback self-test ----------------------------------------------------
// Jumper MISO to MOSI with no peer, build with -DGW_LINK_LOOPBACK_TEST=1 and
// call link_loopback_test() instead of link_up() (extern it at the call site).
#ifndef GW_LINK_LOOPBACK_TEST
#define GW_LINK_LOOPBACK_TEST 0
#endif
#if GW_LINK_LOOPBACK_TEST

esp_err_t link_loopback_test(void)
{
    esp_err_t e = hw_init();
    if (e != ESP_OK) return e;
    uint8_t *pat = heap_caps_malloc(GWSPI_PAYLOAD_MAX, MALLOC_CAP_INTERNAL);
    uint8_t *fr  = heap_caps_malloc(GWSPI_FRAME_MAX, MALLOC_CAP_INTERNAL);
    if (!pat || !fr) { e = ESP_ERR_NO_MEM; goto out; }
    for (size_t i = 0; i < GWSPI_PAYLOAD_MAX; i++) pat[i] = (uint8_t)(i * 7 + 1);

    // 1) Echo across the size range. s_peer_avail stays 0.
    static const uint16_t plens[] = { 0, 1, 7, 100, 2048, GWSPI_PAYLOAD_MAX };
    for (size_t k = 0; k < sizeof plens / sizeof plens[0]; k++) {
        uint16_t plen = plens[k];
        size_t T = GWSPI_HDR_LEN + plen + (plen ? 2u : 0u);
        T = (T + 3u) & ~(size_t)3u;
        rxinfo_t ri;
        e = do_exchange(plen ? pat : NULL, plen, (uint8_t)k, 0, false, &ri);
        if (e != ESP_OK) goto out;
        if (memcmp(s_txf, s_rxf, T) != 0) {
            ESP_LOGE(TAG, "loopback echo mismatch at plen=%u", (unsigned)plen);
            e = ESP_FAIL; goto out;
        }
    }

    // 2) Parse a crafted slave frame, then the same frame again as a duplicate.
    s_rx_last = 0xFF; s_nfly = 0; s_rhead = s_rtail = s_rcount = 0;
    const uint16_t n = 1234;
    memset(fr, 0xEE, GWSPI_FRAME_MAX);
    fr[GWSPI_OFF_MAGIC] = GWSPI_MAGIC_S2M; fr[GWSPI_OFF_FLAGS] = GWSPI_VERSION;
    fr[GWSPI_OFF_SEQ] = 0;
    fr[GWSPI_OFF_ACK] = 0xFF;
    put16(fr + GWSPI_OFF_LEN, n); put16(fr + GWSPI_OFF_AVAIL, 42);
    put16(fr + GWSPI_OFF_CREDIT, 4096);
    put16(fr + GWSPI_OFF_HCRC, crc16_ccitt_tab(0xFFFF, fr, GWSPI_OFF_HCRC));
    memcpy(fr + GWSPI_HDR_LEN, pat, n);
    put16(fr + GWSPI_HDR_LEN + n, crc16_ccitt_tab(0xFFFF, pat, n));
    rxinfo_t ri;
    memset(&ri, 0, sizeof ri);
    parse_frame(fr, GWSPI_FRAME_MAX, &ri);
    uint8_t back[2048];
    if (!ri.hdr_ok || s_rcount != n || ring_pop(back, n) != n ||
        memcmp(back, pat, n) != 0 || s_peer_avail != 42) {
        ESP_LOGE(TAG, "codec/ring ingest failed");
        e = ESP_FAIL; goto out;
    }
    uint32_t dups_before = s_st.dups;
    memset(&ri, 0, sizeof ri);
    parse_frame(fr, GWSPI_FRAME_MAX, &ri);
    if (!ri.hdr_ok || s_rcount != 0 || s_st.dups != dups_before + 1) {
        ESP_LOGE(TAG, "duplicate handling failed");
        e = ESP_FAIL; goto out;
    }
    ESP_LOGI(TAG, "loopback self-test PASS (%s)", link_stats_str());
    e = ESP_OK;
out:
    free(pat); free(fr);
    teardown();
    return e;
}
#endif // GW_LINK_LOOPBACK_TEST
