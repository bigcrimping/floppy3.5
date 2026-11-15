// Grammar (as the host tool's _decode_flux), for byte i:
//   0          end of stream
//   1..249     interval of i ticks
//   250..254   interval of 250 + (i-250)*255 + next - 1 ticks
//   255        opcode: FLUXOP_INDEX u28 (index marker) or FLUXOP_SPACE u28
//              (ticks with no transition); anything else is rejected
// A u28 is 4 bytes of 7 bits each in bits 1..7 (bit 0 is always set).
#include "flux_decode.h"
#include "gw_proto.h"

static uint32_t read_28bit(const uint8_t **p, const uint8_t *end)
{
    uint32_t v = 0;
    for (int i = 0; i < 4 && *p < end; i++) {
        uint8_t b = *(*p)++;
        int s = i * 7 - 1;
        v += (s < 0) ? ((b & 0xFE) >> 1) : ((uint32_t)(b & 0xFE) << s);
    }
    return v;
}

int flux_decode(const uint8_t *raw, size_t raw_len, flux_stream_t *fs)
{
    fs->ticks_len = 0;
    fs->index_len = 0;
    if (raw_len == 0 || raw[raw_len - 1] != 0) {
        return -1; // stream must be 0-terminated
    }

    const uint8_t *p   = raw;
    const uint8_t *end = raw + raw_len - 1; // exclude terminator
    uint32_t ticks = 0;                     // accumulator for SPACE + interval
    uint32_t abs_pos = 0;                   // cumulative ticks since stream start

    while (p < end) {
        uint8_t i = *p++;

        if (i == 0) {
            break;                          // unexpected early terminator; stop
        } else if (i < 250) {
            ticks += i;
        } else if (i < 255) {
            if (p >= end) return -2;
            ticks += 250 + (uint32_t)(i - 250) * 255 + (*p++) - 1;
        } else { // i == 255: opcode escape
            if (p >= end) return -2;
            uint8_t op = *p++;
            if (op == FLUXOP_INDEX) {
                // Doesn't consume the pending ticks. Extra markers (a bouncing
                // INDEX line) are dropped rather than failing the stream.
                uint32_t v = read_28bit(&p, end);
                if (fs->index_len < fs->index_cap)
                    fs->index[fs->index_len++] = abs_pos + ticks + v;
                continue;
            } else if (op == FLUXOP_SPACE) {
                ticks += read_28bit(&p, end);
                continue;
            } else {
                return -4;                  // unknown opcode
            }
        }

        // Emit a transition of `ticks` length.
        abs_pos += ticks;
        if (fs->ticks_len >= fs->ticks_cap) return -5;
        fs->ticks[fs->ticks_len++] = ticks;
        ticks = 0;
    }
    return 0;
}
