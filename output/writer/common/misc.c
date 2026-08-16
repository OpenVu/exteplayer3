/*
 * LinuxDVB Output handling.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

/* ***************************** */
/* Includes                      */
/* ***************************** */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/dvb/video.h>
#include <linux/dvb/audio.h>

#include <memory.h>
#include <asm/types.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>

#include "stm_ioctls.h"
#include "bcm_ioctls.h"

#include "common.h"
#include "output.h"
#include "debug.h"
#include "misc.h"

/* ***************************** */
/* Makros/Constants              */
/* ***************************** */

/* ***************************** */
/* Types                         */
/* ***************************** */

/* ***************************** */
/* Varaibles                     */
/* ***************************** */

/* ***************************** */
/* Prototypes                    */
/* ***************************** */

/* ***************************** */
/* MISC Functions                */
/* ***************************** */

void PutBits(BitPacker_t * ld, unsigned int code, unsigned int length)
{
    unsigned int bit_buf;
    unsigned int bit_left;

    bit_buf = ld->BitBuffer;
    bit_left = ld->Remaining;

#ifdef DEBUG_PUTBITS
    if (ld->debug)
        dprintf("code = %d, length = %d, bit_buf = 0x%x, bit_left = %d\n", code, length, bit_buf, bit_left);
#endif /* DEBUG_PUTBITS */

    if (length < bit_left)
    {
        /* fits into current buffer */
        bit_buf = (bit_buf << length) | code;
        bit_left -= length;
    }
    else
    {
        /* doesn't fit */
        bit_buf <<= bit_left;
        bit_buf |= code >> (length - bit_left);
        ld->Ptr[0] = (char)(bit_buf >> 24);
        ld->Ptr[1] = (char)(bit_buf >> 16);
        ld->Ptr[2] = (char)(bit_buf >> 8);
        ld->Ptr[3] = (char)bit_buf;
        ld->Ptr   += 4;
        length    -= bit_left;
        bit_buf    = code & ((1 << length) - 1);
        bit_left   = 32 - length;
        bit_buf = code;
    }

#ifdef DEBUG_PUTBITS
    if (ld->debug)
        dprintf("bit_left = %d, bit_buf = 0x%x\n", bit_left, bit_buf);
#endif /* DEBUG_PUTBITS */

    /* writeback */
    ld->BitBuffer = bit_buf;
    ld->Remaining = bit_left;
}

void FlushBits(BitPacker_t * ld)
{
    ld->BitBuffer <<= ld->Remaining;
    while (ld->Remaining < 32)
    {
#ifdef DEBUG_PUTBITS
        if (ld->debug)
            dprintf("flushing 0x%2.2x\n", ld->BitBuffer >> 24);
#endif /* DEBUG_PUTBITS */
        *ld->Ptr++ = ld->BitBuffer >> 24;
        ld->BitBuffer <<= 8;
        ld->Remaining += 8;
    }
    ld->Remaining = 32;
    ld->BitBuffer = 0;
}

/* Read first line of a proc/sys file into buf (lowercase). Return true on success. */
static bool ReadFirstLineLower(const char *path, char *buf, size_t bufSize)
{
    FILE *f = fopen(path, "r");
    size_t i = 0;

    if (NULL == f)
    {
        return false;
    }

    if (NULL == fgets(buf, (int)bufSize, f))
    {
        fclose(f);
        return false;
    }
    fclose(f);

    for (i = 0; buf[i] != '\0'; ++i)
    {
        if (buf[i] == '\n' || buf[i] == '\r')
        {
            buf[i] = '\0';
            break;
        }
        if (buf[i] >= 'A' && buf[i] <= 'Z')
        {
            buf[i] = (char)(buf[i] - 'A' + 'a');
        }
    }

    return true;
}

/* HiSilicon based E2 receivers (Qviart Dual/Lunix 4K, Zgemma H9/H11, Octagon SX8x,
 * Edision OS Mio+ 4K, ...) do not always expose the device tree node used before.
 * Detect them by every marker known so far, otherwise all HiSilicon specific
 * work-arounds in this player stay disabled and playback misbehaves.
 */
static bool IsHiSiliconSTB()
{
    static const char *markers[] = {
        "/sys/firmware/devicetree/base/soc/hisilicon_clock/name",
        "/proc/hisi",
        "/dev/hi_mem",
        "/dev/hi_dmx",
        "/sys/module/hi_media",
        NULL
    };
    char buf[128];
    int i = 0;

    for (i = 0; markers[i] != NULL; ++i)
    {
        if (access(markers[i], F_OK) != -1)
        {
            return true;
        }
    }

    /* /proc/stb/info/chipset reports e.g. "hi3798mv200" / "Hi3798MV200" */
    if (ReadFirstLineLower("/proc/stb/info/chipset", buf, sizeof(buf)))
    {
        if (0 == strncmp(buf, "hi3", 3) || NULL != strstr(buf, "hisi"))
        {
            return true;
        }
    }

    return false;
}

/* *********************************************************************** */
/* Output pacing (software playback clock)                                 */
/*                                                                         */
/* exteplayer3 has no pacing of its own - it pushes ES data into           */
/* /dev/dvb/adapter0/{video0,audio0} as fast as the driver accepts it and  */
/* relies on the decoder to block write()/select() once its input FIFO is  */
/* full. On Broadcom/STi boxes that back-pressure exists, so playback runs */
/* at real time.                                                           */
/*                                                                         */
/* On HiSilicon based receivers (Qviart Dual/Lunix 4K, Zgemma H9, ...) the */
/* linuxdvb devices are only an emulation on top of the HiSilicon media    */
/* API. They swallow everything that is written without ever reporting     */
/* "buffer full", so the whole stream is decoded as fast as it can be      */
/* downloaded - picture and sound run too fast. gstplayer does not show    */
/* this because dvbvideosink/dvbaudiosink are GstBaseSinks which wait on   */
/* the GStreamer clock before pushing each buffer.                         */
/*                                                                         */
/* The code below adds exactly that missing piece: injection is throttled  */
/* against a monotonic clock anchored to the stream PTS, so we never write */
/* more than <lead> ms of media ahead of real time.                        */
/*                                                                         */
/* IMPORTANT: this must throttle the *device write*, never the demuxer     */
/* read. When the ffmpeg read thread is the one being slept, there is no   */
/* read ahead left at all and every network hiccup starves the decoder     */
/* (visible as short picture corruption). Therefore, whenever the RAM      */
/* buffering output is active, the wait happens in the buffering worker    */
/* threads and ffmpeg keeps filling the RAM queues at full speed.          */
/*                                                                         */
/* Every elementary stream owns its own clock, exactly like every          */
/* GstBaseSink does in the gstplayer pipeline. The audio writer and the    */
/* video writer run in separate threads, so a single shared anchor would   */
/* need locking and - much worse - a PTS discontinuity in one stream would */
/* wreck the pacing of the other one. Both anchors are taken at (nearly)   */
/* the same wall clock instant when playback starts, so the two streams    */
/* advance at the same rate and stay aligned. The fine grained A/V sync is */
/* done by the decoder from the PTS in the PES header anyway.              */
/* *********************************************************************** */

typedef struct
{
    int64_t wall_us;    /* monotonic clock at the anchor            */
    int64_t pts;        /* stream PTS (90 kHz) at the anchor        */
    int64_t last_pts;   /* PTS of the previous chunk of this stream */
} PacingClock_t;

static int32_t g_pacing_lead_ms = -1;   /* <0 auto, 0 disabled, >0 lead in ms */

static PacingClock_t g_pacing[PACING_STREAM_MAX] =
{
    { -1, -1, -1 },
    { -1, -1, -1 },
};

void OutputPacingSet(int32_t leadMs)
{
    g_pacing_lead_ms = leadMs;
}

void OutputPacingReset(void)
{
    int32_t i = 0;
    for (i = 0; i < PACING_STREAM_MAX; ++i)
    {
        g_pacing[i].wall_us  = -1;
        g_pacing[i].pts      = -1;
        g_pacing[i].last_pts = -1;
    }
}

static int64_t monotonic_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)ts.tv_nsec / 1000LL;
}

int32_t OutputPacingLeadMs(void)
{
    if (g_pacing_lead_ms < 0)
    {
        /* auto: only needed where the driver gives no write back-pressure */
        g_pacing_lead_ms = (STB_HISILICON == GetSTBType()) ? 1500 : 0;
    }
    return g_pacing_lead_ms;
}

void OutputPacingWait(void *_context, int32_t stream, int64_t pts)
{
    Context_t *context = (Context_t *)_context;
    int32_t leadMs = OutputPacingLeadMs();
    PacingClock_t *clk = NULL;
    int64_t now, dpts, jump, target, ahead, maxAhead;

    if (leadMs <= 0 || pts < 0 || pts == (int64_t)INVALID_PTS_VALUE || NULL == context)
    {
        return;
    }

    if (stream < 0 || stream >= PACING_STREAM_MAX)
    {
        stream = PACING_STREAM_VIDEO;
    }

    clk      = &g_pacing[stream];
    maxAhead = (int64_t)leadMs * 1000LL;

    /* freeze our reference clock while the user keeps the playback paused,
     * otherwise we would burst the accumulated delay right after resume */
    if (context->playback->isPaused && clk->wall_us >= 0)
    {
        int64_t pauseStart = monotonic_us();
        while (context->playback->isPaused &&
               context->playback->isPlaying &&
               !context->playback->isSeeking &&
               0 == PlaybackDieNow(0))
        {
            usleep(10000);
        }
        clk->wall_us += monotonic_us() - pauseStart;
    }

    now = monotonic_us();

    if (clk->pts < 0)
    {
        /* First chunk of this stream (start of playback, or after a seek which
         * called OutputPacingReset()): anchor here. Anchoring at "now" is what
         * lets the decoder be pre-filled with <lead> ms in one burst, which is
         * exactly what we want at that point.
         */
        clk->pts      = pts;
        clk->wall_us  = now;
        clk->last_pts = pts;
        return;
    }

    jump          = pts - clk->last_pts;
    clk->last_pts = pts;

    /* A real discontinuity (new segment, 33 bit PTS wrap around, ...) is a
     * jump relative to the PREVIOUS chunk of this very stream.
     *
     * The old code compared against the anchor instead, which is unavoidably
     * true again once the stream has been running longer than the threshold.
     * The clock was therefore re-anchored once per minute, and because each
     * re-anchor also handed out a fresh <lead> ms head start, playback drifted
     * ~2.6% too fast and the lead grew without any bound.
     */
    if (jump < -90000LL || jump > 90000LL * 10LL)
    {
        /* Re-anchor while KEEPING the lead we already have. This chunk is
         * written now, and by definition "now" has to map to
         * target(pts) - maxAhead, hence wall = now + maxAhead.
         */
        clk->pts     = pts;
        clk->wall_us = now + maxAhead;
        return;
    }

    dpts   = pts - clk->pts;
    target = clk->wall_us + (dpts * 1000LL) / 90LL;  /* us */
    ahead  = target - now;

    while (ahead > maxAhead)
    {
        int64_t sleep_us = ahead - maxAhead;

        if (!context->playback->isPlaying ||
            context->playback->isSeeking ||
            0 != PlaybackDieNow(0))
        {
            break;
        }

        /* sleep in slices so stop/seek stay responsive */
        if (sleep_us > 50000LL)
        {
            sleep_us = 50000LL;
        }
        usleep((useconds_t)sleep_us);

        ahead = target - monotonic_us();
    }
}

stb_type_t GetSTBType()
{
    static stb_type_t type = STB_UNKNOWN;
    if (type == STB_UNKNOWN) {
        if (access("/proc/stb/tpm/0/serial", F_OK) != -1) {
            type = STB_DREAMBOX;
        }
        else if (access("/proc/stb/info/vumodel", F_OK) != -1 && \
                 access("/proc/stb/info/boxtype", F_OK) == -1 ) {
            // some STB like Octagon SF4008 has also /proc/stb/info/vumodel
            // but VU PLUS does not have /proc/stb/info/boxtype
            // please see: https://gitlab.com/e2i/e2iplayer/issues/282
            type = STB_VUPLUS;
        }
        else if (IsHiSiliconSTB()) {
            type = STB_HISILICON;
        }
        else {
            type = STB_OTHER;
        }
    }

    return type;
}
