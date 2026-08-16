/*
 * RAM write buffering utilities
 * samsamsam 2018
 *
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
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <time.h>

#include "common.h"
#include "debug.h"
#include "misc.h"
#include "writer.h"

/* ***************************** */
/* Types                         */
/* ***************************** */
typedef enum OutputType_e{
    OUTPUT_UNK,
    OUTPUT_AUDIO,
    OUTPUT_VIDEO,
} OutputType_t;

typedef struct BufferingNode_s {
    uint32_t dataSize;
    OutputType_t dataType;
    void *stamp;
    int64_t pts;
    struct BufferingNode_s *next;
} BufferingNode_t;

/* One independent queue + writer thread per elementary stream.
 *
 * Before, audio and video shared a single FIFO served by a single thread, so
 * every delay on the video device (decoder input full, a slow write, the
 * software pacing wait) also held back all audio chunks queued behind it -
 * which is exactly what made the sound break up, while gstplayer, where
 * dvbvideosink and dvbaudiosink each own their streaming thread, stayed clean.
 * Now each stream is injected completely independently, just like there.
 */
typedef struct StreamQueue_s {
    const char      *name;
    OutputType_t     dataType;
    int32_t          pacingStream;

    Context_t       *context;
    int              fd;

    pthread_t        thread;
    bool             threadStarted;
    int              pfd[2];              /* wake up pipe for this thread */

    pthread_mutex_t  mtx;
    pthread_cond_t   exitCond;
    pthread_cond_t   dataConsumedCond;
    pthread_cond_t   writeFinishedCond;
    pthread_cond_t   dataAddedCond;

    BufferingNode_t *head;
    BufferingNode_t *tail;

    uint32_t         dataSize;
    uint32_t         maxDataSize;

    bool             duringWrite;
    bool             signalWriteFinish;
} StreamQueue_t;

/* ***************************** */
/* Makros/Constants              */
/* ***************************** */
#define cERR_LINUX_DVB_BUFFERING_NO_ERROR      0
#define cERR_LINUX_DVB_BUFFERING_ERROR        -1

/* The video elementary stream carries almost all of the bit rate, so it gets
 * the lion's share of the configured RAM buffer. The audio share still covers
 * many seconds of playback because of its much lower bit rate.
 */
#define VIDEO_BUFFER_NUMERATOR    7
#define VIDEO_BUFFER_DENOMINATOR  8
#define MIN_QUEUE_SIZE            (256 * 1024)

/* ***************************** */
/* Varaibles                     */
/* ***************************** */
static StreamQueue_t g_videoQueue = {
    .name = "video", .dataType = OUTPUT_VIDEO, .pacingStream = PACING_STREAM_VIDEO,
    .context = NULL, .fd = -1, .threadStarted = false, .pfd = {-1, -1},
};

static StreamQueue_t g_audioQueue = {
    .name = "audio", .dataType = OUTPUT_AUDIO, .pacingStream = PACING_STREAM_AUDIO,
    .context = NULL, .fd = -1, .threadStarted = false, .pfd = {-1, -1},
};

static uint32_t maxBufferingDataSize = 0;

static pthread_mutex_t *g_pDVBMtx = NULL;

static void *g_pWriteStamp = NULL;
static int64_t g_writePts = -1;

/* ***************************** */
/* Prototypes                    */
/* ***************************** */

/* ***************************** */
/* MISC Functions                */
/* ***************************** */
static void QueueWakeUp(StreamQueue_t *q)
{
    if (q->pfd[1] >= 0)
    {
        int ret = write(q->pfd[1], "x", 1);
        if (ret != 1) {
            buff_printf(20, "WriteWakeUp(%s) write return %d\n", q->name, ret);
        }
    }
}

/* Registered as PlaybackDieNow callback - has to wake up every writer thread */
static void WriteWakeUp()
{
    QueueWakeUp(&g_videoQueue);
    QueueWakeUp(&g_audioQueue);
}

static void QueueDropAll(StreamQueue_t *q)
{
    while (q->head)
    {
        BufferingNode_t *nodePtr = q->head;
        q->head = nodePtr->next;
        free(nodePtr);
    }
    q->head = NULL;
    q->tail = NULL;
    q->dataSize = 0;
}

/* **************************** */
/* Worker Thread                */
/* **************************** */
static void LinuxDvbBuffThread(StreamQueue_t *q)
{
    int flags = 0;
    /* MUST NOT be static: one instance of this thread runs per stream */
    BufferingNode_t *nodePtr = NULL;
    Context_t *context = q->context;

    buff_printf(20, "ENTER [%s]\n", q->name);

    if (pipe(q->pfd) == -1)
        buff_err("critical error\n");

    /* Make read and write ends of pipe nonblocking */
    if ((flags = fcntl(q->pfd[0], F_GETFL)) == -1)
        buff_err("critical error\n");

    /* Make read end nonblocking */
    flags |= O_NONBLOCK;
    if (fcntl(q->pfd[0], F_SETFL, flags) == -1)
        buff_err("critical error\n");

    if ((flags = fcntl(q->pfd[1], F_GETFL)) == -1)
        buff_err("critical error\n");

    /* Make write end nonblocking */
    flags |= O_NONBLOCK;
    if (fcntl(q->pfd[1], F_SETFL, flags) == -1)
        buff_err("critical error\n");

    PlaybackDieNowRegisterCallback(WriteWakeUp);

    while (0 == PlaybackDieNow(0))
    {
        pthread_mutex_lock(&q->mtx);
        q->duringWrite = false;
        if (q->signalWriteFinish)
        {
            pthread_cond_signal(&q->writeFinishedCond);
            q->signalWriteFinish = false;
        }

        if (nodePtr)
        {
            free(nodePtr);
            nodePtr = NULL;
            /* signal that we free some space in queue */
            pthread_cond_signal(&q->dataConsumedCond);
        }

        if (!q->head)
        {
            assert(q->tail == NULL);

            /* Queue is empty we need to wait for data to be added */
            pthread_cond_wait(&q->dataAddedCond, &q->mtx);
            pthread_mutex_unlock(&q->mtx);
            continue; /* To check PlaybackDieNow(0) */
        }
        else
        {
            nodePtr = q->head;
            q->head = q->head->next;
            if (q->head == NULL)
            {
                q->tail = NULL;
            }

            if (q->dataSize >= (nodePtr->dataSize + sizeof(BufferingNode_t)))
            {
                q->dataSize -= (nodePtr->dataSize + sizeof(BufferingNode_t));
            }
            else
            {
                q->dataSize = 0;
            }
        }

        /* We will write data without mutex
         * this have some disadvantage because we can
         * write some portion of data after LinuxDvbBuffFlush,
         * for example after seek.
         */
        if (nodePtr && !context->playback->isSeeking && context->playback->stamp == nodePtr->stamp)
        {
            /* Write data to valid output */
            uint8_t *dataPtr = (uint8_t *)nodePtr + sizeof(BufferingNode_t);
            int fd = q->fd;
            q->duringWrite = true;
            pthread_mutex_unlock(&q->mtx);

            /* Pace the injection here, on the consumer side, so the demuxer
             * thread can keep reading ahead into the RAM queue. On boxes whose
             * DVB driver gives no write back-pressure (HiSilicon) this is what
             * keeps playback at real time. Each stream has its own clock, so
             * waiting here never holds back the other stream.
             */
            OutputPacingWait(context, q->pacingStream, nodePtr->pts);

            if (0 != WriteWithRetry(context, q->pfd[0], fd, g_pDVBMtx, dataPtr, nodePtr->dataSize))
            {
                buff_err("Something is WRONG [%s]\n", q->name);
            }
        }
        else
        {
            pthread_mutex_unlock(&q->mtx);
        }
    }

    pthread_mutex_lock(&q->mtx);
    q->duringWrite = false;
    pthread_cond_signal(&q->writeFinishedCond);
    pthread_cond_signal(&q->exitCond);
    pthread_mutex_unlock(&q->mtx);

    buff_printf(20, "EXIT [%s]\n", q->name);
    q->threadStarted = false;

    close(q->pfd[0]);
    close(q->pfd[1]);
    q->pfd[0] = -1;
    q->pfd[1] = -1;
}

static int32_t QueueStart(StreamQueue_t *q, Context_t *context, int outfd, uint32_t maxSize)
{
    int32_t error = 0;
    pthread_attr_t attr;

    if (q->threadStarted)
    {
        return cERR_LINUX_DVB_BUFFERING_NO_ERROR;
    }

    q->context     = context;
    q->fd          = outfd;
    q->head        = NULL;
    q->tail        = NULL;
    q->dataSize    = 0;
    q->maxDataSize = maxSize;
    q->duringWrite = false;
    q->signalWriteFinish = false;
    q->pfd[0]      = -1;
    q->pfd[1]      = -1;

    /* init the synchronization primitives BEFORE the thread can touch them.
     * They used to be initialised after pthread_create(), which is a race.
     */
    pthread_mutex_init(&q->mtx, NULL);
    pthread_cond_init(&q->exitCond, NULL);
    pthread_cond_init(&q->dataConsumedCond, NULL);
    pthread_cond_init(&q->writeFinishedCond, NULL);
    pthread_cond_init(&q->dataAddedCond, NULL);

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    if ((error = pthread_create(&q->thread, &attr, (void *)&LinuxDvbBuffThread, q)) != 0)
    {
        buff_printf(10, "Creating %s thread, error:%d:%s\n", q->name, error, strerror(error));
        q->threadStarted = false;
        return cERR_LINUX_DVB_BUFFERING_ERROR;
    }

    buff_printf(10, "Created %s thread, queue size %u\n", q->name, maxSize);
    q->threadStarted = true;
    return cERR_LINUX_DVB_BUFFERING_NO_ERROR;
}

static void QueueRequestStop(StreamQueue_t *q)
{
    q->fd = -1;

    if (!q->threadStarted)
    {
        return;
    }

    /* WakeUp if we are waiting in the write */
    QueueWakeUp(q);

    pthread_mutex_lock(&q->mtx);
    /* wake up if thread is waiting for data */
    pthread_cond_signal(&q->dataAddedCond);
    pthread_mutex_unlock(&q->mtx);
}

static void QueueWaitStopped(StreamQueue_t *q, time_t deadline)
{
    struct timespec max_wait = {0, 0};

    if (!q->threadStarted)
    {
        return;
    }

    pthread_mutex_lock(&q->mtx);
    if (q->threadStarted)
    {
        max_wait.tv_sec = deadline;
        pthread_cond_timedwait(&q->exitCond, &q->mtx, &max_wait);
    }
    pthread_mutex_unlock(&q->mtx);
}

static void QueueFlush(StreamQueue_t *q)
{
    if (!q->threadStarted)
    {
        return;
    }

    /* signal if we are waiting for the write to the DVB decoder */
    QueueWakeUp(q);

    pthread_mutex_lock(&q->mtx);
    QueueDropAll(q);
    buff_printf(40, "[%s] bufferingDataSize [%u]\n", q->name, q->dataSize);

    /* signal that queue is empty */
    pthread_cond_signal(&q->dataConsumedCond);

    while (q->duringWrite && !PlaybackDieNow(0))
    {
        q->signalWriteFinish = true;
        pthread_cond_wait(&q->writeFinishedCond, &q->mtx);
    }

    pthread_mutex_unlock(&q->mtx);
}

int32_t LinuxDvbBuffSetSize(const uint32_t bufferSize)
{
    maxBufferingDataSize = bufferSize;
    return cERR_LINUX_DVB_BUFFERING_NO_ERROR;
}

uint32_t LinuxDvbBuffGetSize()
{
    return maxBufferingDataSize;
}

int32_t LinuxDvbBuffOpen(Context_t *context, char *type, int outfd, void *mtx)
{
    int32_t ret = cERR_LINUX_DVB_BUFFERING_NO_ERROR;
    uint32_t videoSize = 0;
    uint32_t audioSize = 0;

    buff_printf(10, "[%s]\n", type);

    g_pDVBMtx = mtx;

    videoSize = (uint32_t)(((uint64_t)maxBufferingDataSize * VIDEO_BUFFER_NUMERATOR) / VIDEO_BUFFER_DENOMINATOR);
    audioSize = maxBufferingDataSize - videoSize;
    if (videoSize < MIN_QUEUE_SIZE) videoSize = MIN_QUEUE_SIZE;
    if (audioSize < MIN_QUEUE_SIZE) audioSize = MIN_QUEUE_SIZE;

    if (!strcmp("video", type))
    {
        ret = QueueStart(&g_videoQueue, context, outfd, videoSize);
    }
    else if (!strcmp("audio", type))
    {
        ret = QueueStart(&g_audioQueue, context, outfd, audioSize);
    }
    else
    {
        ret = cERR_LINUX_DVB_BUFFERING_ERROR;
    }

    buff_printf(10, "exiting with value %d\n", ret);
    return ret;
}

int32_t LinuxDvbBuffClose(Context_t *context)
{
    int32_t ret = 0;
    time_t deadline = time(NULL) + 2;

    buff_printf(10, "\n");

    /* ask both writers to stop first, then wait - so the two timeouts run in
     * parallel instead of adding up */
    QueueRequestStop(&g_videoQueue);
    QueueRequestStop(&g_audioQueue);

    QueueWaitStopped(&g_videoQueue, deadline);
    QueueWaitStopped(&g_audioQueue, deadline);

    ret = (g_videoQueue.threadStarted || g_audioQueue.threadStarted)
          ? cERR_LINUX_DVB_BUFFERING_ERROR : cERR_LINUX_DVB_BUFFERING_NO_ERROR;

    buff_printf(10, "exiting with value %d\n", ret);
    return ret;
}

int32_t LinuxDvbBuffFlush(Context_t *context)
{
    buff_printf(40, "ENTER\n");

    QueueFlush(&g_videoQueue);
    QueueFlush(&g_audioQueue);

    /* queues dropped (seek/flush) -> the playback clocks have to be re-anchored */
    OutputPacingReset();

    buff_printf(40, "EXIT\n");
    return 0;
}

int32_t LinuxDvbBuffResume(Context_t *context)
{
    /* signal if we are waiting for write to DVB decoders */
    WriteWakeUp();

    return 0;
}

void LinuxDvbBuffSetStamp(void *stamp)
{
    g_pWriteStamp = stamp;
}

/* Called by the demuxer thread right before the data is queued, so the
 * buffering worker knows at which stream time the chunk has to be injected.
 */
void LinuxDvbBuffSetPts(int64_t pts)
{
    g_writePts = pts;
}

ssize_t BufferingWriteV(int fd, const struct iovec *iov, int ic)
{
    StreamQueue_t *q = NULL;
    BufferingNode_t *nodePtr = NULL;
    uint8_t *dataPtr = NULL;
    uint32_t chunkSize = 0;
    int32_t i = 0;

    buff_printf(60, "ENTER\n");
    if (g_videoQueue.threadStarted && fd == g_videoQueue.fd)
    {
        buff_printf(60, "VIDEO\n");
        q = &g_videoQueue;
    }
    else if (g_audioQueue.threadStarted && fd == g_audioQueue.fd)
    {
        buff_printf(60, "AUDIO\n");
        q = &g_audioQueue;
    }
    else
    {
        buff_err("Unknown output type\n");
        return cERR_LINUX_DVB_BUFFERING_ERROR;
    }

    for (i=0; i<ic; ++i)
    {
        chunkSize += iov[i].iov_len;
    }
    chunkSize += sizeof(BufferingNode_t);

    /* Allocate memory for queue node + data */
    nodePtr = malloc(chunkSize);
    if (!nodePtr)
    {
        buff_err("OUT OF MEM\n");
        return cERR_LINUX_DVB_BUFFERING_ERROR;
    }

    /* Copy data to new buffer */
    dataPtr = (uint8_t *)nodePtr + sizeof(BufferingNode_t);
    for (i=0; i<ic; ++i)
    {
        memcpy(dataPtr, iov[i].iov_base, iov[i].iov_len);
        dataPtr += iov[i].iov_len;
    }

    pthread_mutex_lock(&q->mtx);
    while (0 == PlaybackDieNow(0))
    {
        if (q->dataSize + chunkSize >= q->maxDataSize)
        {
            /* Buffering queue is full we need wait for space*/
            pthread_cond_wait(&q->dataConsumedCond, &q->mtx);
        }
        else
        {
            /* Add chunk to buffering queue */
            q->dataSize += chunkSize;
            chunkSize   -= sizeof(BufferingNode_t);

            nodePtr->dataSize = chunkSize;
            nodePtr->dataType = q->dataType;
            nodePtr->stamp    = g_pWriteStamp;
            nodePtr->pts      = g_writePts;
            nodePtr->next     = NULL;

            if (q->head == NULL)
            {
                q->head = nodePtr;
                q->tail = nodePtr;
            }
            else
            {
                q->tail->next = nodePtr;
                q->tail = nodePtr;
            }

            /* signal that we added some data to queue */
            pthread_cond_signal(&q->dataAddedCond);
            break;
        }
    }
    pthread_mutex_unlock(&q->mtx);
    buff_printf(60, "EXIT\n");
    return chunkSize;
}
