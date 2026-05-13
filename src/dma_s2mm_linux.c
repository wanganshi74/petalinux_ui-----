#include "dma_s2mm_linux.h"

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "libaxidma.h"

#ifndef AXIDMA_RX_BYTES
#define AXIDMA_RX_BYTES DMA_S2MM_FRAME_BYTES
#endif

#define AXIDMA_BUF_COUNT 4U

typedef struct {
    pthread_t worker;
    pthread_mutex_t lock;
    int lock_inited;
    int thread_started;
    int stop_requested;
    int in_transfer;
    unsigned rx_index;
    int rx_channel;
    size_t transfer_size;
    void *buffers[AXIDMA_BUF_COUNT];
    uint64_t latest_frame[DMA_S2MM_FRAME_WORDS];
    unsigned long long completed_count;
    unsigned long long error_count;
    uint64_t last_complete_ms;
    uint64_t last_warn_ms;
    int latest_valid;
    const char *tag;
} dma_state_t;

static pthread_mutex_t g_axidma_lock = PTHREAD_MUTEX_INITIALIZER;
static axidma_dev_t g_axidma_dev = NULL;
static unsigned g_axidma_users = 0U;

static dma_state_t g_dma_main = {
    .rx_channel = -1,
    .rx_index = 0U,
    .tag = "dma_main"
};

static dma_state_t g_dma_scope = {
    .rx_channel = -1,
    .rx_index = 1U,
    .tag = "dma_scope"
};

static uint64_t monotonic_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void dma_note_success(dma_state_t *dma, const void *buf, size_t len)
{
    pthread_mutex_lock(&dma->lock);
    if(buf != NULL && len >= DMA_S2MM_FRAME_BYTES) {
        memcpy(dma->latest_frame, buf, DMA_S2MM_FRAME_BYTES);
        dma->latest_valid = 1;
    }
    dma->completed_count++;
    dma->last_complete_ms = monotonic_ms();
    pthread_mutex_unlock(&dma->lock);
}

static void dma_note_error(dma_state_t *dma)
{
    pthread_mutex_lock(&dma->lock);
    dma->error_count++;
    pthread_mutex_unlock(&dma->lock);
}

static void *dma_worker_thread(void *arg)
{
    dma_state_t *dma = (dma_state_t *)arg;
    axidma_dev_t dev;
    unsigned index = 0U;

    pthread_mutex_lock(&g_axidma_lock);
    dev = g_axidma_dev;
    pthread_mutex_unlock(&g_axidma_lock);

    if(dev == NULL) {
        return NULL;
    }

    while(!dma->stop_requested) {
        void *buf = dma->buffers[index];
        int rc;

        pthread_mutex_lock(&dma->lock);
        dma->in_transfer = 1;
        pthread_mutex_unlock(&dma->lock);

        rc = axidma_oneway_transfer(dev, dma->rx_channel, buf, dma->transfer_size, true);

        pthread_mutex_lock(&dma->lock);
        dma->in_transfer = 0;
        pthread_mutex_unlock(&dma->lock);

        if(dma->stop_requested) {
            break;
        }

        if(rc < 0) {
            dma_note_error(dma);
            axidma_stop_transfer(dev, dma->rx_channel);
            usleep(5000);
            continue;
        }

        dma_note_success(dma, buf, dma->transfer_size);
        index = (index + 1U) % AXIDMA_BUF_COUNT;
    }

    return NULL;
}

static axidma_dev_t dma_shared_acquire(void)
{
    axidma_dev_t dev;

    pthread_mutex_lock(&g_axidma_lock);
    if(g_axidma_dev == NULL) {
        g_axidma_dev = axidma_init();
    }
    if(g_axidma_dev != NULL) {
        g_axidma_users++;
    }
    dev = g_axidma_dev;
    pthread_mutex_unlock(&g_axidma_lock);

    return dev;
}

static void dma_shared_release(void)
{
    pthread_mutex_lock(&g_axidma_lock);
    if(g_axidma_users > 0U) {
        g_axidma_users--;
        if(g_axidma_users == 0U && g_axidma_dev != NULL) {
            axidma_destroy(g_axidma_dev);
            g_axidma_dev = NULL;
        }
    }
    pthread_mutex_unlock(&g_axidma_lock);
}

static int dma_init_state(dma_state_t *dma, unsigned rx_index)
{
    axidma_dev_t dev;
    const array_t *rx_channels;
    unsigned i;

    if(dma->thread_started || dma->rx_channel >= 0) {
        return 0;
    }

    dma->rx_index = rx_index;
    dma->rx_channel = -1;
    dma->transfer_size = AXIDMA_RX_BYTES;
    dma->completed_count = 0ULL;
    dma->error_count = 0ULL;
    dma->last_complete_ms = 0ULL;
    dma->last_warn_ms = 0ULL;
    dma->latest_valid = 0;
    dma->stop_requested = 0;
    dma->in_transfer = 0;

    memset(dma->buffers, 0, sizeof(dma->buffers));
    memset(dma->latest_frame, 0, sizeof(dma->latest_frame));

    if(pthread_mutex_init(&dma->lock, NULL) != 0) {
        fprintf(stderr, "%s: pthread_mutex_init failed\n", dma->tag);
        return -1;
    }
    dma->lock_inited = 1;

    dev = dma_shared_acquire();
    if(dev == NULL) {
        fprintf(stderr, "%s: axidma_init failed, check /dev/axidma and run as root\n", dma->tag);
        goto err_release_lock;
    }

    rx_channels = axidma_get_dma_rx(dev);
    if(rx_channels == NULL || rx_channels->len < 1) {
        fprintf(stderr, "%s: no AXI DMA RX channel found\n", dma->tag);
        goto err_release_dev;
    }

    fprintf(stderr, "%s: found %d RX channel(s):", dma->tag, rx_channels->len);
    for(i = 0; i < (unsigned)rx_channels->len; ++i) {
        fprintf(stderr, " %d", rx_channels->data[i]);
    }
    fprintf(stderr, "\n");

    if(rx_index >= (unsigned)rx_channels->len) {
        fprintf(stderr, "%s: requested RX index %u out of range, using 0\n", dma->tag, rx_index);
        rx_index = 0U;
        dma->rx_index = 0U;
    }

    dma->rx_channel = rx_channels->data[rx_index];
    fprintf(stderr, "%s: using RX channel %d (index %u), transfer_size=%lu bytes\n",
            dma->tag, dma->rx_channel, rx_index, (unsigned long)dma->transfer_size);

    for(i = 0; i < AXIDMA_BUF_COUNT; ++i) {
        dma->buffers[i] = axidma_malloc(dev, dma->transfer_size);
        if(dma->buffers[i] == NULL) {
            fprintf(stderr, "%s: axidma_malloc failed for buffer %u\n", dma->tag, i);
            goto err_free_buffers;
        }
    }

    dma->last_complete_ms = monotonic_ms();
    if(pthread_create(&dma->worker, NULL, dma_worker_thread, dma) != 0) {
        fprintf(stderr, "%s: pthread_create failed (%s)\n", dma->tag, strerror(errno));
        goto err_free_buffers;
    }

    dma->thread_started = 1;
    return 0;

err_free_buffers:
    for(i = 0; i < AXIDMA_BUF_COUNT; ++i) {
        if(dma->buffers[i] != NULL) {
            axidma_free(dev, dma->buffers[i], dma->transfer_size);
            dma->buffers[i] = NULL;
        }
    }
err_release_dev:
    dma_shared_release();
err_release_lock:
    dma->rx_channel = -1;
    if(dma->lock_inited) {
        pthread_mutex_destroy(&dma->lock);
        dma->lock_inited = 0;
    }
    return -1;
}

static int dma_service_state(dma_state_t *dma)
{
    uint64_t now;
    uint64_t last_complete_ms;

    if(!dma->lock_inited || (!dma->thread_started && dma->rx_channel < 0)) {
        return -1;
    }

    pthread_mutex_lock(&dma->lock);
    last_complete_ms = dma->last_complete_ms;
    now = monotonic_ms();
    if((now - last_complete_ms) > 2000U && (now - dma->last_warn_ms) > 2000U) {
        dma->last_warn_ms = now;
        fprintf(stderr,
                "%s: no completed DMA transfer for %llums, completed=%llu errors=%llu\n",
                dma->tag,
                (unsigned long long)(now - last_complete_ms),
                dma->completed_count,
                dma->error_count);
    }
    pthread_mutex_unlock(&dma->lock);

    return 0;
}

static unsigned long long dma_completed_count_state(dma_state_t *dma)
{
    unsigned long long completed;

    if(!dma->lock_inited) {
        return 0ULL;
    }

    pthread_mutex_lock(&dma->lock);
    completed = dma->completed_count;
    pthread_mutex_unlock(&dma->lock);
    return completed;
}

static int dma_copy_latest_frame_state(dma_state_t *dma, uint64_t out[DMA_S2MM_FRAME_WORDS])
{
    if(out == NULL || !dma->lock_inited) {
        return 0;
    }

    pthread_mutex_lock(&dma->lock);
    if(!dma->latest_valid) {
        pthread_mutex_unlock(&dma->lock);
        return 0;
    }

    memcpy(out, dma->latest_frame, DMA_S2MM_FRAME_BYTES);
    pthread_mutex_unlock(&dma->lock);
    return 1;
}

static void dma_shutdown_state(dma_state_t *dma)
{
    axidma_dev_t dev;
    unsigned i;
    int should_stop = 0;

    pthread_mutex_lock(&g_axidma_lock);
    dev = g_axidma_dev;
    pthread_mutex_unlock(&g_axidma_lock);

    if(dma->lock_inited) {
        pthread_mutex_lock(&dma->lock);
        dma->stop_requested = 1;
        should_stop = dma->in_transfer;
        pthread_mutex_unlock(&dma->lock);
    }

    if(should_stop && dev != NULL && dma->rx_channel >= 0) {
        axidma_stop_transfer(dev, dma->rx_channel);
    }

    if(dma->thread_started) {
        pthread_join(dma->worker, NULL);
        dma->thread_started = 0;
    }

    if(dev != NULL) {
        for(i = 0; i < AXIDMA_BUF_COUNT; ++i) {
            if(dma->buffers[i] != NULL) {
                axidma_free(dev, dma->buffers[i], dma->transfer_size);
                dma->buffers[i] = NULL;
            }
        }
    }

    dma->rx_channel = -1;

    if(dma->lock_inited) {
        pthread_mutex_destroy(&dma->lock);
        dma->lock_inited = 0;
    }

    dma->stop_requested = 0;
    dma->in_transfer = 0;
    dma->latest_valid = 0;
    dma_shared_release();
}

int dma_s2mm_linux_init(void)
{
    return dma_init_state(&g_dma_main, 0U);
}

int dma_s2mm_linux_kick(unsigned dwell_ms)
{
    (void)dwell_ms;
    return dma_s2mm_linux_init();
}

int dma_s2mm_linux_service(void)
{
    return dma_service_state(&g_dma_main);
}

unsigned long long dma_s2mm_linux_completed_count(void)
{
    return dma_completed_count_state(&g_dma_main);
}

int dma_s2mm_linux_copy_latest_frame(uint64_t out[DMA_S2MM_FRAME_WORDS])
{
    return dma_copy_latest_frame_state(&g_dma_main, out);
}

void dma_s2mm_linux_shutdown(void)
{
    dma_shutdown_state(&g_dma_main);
}

int dma_s2mm_linux_scope_init(unsigned rx_index)
{
    return dma_init_state(&g_dma_scope, rx_index);
}

int dma_s2mm_linux_scope_kick(unsigned dwell_ms)
{
    (void)dwell_ms;
    return dma_s2mm_linux_scope_init(g_dma_scope.rx_index);
}

int dma_s2mm_linux_scope_service(void)
{
    return dma_service_state(&g_dma_scope);
}

unsigned long long dma_s2mm_linux_scope_completed_count(void)
{
    return dma_completed_count_state(&g_dma_scope);
}

int dma_s2mm_linux_scope_copy_latest_frame(uint64_t out[DMA_S2MM_FRAME_WORDS])
{
    return dma_copy_latest_frame_state(&g_dma_scope, out);
}

void dma_s2mm_linux_scope_shutdown(void)
{
    dma_shutdown_state(&g_dma_scope);
}
