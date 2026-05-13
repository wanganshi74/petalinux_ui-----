#ifndef LIBAXIDMA_H_
#define LIBAXIDMA_H_

#include <stddef.h>
#include <stdbool.h>

#include "axidma_ioctl.h"

struct axidma_dev;
typedef struct axidma_dev *axidma_dev_t;

typedef struct array {
    int len;
    int *data;
} array_t;

typedef void (*axidma_cb_t)(int channel_id, void *data);

struct axidma_dev *axidma_init(void);
void axidma_destroy(axidma_dev_t dev);

const array_t *axidma_get_dma_tx(axidma_dev_t dev);
const array_t *axidma_get_dma_rx(axidma_dev_t dev);
const array_t *axidma_get_vdma_tx(axidma_dev_t dev);
const array_t *axidma_get_vdma_rx(axidma_dev_t dev);

void *axidma_malloc(axidma_dev_t dev, size_t size);
void axidma_free(axidma_dev_t dev, void *addr, size_t size);

int axidma_register_buffer(axidma_dev_t dev, int dmabuf_fd, void *user_addr, size_t size);
void axidma_unregister_buffer(axidma_dev_t dev, void *user_addr);

void axidma_set_callback(axidma_dev_t dev, int channel, axidma_cb_t callback, void *data);

int axidma_oneway_transfer(axidma_dev_t dev, int channel, void *buf, size_t len, bool wait);
int axidma_twoway_transfer(axidma_dev_t dev, int tx_channel, void *tx_buf, size_t tx_len,
                           struct axidma_video_frame *tx_frame, int rx_channel,
                           void *rx_buf, size_t rx_len, struct axidma_video_frame *rx_frame,
                           bool wait);
int axidma_video_transfer(axidma_dev_t dev, int display_channel, size_t width, size_t height,
                          size_t depth, void **frame_buffers, int num_buffers);
void axidma_stop_transfer(axidma_dev_t dev, int channel);

#endif
