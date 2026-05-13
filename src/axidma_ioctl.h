#ifndef AXIDMA_IOCTL_H_
#define AXIDMA_IOCTL_H_

#include <asm/ioctl.h>
#include <stdbool.h>
#include <stddef.h>

#define AXIDMA_DEV_NAME "axidma"
#define AXIDMA_DEV_PATH ("/dev/" AXIDMA_DEV_NAME)

struct dma_chan;

enum axidma_dir {
    AXIDMA_WRITE,
    AXIDMA_READ
};

enum axidma_type {
    AXIDMA_DMA,
    AXIDMA_VDMA
};

struct axidma_video_frame {
    int height;
    int width;
    int depth;
};

struct axidma_chan {
    enum axidma_dir dir;
    enum axidma_type type;
    int channel_id;
    const char *name;
    struct dma_chan *chan;
};

struct axidma_num_channels {
    int num_channels;
    int num_dma_tx_channels;
    int num_dma_rx_channels;
    int num_vdma_tx_channels;
    int num_vdma_rx_channels;
};

struct axidma_channel_info {
    struct axidma_chan *channels;
};

struct axidma_register_buffer {
    int fd;
    size_t size;
    void *user_addr;
};

struct axidma_transaction {
    bool wait;
    int channel_id;
    void *buf;
    size_t buf_len;
    union {
        struct axidma_video_frame frame;
    };
};

struct axidma_inout_transaction {
    bool wait;
    int tx_channel_id;
    void *tx_buf;
    size_t tx_buf_len;
    struct axidma_video_frame tx_frame;
    int rx_channel_id;
    void *rx_buf;
    size_t rx_buf_len;
    struct axidma_video_frame rx_frame;
};

struct axidma_video_transaction {
    int channel_id;
    int num_frame_buffers;
    void **frame_buffers;
    struct axidma_video_frame frame;
};

#define AXIDMA_IOCTL_MAGIC          'W'
#define AXIDMA_NUM_IOCTLS           10

#define AXIDMA_GET_NUM_DMA_CHANNELS _IOW(AXIDMA_IOCTL_MAGIC, 0, struct axidma_num_channels)
#define AXIDMA_GET_DMA_CHANNELS     _IOR(AXIDMA_IOCTL_MAGIC, 1, struct axidma_channel_info)
#define AXIDMA_SET_DMA_SIGNAL       _IO(AXIDMA_IOCTL_MAGIC, 2)
#define AXIDMA_REGISTER_BUFFER      _IOR(AXIDMA_IOCTL_MAGIC, 3, struct axidma_register_buffer)
#define AXIDMA_DMA_READ             _IOR(AXIDMA_IOCTL_MAGIC, 4, struct axidma_transaction)
#define AXIDMA_DMA_WRITE            _IOR(AXIDMA_IOCTL_MAGIC, 5, struct axidma_transaction)
#define AXIDMA_DMA_READWRITE        _IOR(AXIDMA_IOCTL_MAGIC, 6, struct axidma_inout_transaction)
#define AXIDMA_DMA_VIDEO_READ       _IOR(AXIDMA_IOCTL_MAGIC, 7, struct axidma_video_transaction)
#define AXIDMA_DMA_VIDEO_WRITE      _IOR(AXIDMA_IOCTL_MAGIC, 8, struct axidma_video_transaction)
#define AXIDMA_STOP_DMA_CHANNEL     _IOR(AXIDMA_IOCTL_MAGIC, 9, struct axidma_chan)
#define AXIDMA_UNREGISTER_BUFFER    _IO(AXIDMA_IOCTL_MAGIC, 10)

#endif
