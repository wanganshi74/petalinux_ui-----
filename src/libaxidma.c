#include "libaxidma.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

typedef struct dma_channel {
    enum axidma_dir dir;
    enum axidma_type type;
    int channel_id;
    axidma_cb_t callback;
    void *user_data;
} dma_channel_t;

struct axidma_dev {
    bool initialized;
    int fd;
    array_t dma_tx_chans;
    array_t dma_rx_chans;
    array_t vdma_tx_chans;
    array_t vdma_rx_chans;
    int num_channels;
    dma_channel_t *channels;
};

static struct axidma_dev g_axidma_dev = {0};

static int categorize_channels(axidma_dev_t dev,
                               struct axidma_chan *channels,
                               struct axidma_num_channels *num_chan)
{
    int i;

    dev->channels = malloc(num_chan->num_channels * sizeof(dev->channels[0]));
    if(dev->channels == NULL) {
        return -ENOMEM;
    }

    dev->dma_tx_chans.data = malloc(num_chan->num_dma_tx_channels * sizeof(dev->dma_tx_chans.data[0]));
    dev->dma_rx_chans.data = malloc(num_chan->num_dma_rx_channels * sizeof(dev->dma_rx_chans.data[0]));
    dev->vdma_tx_chans.data = malloc(num_chan->num_vdma_tx_channels * sizeof(dev->vdma_tx_chans.data[0]));
    dev->vdma_rx_chans.data = malloc(num_chan->num_vdma_rx_channels * sizeof(dev->vdma_rx_chans.data[0]));
    if((num_chan->num_dma_tx_channels > 0 && dev->dma_tx_chans.data == NULL) ||
       (num_chan->num_dma_rx_channels > 0 && dev->dma_rx_chans.data == NULL) ||
       (num_chan->num_vdma_tx_channels > 0 && dev->vdma_tx_chans.data == NULL) ||
       (num_chan->num_vdma_rx_channels > 0 && dev->vdma_rx_chans.data == NULL)) {
        free(dev->channels);
        free(dev->dma_tx_chans.data);
        free(dev->dma_rx_chans.data);
        free(dev->vdma_tx_chans.data);
        free(dev->vdma_rx_chans.data);
        memset(&dev->dma_tx_chans, 0, sizeof(dev->dma_tx_chans));
        memset(&dev->dma_rx_chans, 0, sizeof(dev->dma_rx_chans));
        memset(&dev->vdma_tx_chans, 0, sizeof(dev->vdma_tx_chans));
        memset(&dev->vdma_rx_chans, 0, sizeof(dev->vdma_rx_chans));
        return -ENOMEM;
    }

    dev->num_channels = num_chan->num_channels;
    dev->dma_tx_chans.len = 0;
    dev->dma_rx_chans.len = 0;
    dev->vdma_tx_chans.len = 0;
    dev->vdma_rx_chans.len = 0;

    for(i = 0; i < num_chan->num_channels; ++i) {
        array_t *array = NULL;
        struct axidma_chan *chan = &channels[i];
        dma_channel_t *dma_chan = &dev->channels[i];

        if(chan->dir == AXIDMA_WRITE && chan->type == AXIDMA_DMA) {
            array = &dev->dma_tx_chans;
        } else if(chan->dir == AXIDMA_READ && chan->type == AXIDMA_DMA) {
            array = &dev->dma_rx_chans;
        } else if(chan->dir == AXIDMA_WRITE && chan->type == AXIDMA_VDMA) {
            array = &dev->vdma_tx_chans;
        } else if(chan->dir == AXIDMA_READ && chan->type == AXIDMA_VDMA) {
            array = &dev->vdma_rx_chans;
        }

        assert(array != NULL);
        array->data[array->len++] = chan->channel_id;

        dma_chan->dir = chan->dir;
        dma_chan->type = chan->type;
        dma_chan->channel_id = chan->channel_id;
        dma_chan->callback = NULL;
        dma_chan->user_data = NULL;
    }

    return 0;
}

static int probe_channels(axidma_dev_t dev)
{
    int rc;
    struct axidma_chan *channels;
    struct axidma_num_channels num_chan;
    struct axidma_channel_info channel_info;

    rc = ioctl(dev->fd, AXIDMA_GET_NUM_DMA_CHANNELS, &num_chan);
    if(rc < 0) {
        perror("Unable to get the number of DMA channels");
        return rc;
    }
    if(num_chan.num_channels == 0) {
        fprintf(stderr, "No DMA channels are present.\n");
        return -ENODEV;
    }

    channels = malloc(num_chan.num_channels * sizeof(channels[0]));
    if(channels == NULL) {
        return -ENOMEM;
    }

    channel_info.channels = channels;
    rc = ioctl(dev->fd, AXIDMA_GET_DMA_CHANNELS, &channel_info);
    if(rc < 0) {
        perror("Unable to get DMA channel information");
        free(channels);
        return rc;
    }

    rc = categorize_channels(dev, channels, &num_chan);
    free(channels);
    return rc;
}

static void axidma_callback(int signal, siginfo_t *siginfo, void *context)
{
    int channel_id;
    dma_channel_t *chan;

    (void)signal;
    (void)context;

    assert(0 <= siginfo->si_int && siginfo->si_int < g_axidma_dev.num_channels);
    channel_id = siginfo->si_int;
    chan = &g_axidma_dev.channels[channel_id];
    if(chan->callback != NULL) {
        chan->callback(channel_id, chan->user_data);
    }
}

static int setup_dma_callback(axidma_dev_t dev)
{
    int rc;
    struct sigaction sigact;

    memset(&sigact, 0, sizeof(sigact));
    sigact.sa_sigaction = axidma_callback;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = SA_RESTART | SA_SIGINFO;

    rc = sigaction(SIGRTMIN, &sigact, NULL);
    if(rc < 0) {
        perror("Failed to register DMA callback");
        return rc;
    }

    rc = ioctl(dev->fd, AXIDMA_SET_DMA_SIGNAL, SIGRTMIN);
    if(rc < 0) {
        perror("Failed to set the DMA callback signal");
        return rc;
    }

    return 0;
}

static dma_channel_t *find_channel(axidma_dev_t dev, int channel_id)
{
    int i;

    for(i = 0; i < dev->num_channels; ++i) {
        if(dev->channels[i].channel_id == channel_id) {
            return &dev->channels[i];
        }
    }

    return NULL;
}

static unsigned long dir_to_ioctl(enum axidma_dir dir)
{
    switch(dir) {
    case AXIDMA_READ:
        return AXIDMA_DMA_READ;
    case AXIDMA_WRITE:
        return AXIDMA_DMA_WRITE;
    }

    assert(false);
    return 0;
}

struct axidma_dev *axidma_init(void)
{
    assert(!g_axidma_dev.initialized);

    g_axidma_dev.fd = open(AXIDMA_DEV_PATH, O_RDWR | O_EXCL);
    if(g_axidma_dev.fd < 0) {
        perror("Error opening AXI DMA device");
        fprintf(stderr, "Expected the AXI DMA device at the path `%s`\n", AXIDMA_DEV_PATH);
        return NULL;
    }

    if(probe_channels(&g_axidma_dev) < 0) {
        close(g_axidma_dev.fd);
        g_axidma_dev.fd = -1;
        return NULL;
    }

    if(setup_dma_callback(&g_axidma_dev) < 0) {
        close(g_axidma_dev.fd);
        g_axidma_dev.fd = -1;
        return NULL;
    }

    g_axidma_dev.initialized = true;
    return &g_axidma_dev;
}

void axidma_destroy(axidma_dev_t dev)
{
    free(dev->vdma_rx_chans.data);
    free(dev->vdma_tx_chans.data);
    free(dev->dma_rx_chans.data);
    free(dev->dma_tx_chans.data);
    free(dev->channels);

    dev->vdma_rx_chans.data = NULL;
    dev->vdma_tx_chans.data = NULL;
    dev->dma_rx_chans.data = NULL;
    dev->dma_tx_chans.data = NULL;
    dev->channels = NULL;

    if(close(dev->fd) < 0) {
        perror("Failed to close the AXI DMA device");
        assert(false);
    }

    dev->fd = -1;
    dev->initialized = false;
}

const array_t *axidma_get_dma_tx(axidma_dev_t dev)
{
    return &dev->dma_tx_chans;
}

const array_t *axidma_get_dma_rx(axidma_dev_t dev)
{
    return &dev->dma_rx_chans;
}

const array_t *axidma_get_vdma_tx(axidma_dev_t dev)
{
    return &dev->vdma_tx_chans;
}

const array_t *axidma_get_vdma_rx(axidma_dev_t dev)
{
    return &dev->vdma_rx_chans;
}

void *axidma_malloc(axidma_dev_t dev, size_t size)
{
    void *addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd, 0);
    if(addr == MAP_FAILED) {
        return NULL;
    }

    return addr;
}

void axidma_free(axidma_dev_t dev, void *addr, size_t size)
{
    (void)dev;
    if(munmap(addr, size) < 0) {
        perror("Failed to free the AXI DMA memory mapped region");
        assert(false);
    }
}

int axidma_register_buffer(axidma_dev_t dev, int dmabuf_fd, void *user_addr, size_t size)
{
    int rc;
    struct axidma_register_buffer register_buffer;

    register_buffer.fd = dmabuf_fd;
    register_buffer.size = size;
    register_buffer.user_addr = user_addr;

    rc = ioctl(dev->fd, AXIDMA_REGISTER_BUFFER, &register_buffer);
    if(rc < 0) {
        perror("Failed to register the external DMA buffer");
    }

    return rc;
}

void axidma_unregister_buffer(axidma_dev_t dev, void *user_addr)
{
    if(ioctl(dev->fd, AXIDMA_UNREGISTER_BUFFER, user_addr) < 0) {
        perror("Failed to unregister the external DMA buffer");
        assert(false);
    }
}

void axidma_set_callback(axidma_dev_t dev, int channel, axidma_cb_t callback, void *data)
{
    dma_channel_t *chan = find_channel(dev, channel);
    assert(chan != NULL);
    chan->callback = callback;
    chan->user_data = data;
}

int axidma_oneway_transfer(axidma_dev_t dev, int channel, void *buf, size_t len, bool wait)
{
    int rc;
    struct axidma_transaction trans;
    unsigned long cmd;
    dma_channel_t *dma_chan = find_channel(dev, channel);

    assert(dma_chan != NULL);
    trans.wait = wait;
    trans.channel_id = channel;
    trans.buf = buf;
    trans.buf_len = len;
    cmd = dir_to_ioctl(dma_chan->dir);

    rc = ioctl(dev->fd, cmd, &trans);
    if(rc < 0) {
        perror("Failed to perform the AXI DMA transfer");
        return rc;
    }

    return 0;
}

int axidma_twoway_transfer(axidma_dev_t dev, int tx_channel, void *tx_buf, size_t tx_len,
                           struct axidma_video_frame *tx_frame, int rx_channel,
                           void *rx_buf, size_t rx_len, struct axidma_video_frame *rx_frame,
                           bool wait)
{
    int rc;
    struct axidma_inout_transaction trans;

    assert(find_channel(dev, tx_channel) != NULL);
    assert(find_channel(dev, tx_channel)->dir == AXIDMA_WRITE);
    assert(find_channel(dev, rx_channel) != NULL);
    assert(find_channel(dev, rx_channel)->dir == AXIDMA_READ);

    memset(&trans, 0, sizeof(trans));
    trans.wait = wait;
    trans.tx_channel_id = tx_channel;
    trans.tx_buf = tx_buf;
    trans.tx_buf_len = tx_len;
    trans.rx_channel_id = rx_channel;
    trans.rx_buf = rx_buf;
    trans.rx_buf_len = rx_len;

    if(tx_frame == NULL) {
        memset(&trans.tx_frame, -1, sizeof(trans.tx_frame));
    } else {
        memcpy(&trans.tx_frame, tx_frame, sizeof(trans.tx_frame));
    }

    if(rx_frame == NULL) {
        memset(&trans.rx_frame, -1, sizeof(trans.rx_frame));
    } else {
        memcpy(&trans.rx_frame, rx_frame, sizeof(trans.rx_frame));
    }

    rc = ioctl(dev->fd, AXIDMA_DMA_READWRITE, &trans);
    if(rc < 0) {
        perror("Failed to perform the AXI DMA read-write transfer");
    }

    return rc;
}

int axidma_video_transfer(axidma_dev_t dev, int display_channel, size_t width, size_t height,
                          size_t depth, void **frame_buffers, int num_buffers)
{
    int rc;
    unsigned long cmd;
    struct axidma_video_transaction trans;
    dma_channel_t *dma_chan = find_channel(dev, display_channel);

    assert(dma_chan != NULL);
    assert(dma_chan->type == AXIDMA_VDMA);

    memset(&trans, 0, sizeof(trans));
    trans.channel_id = display_channel;
    trans.num_frame_buffers = num_buffers;
    trans.frame_buffers = frame_buffers;
    trans.frame.width = (int)width;
    trans.frame.height = (int)height;
    trans.frame.depth = (int)depth;

    cmd = (dma_chan->dir == AXIDMA_READ) ? AXIDMA_DMA_VIDEO_READ : AXIDMA_DMA_VIDEO_WRITE;
    rc = ioctl(dev->fd, cmd, &trans);
    if(rc < 0) {
        perror("Failed to perform the AXI DMA video transfer");
    }

    return rc;
}

void axidma_stop_transfer(axidma_dev_t dev, int channel)
{
    struct axidma_chan chan;
    dma_channel_t *dma_chan = find_channel(dev, channel);

    assert(dma_chan != NULL);
    memset(&chan, 0, sizeof(chan));
    chan.channel_id = channel;
    chan.dir = dma_chan->dir;
    chan.type = dma_chan->type;

    if(ioctl(dev->fd, AXIDMA_STOP_DMA_CHANNEL, &chan) < 0) {
        perror("Failed to stop the DMA channel");
        assert(false);
    }
}
