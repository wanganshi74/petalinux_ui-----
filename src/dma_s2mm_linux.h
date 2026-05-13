#ifndef LINUX_POWER_UI_DMA_S2MM_LINUX_H
#define LINUX_POWER_UI_DMA_S2MM_LINUX_H

#include <stddef.h>
#include <stdint.h>

#define DMA_S2MM_FRAME_BYTES 4096U
#define DMA_S2MM_FRAME_WORDS (DMA_S2MM_FRAME_BYTES / sizeof(uint64_t))

int dma_s2mm_linux_init(void);
int dma_s2mm_linux_kick(unsigned dwell_ms);
int dma_s2mm_linux_service(void);
unsigned long long dma_s2mm_linux_completed_count(void);
int dma_s2mm_linux_copy_latest_frame(uint64_t out[DMA_S2MM_FRAME_WORDS]);
void dma_s2mm_linux_shutdown(void);

int dma_s2mm_linux_scope_init(unsigned rx_index);
int dma_s2mm_linux_scope_kick(unsigned dwell_ms);
int dma_s2mm_linux_scope_service(void);
unsigned long long dma_s2mm_linux_scope_completed_count(void);
int dma_s2mm_linux_scope_copy_latest_frame(uint64_t out[DMA_S2MM_FRAME_WORDS]);
void dma_s2mm_linux_scope_shutdown(void);

#endif
