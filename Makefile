UI_DRM_CARD ?= /dev/dri/card0
OUT ?= drm_page1
TD_STATS_BASE ?= 0x40000000
PLL_PHASE_BASE ?= 0x40010000
AXIDMA_RX_BYTES ?= 4096
FLAGS := $(shell pkg-config --cflags --libs libdrm) -Wall -O2 -g

all: $(OUT)

$(OUT): src/drm_page1.c src/power_model.c src/power_model.h src/amp_comp.c src/amp_comp.h src/dma_s2mm_linux.c src/dma_s2mm_linux.h src/libaxidma.c src/libaxidma.h src/axidma_ioctl.h
	$(CC) -o $(OUT) src/drm_page1.c src/power_model.c src/amp_comp.c src/dma_s2mm_linux.c src/libaxidma.c $(FLAGS) -lpthread \
		-DDRM_PAGE1_CARD=\"$(UI_DRM_CARD)\" \
		-DTD_STATS_BASE_ADDR=$(TD_STATS_BASE) \
		-DPLL_PHASE_BASE_ADDR=$(PLL_PHASE_BASE) \
		-DAXIDMA_RX_BYTES=$(AXIDMA_RX_BYTES)

clean:
	rm -f $(OUT)

.PHONY: all clean

