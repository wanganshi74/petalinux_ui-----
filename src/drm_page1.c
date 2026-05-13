#define _GNU_SOURCE

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <linux/input.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "dma_s2mm_linux.h"
#include "power_model.h"

#ifndef DRM_PAGE1_CARD
#define DRM_PAGE1_CARD "/dev/dri/card0"
#endif

#define BASE_W 800
#define BASE_H 480

#define COLOR_BG             0x09111a
#define COLOR_SHELL          0x0f1822
#define COLOR_PANEL          0x13202d
#define COLOR_CARD           0x1a2b3c
#define COLOR_CARD_ALT       0x102033
#define COLOR_BORDER         0x32506d
#define COLOR_DIVIDER        0x22384c
#define COLOR_TEXT_PRIMARY   0xecf4fb
#define COLOR_TEXT_MUTED     0x93aac0
#define COLOR_TEXT_SECONDARY 0x637d96
#define COLOR_TEXT_HEADING   0xf8fbfe
#define COLOR_ACCENT_BLUE    0x51c6ff
#define COLOR_ACCENT_PURPLE  0xc68aff
#define COLOR_ACCENT_GREEN   0x7adf97
#define COLOR_ACCENT_AMBER   0xf1c56d
#define COLOR_LOGO_BG        0x254c74
#define COLOR_LOGO_TEXT      0xf7fbff
#define COLOR_NAV_ON         0x2e8df5
#define COLOR_NAV_OFF        0x1a2b3c

#define DRM_PAGE1_BUILD_TAG  "double-buffer-v11-pages"

#define UI_INPUT_ENV "POWER_UI_INPUT"
#define UI_PAGE_COUNT 4U
#define FFT_HARMONIC_COUNT 7U
#define FFT_BAR_SRC_MAX 136U
#define FFT_BAR_MIN_BASE 10
#define SCOPE_SAMPLE_WORDS (DMA_S2MM_FRAME_BYTES / sizeof(uint32_t))
#define SCOPE_VIEW_POINTS 512
#define SCOPE_HISTORY_FRAMES 32U
#define SCOPE_HISTORY_SAMPLES (SCOPE_SAMPLE_WORDS * SCOPE_HISTORY_FRAMES)
#define SCOPE_WINDOW_SAMPLES 20000U
#define SCOPE_TRIGGER_HYST_RAW 64
#define SCOPE_TRIGGER_PRE_POINTS (SCOPE_VIEW_POINTS / 3)
#define SCOPE_DEFAULT_RX_INDEX 1U

#define BITS_PER_LONG_LOCAL ((int)(sizeof(unsigned long) * 8U))
#define NBITS_LOCAL(x) ((((x) - 1) / BITS_PER_LONG_LOCAL) + 1)

int fd = -1;
static volatile sig_atomic_t g_stop_requested = 0;

struct modeset_dev;
struct modeset_buf;
static int modeset_find_crtc(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev);
static int modeset_create_fb(int fd, struct modeset_dev *dev);
static int modeset_setup_dev(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev);
static int modeset_open(int *out, const char *node);
static int modeset_prepare(int fd);
static void modeset_draw(void);
static void modeset_cleanup(int fd);
static int scale_x(const struct modeset_dev *dev, int x);
static int scale_y(const struct modeset_dev *dev, int y);
static void fill_rect(struct modeset_dev *dev, int x, int y, int w, int h, uint32_t color);
static void stroke_rect(struct modeset_dev *dev, int x, int y, int w, int h, int thickness, uint32_t color);
static void draw_text(struct modeset_dev *dev, int x, int y, const char *text, int scale, uint32_t color);
static void draw_text_right(struct modeset_dev *dev, int right, int y, const char *text, int scale, uint32_t color);
static void draw_text_center(struct modeset_dev *dev, int x, int y, int w, const char *text, int scale, uint32_t color);
static void draw_rule(struct modeset_dev *dev, int x, int y, int w, int thickness, uint32_t color);

static const uint8_t GLYPH_SPACE[7] = { 0, 0, 0, 0, 0, 0, 0 };
static const uint8_t GLYPH_DOT[7]   = { 0, 0, 0, 0, 0, 0x04, 0x04 };
static const uint8_t GLYPH_MINUS[7] = { 0, 0, 0, 0x1f, 0, 0, 0 };
static const uint8_t GLYPH_SLASH[7] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0, 0 };
static const uint8_t GLYPH_PERCENT[7] = { 0x19, 0x19, 0x02, 0x04, 0x08, 0x13, 0x13 };
static const uint8_t GLYPH_0[7] = { 0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e };
static const uint8_t GLYPH_1[7] = { 0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e };
static const uint8_t GLYPH_2[7] = { 0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f };
static const uint8_t GLYPH_3[7] = { 0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e };
static const uint8_t GLYPH_4[7] = { 0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02 };
static const uint8_t GLYPH_5[7] = { 0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e };
static const uint8_t GLYPH_6[7] = { 0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e };
static const uint8_t GLYPH_7[7] = { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 };
static const uint8_t GLYPH_8[7] = { 0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e };
static const uint8_t GLYPH_9[7] = { 0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c };
static const uint8_t GLYPH_A[7] = { 0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 };
static const uint8_t GLYPH_B[7] = { 0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e };
static const uint8_t GLYPH_C[7] = { 0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e };
static const uint8_t GLYPH_D[7] = { 0x1c, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1c };
static const uint8_t GLYPH_E[7] = { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f };
static const uint8_t GLYPH_F[7] = { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 };
static const uint8_t GLYPH_G[7] = { 0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f };
static const uint8_t GLYPH_H[7] = { 0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 };
static const uint8_t GLYPH_I[7] = { 0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e };
static const uint8_t GLYPH_J[7] = { 0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c };
static const uint8_t GLYPH_K[7] = { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 };
static const uint8_t GLYPH_L[7] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f };
static const uint8_t GLYPH_M[7] = { 0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11 };
static const uint8_t GLYPH_N[7] = { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 };
static const uint8_t GLYPH_O[7] = { 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e };
static const uint8_t GLYPH_P[7] = { 0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10 };
static const uint8_t GLYPH_Q[7] = { 0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d };
static const uint8_t GLYPH_R[7] = { 0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11 };
static const uint8_t GLYPH_S[7] = { 0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e };
static const uint8_t GLYPH_T[7] = { 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 };
static const uint8_t GLYPH_U[7] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e };
static const uint8_t GLYPH_V[7] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04 };
static const uint8_t GLYPH_W[7] = { 0x11, 0x11, 0x11, 0x15, 0x15, 0x1b, 0x11 };
static const uint8_t GLYPH_X[7] = { 0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11 };
static const uint8_t GLYPH_Y[7] = { 0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04 };
static const uint8_t GLYPH_Z[7] = { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f };

struct modeset_buf {
    uint32_t stride;
    uint32_t size;
    uint32_t handle;
    uint32_t fb;
    uint8_t *map;
};

struct modeset_dev {
    struct modeset_dev *next;
    uint32_t width;
    uint32_t height;
    drmModeModeInfo mode;
    uint32_t conn;
    uint32_t crtc;
    drmModeCrtc *saved_crtc;
    struct modeset_buf bufs[2];
    int front_buf;
    int draw_buf;
};

typedef enum {
    UI_PAGE_MAIN = 0,
    UI_PAGE_FFT = 1,
    UI_PAGE_HOLD = 2,
    UI_PAGE_SCOPE = 3
} ui_page_t;

static void draw_page_chrome(struct modeset_dev *dev, ui_page_t active_page,
                             const char *page_text, const char *subtitle);

typedef enum {
    SCOPE_TRIGGER_RISING = 0,
    SCOPE_TRIGGER_FALLING = 1
} scope_trigger_edge_t;

typedef struct {
    int running;
    scope_trigger_edge_t edge;
    int32_t trigger_level_v;
    int32_t trigger_level_i;
    int triggered;
    int trigger_index;
    uint32_t sample_count;
    uint32_t history_count;
    uint32_t history_head;
    uint32_t window_samples;
    int16_t voltage_history[SCOPE_HISTORY_SAMPLES];
    int16_t current_history[SCOPE_HISTORY_SAMPLES];
    int voltage_y_min[SCOPE_VIEW_POINTS];
    int voltage_y_max[SCOPE_VIEW_POINTS];
    int current_y_min[SCOPE_VIEW_POINTS];
    int current_y_max[SCOPE_VIEW_POINTS];
    int trigger_screen_x;
    char voltage_rms[24];
    char current_rms[24];
    char voltage_pkpk[24];
    char current_pkpk[24];
    char debug_info[40];
} scope_view_t;

static int16_t scope_get_sample(const int16_t *samples, const scope_view_t *scope, uint32_t logical_index);

typedef struct {
    ui_page_t page;
    uint8_t fft_tab;
    hold_state_t hold_state;
    fft_view_t fft_view;
    hold_view_t hold_view;
    scope_view_t scope_view;
    int input_fd;
    int input_x_min;
    int input_x_max;
    int input_y_min;
    int input_y_max;
    int input_raw_x;
    int input_raw_y;
    int input_have_x;
    int input_have_y;
    int touch_down;
    int dirty;
} ui_state_t;

static struct modeset_dev *modeset_list = NULL;

static void sig_handler(int sig)
{
    (void)sig;
    g_stop_requested = 1;
}

static int test_bit_local(int bit, const unsigned long *array)
{
    return (int)((array[bit / BITS_PER_LONG_LOCAL] >> (bit % BITS_PER_LONG_LOCAL)) & 1UL);
}

static int clamp_int(int value, int min_value, int max_value)
{
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t scope_isqrt_u64(uint64_t x)
{
    uint64_t op = x;
    uint64_t res = 0;
    uint64_t one = (uint64_t)1 << 62;

    while(one > op) {
        one >>= 2;
    }

    while(one != 0) {
        if(op >= res + one) {
            op -= res + one;
            res += 2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return (uint32_t)res;
}

static void scope_view_defaults(scope_view_t *scope)
{
    memset(scope, 0, sizeof(*scope));
    scope->running = 1;
    scope->edge = SCOPE_TRIGGER_RISING;
    scope->window_samples = SCOPE_WINDOW_SAMPLES;
}

static int64_t scope_raw_to_milli(int32_t raw)
{
    return ((int64_t)raw * 2500LL) / 32768LL;
}

static void scope_fmt_milli(char *buf, size_t len, int64_t milli)
{
    unsigned long whole;
    unsigned long frac;

    if(milli < 0) {
        milli = -milli;
        whole = (unsigned long)((uint64_t)milli / 1000U);
        frac = (unsigned long)((uint64_t)milli % 1000U);
        snprintf(buf, len, "-%lu.%03lu", whole, frac);
        return;
    }

    whole = (unsigned long)((uint64_t)milli / 1000U);
    frac = (unsigned long)((uint64_t)milli % 1000U);
    snprintf(buf, len, "%lu.%03lu", whole, frac);
}

static int scope_find_trigger_index(const scope_view_t *scope, int use_voltage)
{
    const int16_t *samples = use_voltage ? scope->voltage_history : scope->current_history;
    int32_t trigger = use_voltage ? scope->trigger_level_v : scope->trigger_level_i;
    uint32_t i;

    if(scope->sample_count < 2U) {
        return -1;
    }

    for(i = 1; i < scope->sample_count; ++i) {
        int32_t prev = scope_get_sample(samples, scope, i - 1U);
        int32_t curr = scope_get_sample(samples, scope, i);

        if(scope->edge == SCOPE_TRIGGER_RISING) {
            if(prev <= (trigger - SCOPE_TRIGGER_HYST_RAW) &&
               curr >= (trigger + SCOPE_TRIGGER_HYST_RAW)) {
                return (int)i;
            }
        }
        else {
            if(prev >= (trigger + SCOPE_TRIGGER_HYST_RAW) &&
               curr <= (trigger - SCOPE_TRIGGER_HYST_RAW)) {
                return (int)i;
            }
        }
    }

    return -1;
}

static uint32_t scope_history_index(const scope_view_t *scope, uint32_t logical_index)
{
    uint32_t first;

    if(scope->history_count == 0U) {
        return 0U;
    }

    first = (scope->history_head + SCOPE_HISTORY_SAMPLES - scope->history_count) % SCOPE_HISTORY_SAMPLES;
    return (first + logical_index) % SCOPE_HISTORY_SAMPLES;
}

static int16_t scope_get_sample(const int16_t *samples, const scope_view_t *scope, uint32_t logical_index)
{
    if(scope->history_count == 0U || logical_index >= scope->history_count) {
        return 0;
    }
    return samples[scope_history_index(scope, logical_index)];
}

static int scope_map_raw_to_screen(int32_t raw, int graph_y, int graph_h)
{
    int half_h = graph_h / 2;
    int center_y = graph_y + half_h;
    int y = center_y - (int)((int64_t)raw * (int64_t)(half_h - 6) / 32768LL);

    if(y < graph_y + 2) {
        y = graph_y + 2;
    }
    if(y > graph_y + graph_h - 3) {
        y = graph_y + graph_h - 3;
    }
    return y;
}

static int32_t scope_estimate_trigger_level(const int16_t *samples, const scope_view_t *scope,
                                            uint32_t start, uint32_t count)
{
    int32_t raw_min = 32767;
    int32_t raw_max = -32768;
    uint32_t end = start + count;
    uint32_t i;

    if(end > scope->history_count) {
        end = scope->history_count;
    }

    for(i = start; i < end; ++i) {
        int32_t raw = scope_get_sample(samples, scope, i);
        if(raw < raw_min) {
            raw_min = raw;
        }
        if(raw > raw_max) {
            raw_max = raw;
        }
    }

    if(raw_min > raw_max) {
        return 0;
    }
    return (raw_min + raw_max) / 2;
}

static void scope_map_channel_to_screen(const int16_t *samples, const scope_view_t *scope,
                                        uint32_t start, uint32_t window_samples,
                                        int graph_y, int graph_h, int *out_min_y, int *out_max_y)
{
    uint32_t col;

    for(col = 0; col < SCOPE_VIEW_POINTS; ++col) {
        uint32_t seg_start = start + (uint32_t)(((uint64_t)window_samples * col) / SCOPE_VIEW_POINTS);
        uint32_t seg_end = start + (uint32_t)(((uint64_t)window_samples * (col + 1U)) / SCOPE_VIEW_POINTS);
        int32_t raw_min = 32767;
        int32_t raw_max = -32768;
        uint32_t i;

        if(seg_end <= seg_start) {
            seg_end = seg_start + 1U;
        }
        if(seg_end > scope->history_count) {
            seg_end = scope->history_count;
        }

        for(i = seg_start; i < seg_end; ++i) {
            int32_t raw = scope_get_sample(samples, scope, i);
            if(raw < raw_min) {
                raw_min = raw;
            }
            if(raw > raw_max) {
                raw_max = raw;
            }
        }

        if(raw_min > raw_max) {
            raw_min = 0;
            raw_max = 0;
        }

        out_min_y[col] = scope_map_raw_to_screen(raw_max, graph_y, graph_h);
        out_max_y[col] = scope_map_raw_to_screen(raw_min, graph_y, graph_h);
    }
}

static void scope_update_metrics(scope_view_t *scope)
{
    int32_t v_min = 32767;
    int32_t v_max = -32768;
    int32_t i_min = 32767;
    int32_t i_max = -32768;
    uint64_t v_sq = 0U;
    uint64_t i_sq = 0U;
    uint32_t i;

    if(scope->history_count == 0U) {
        snprintf(scope->voltage_rms, sizeof(scope->voltage_rms), "0.000");
        snprintf(scope->current_rms, sizeof(scope->current_rms), "0.000");
        snprintf(scope->voltage_pkpk, sizeof(scope->voltage_pkpk), "0.000");
        snprintf(scope->current_pkpk, sizeof(scope->current_pkpk), "0.000");
        snprintf(scope->debug_info, sizeof(scope->debug_info), "RAW 0");
        return;
    }

    for(i = 0; i < scope->history_count; ++i) {
        int32_t vr = scope_get_sample(scope->voltage_history, scope, i);
        int32_t ir = scope_get_sample(scope->current_history, scope, i);

        if(vr < v_min) {
            v_min = vr;
        }
        if(vr > v_max) {
            v_max = vr;
        }
        if(ir < i_min) {
            i_min = ir;
        }
        if(ir > i_max) {
            i_max = ir;
        }
        v_sq += (uint64_t)((int64_t)vr * (int64_t)vr);
        i_sq += (uint64_t)((int64_t)ir * (int64_t)ir);
    }

    scope_fmt_milli(scope->voltage_rms, sizeof(scope->voltage_rms),
                    scope_raw_to_milli((int32_t)scope_isqrt_u64(v_sq / scope->history_count)));
    scope_fmt_milli(scope->current_rms, sizeof(scope->current_rms),
                    scope_raw_to_milli((int32_t)scope_isqrt_u64(i_sq / scope->history_count)));
    scope_fmt_milli(scope->voltage_pkpk, sizeof(scope->voltage_pkpk),
                    scope_raw_to_milli(v_max - v_min));
    scope_fmt_milli(scope->current_pkpk, sizeof(scope->current_pkpk),
                    scope_raw_to_milli(i_max - i_min));
    snprintf(scope->debug_info, sizeof(scope->debug_info), "RAW V %d..%d", (int)v_min, (int)v_max);
}

static void scope_push_frame(scope_view_t *scope, const uint64_t *frame_words)
{
    const uint32_t *samples = (const uint32_t *)frame_words;
    uint32_t i;

    if(frame_words == NULL) {
        return;
    }

    for(i = 0; i < SCOPE_SAMPLE_WORDS; ++i) {
        uint32_t word = samples[i];
        scope->voltage_history[scope->history_head] = (int16_t)(word & 0xffffU);
        scope->current_history[scope->history_head] = (int16_t)((word >> 16) & 0xffffU);
        scope->history_head = (scope->history_head + 1U) % SCOPE_HISTORY_SAMPLES;
        if(scope->history_count < SCOPE_HISTORY_SAMPLES) {
            scope->history_count++;
        }
    }
}

static int scope_build_view_from_history(scope_view_t *scope)
{
    uint32_t window_samples;
    uint32_t start;
    int trigger_index;

    scope->sample_count = scope->history_count;
    if(scope->history_count < 2U) {
        return 0;
    }

    window_samples = scope->window_samples;
    if(window_samples > scope->history_count) {
        window_samples = scope->history_count;
    }
    if(scope->history_count < SCOPE_VIEW_POINTS) {
        window_samples = scope->history_count;
    }
    else if(window_samples < SCOPE_VIEW_POINTS) {
        window_samples = SCOPE_VIEW_POINTS;
    }

    start = scope->history_count - window_samples;
    scope->trigger_level_v = scope_estimate_trigger_level(scope->voltage_history, scope, start, window_samples);
    scope->trigger_level_i = scope_estimate_trigger_level(scope->current_history, scope, start, window_samples);
    trigger_index = scope_find_trigger_index(scope, 1);
    scope->triggered = (trigger_index >= 0);
    scope->trigger_index = trigger_index;

    if(trigger_index < 0) {
        scope->trigger_screen_x = -1;
    }
    else {
        uint32_t desired = (uint32_t)trigger_index;
        uint32_t pre = (uint32_t)((uint64_t)window_samples * SCOPE_TRIGGER_PRE_POINTS / SCOPE_VIEW_POINTS);

        if(desired > pre) {
            start = desired - pre;
        }
        else {
            start = 0U;
        }
        if(start + window_samples > scope->history_count) {
            start = scope->history_count - window_samples;
        }
        if((uint32_t)trigger_index >= start) {
            scope->trigger_screen_x = (int)((uint64_t)((uint32_t)trigger_index - start) * SCOPE_VIEW_POINTS / window_samples);
        }
        else {
            scope->trigger_screen_x = -1;
        }
    }

    scope_map_channel_to_screen(scope->voltage_history, scope, start, window_samples,
                                152, 182, scope->voltage_y_min, scope->voltage_y_max);
    scope_map_channel_to_screen(scope->current_history, scope, start, window_samples,
                                152, 182, scope->current_y_min, scope->current_y_max);
    scope_update_metrics(scope);
    return 1;
}

static int scope_build_view_from_dma(scope_view_t *scope, const uint64_t *frame_words)
{
    if(frame_words == NULL) {
        return 0;
    }

    scope_push_frame(scope, frame_words);
    return scope_build_view_from_history(scope);
}

static void draw_scope_page(struct modeset_dev *dev, const scope_view_t *scope)
{
    int graph_x = scale_x(dev, 54);
    int graph_y = scale_y(dev, 152);
    int graph_w = scale_x(dev, 516);
    int graph_h = scale_y(dev, 182);
    int card_x = scale_x(dev, 588);
    int card_y = scale_y(dev, 152);
    int card_w = scale_x(dev, 146);
    int card_h = scale_y(dev, 182);
    int footer_y = scale_y(dev, 352);
    int footer_h = scale_y(dev, 34);
    int btn_y = scale_y(dev, 356);
    int btn_w = scale_x(dev, 220);
    int btn_h = scale_y(dev, 34);
    int border = (dev->width >= 1024U) ? 3 : 2;
    int font_micro = 1;
    int font_small = 2;
    int font_medium = 2;
    int i;

    draw_page_chrome(dev, UI_PAGE_SCOPE, "PAGE 4", "TIME DOMAIN OSCILLOSCOPE");

    fill_rect(dev, graph_x, graph_y, graph_w, graph_h, COLOR_CARD_ALT);
    stroke_rect(dev, graph_x, graph_y, graph_w, graph_h, border, COLOR_BORDER);

    for(i = 1; i < 10; ++i) {
        int x = graph_x + (graph_w * i) / 10;
        fill_rect(dev, x, graph_y + scale_y(dev, 8), border, graph_h - scale_y(dev, 16), COLOR_DIVIDER);
    }
    for(i = 1; i < 8; ++i) {
        int y = graph_y + (graph_h * i) / 8;
        fill_rect(dev, graph_x + scale_x(dev, 8), y, graph_w - scale_x(dev, 16), border, COLOR_DIVIDER);
    }
    fill_rect(dev, graph_x + graph_w / 2, graph_y + scale_y(dev, 6), border, graph_h - scale_y(dev, 12), COLOR_BORDER);
    fill_rect(dev, graph_x + scale_x(dev, 6), graph_y + graph_h / 2, graph_w - scale_x(dev, 12), border, COLOR_BORDER);

    if(scope->trigger_screen_x >= 0 && scope->trigger_screen_x < SCOPE_VIEW_POINTS) {
        int tx = graph_x + (scope->trigger_screen_x * (graph_w - 1)) / (SCOPE_VIEW_POINTS - 1);
        fill_rect(dev, tx, graph_y + scale_y(dev, 4), border, graph_h - scale_y(dev, 8), COLOR_ACCENT_AMBER);
        draw_text(dev, tx - scale_x(dev, 4), graph_y - scale_y(dev, 14), "T", font_micro, COLOR_ACCENT_AMBER);
    }

    for(i = 1; i < SCOPE_VIEW_POINTS; ++i) {
        int x0 = graph_x + ((i - 1) * (graph_w - 1)) / (SCOPE_VIEW_POINTS - 1);
        int x1 = graph_x + (i * (graph_w - 1)) / (SCOPE_VIEW_POINTS - 1);
        int yv0_min = scale_y(dev, scope->voltage_y_min[i - 1]);
        int yv0_max = scale_y(dev, scope->voltage_y_max[i - 1]);
        int yv1_min = scale_y(dev, scope->voltage_y_min[i]);
        int yv1_max = scale_y(dev, scope->voltage_y_max[i]);
        int yi0_min = scale_y(dev, scope->current_y_min[i - 1]);
        int yi0_max = scale_y(dev, scope->current_y_max[i - 1]);
        int yi1_min = scale_y(dev, scope->current_y_min[i]);
        int yi1_max = scale_y(dev, scope->current_y_max[i]);

        fill_rect(dev, x0, yv0_min, border, yv0_max - yv0_min + 1, COLOR_ACCENT_BLUE);
        fill_rect(dev, x0 + border, yi0_min, border, yi0_max - yi0_min + 1, COLOR_ACCENT_GREEN);
        fill_rect(dev, x1, yv1_min, border, yv1_max - yv1_min + 1, COLOR_ACCENT_BLUE);
        fill_rect(dev, x1 + border, yi1_min, border, yi1_max - yi1_min + 1, COLOR_ACCENT_GREEN);
    }

    fill_rect(dev, card_x, card_y, card_w, card_h, COLOR_CARD);
    stroke_rect(dev, card_x, card_y, card_w, card_h, border, COLOR_BORDER);
    draw_text(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 12), "SCOPE STATUS", font_small, COLOR_TEXT_PRIMARY);
    draw_rule(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 32), card_w - scale_x(dev, 24), border, COLOR_DIVIDER);

    draw_text(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 48), "STATE", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, card_x + card_w - scale_x(dev, 12), card_y + scale_y(dev, 44),
                    scope->running ? "RUN" : "FREEZE", font_small,
                    scope->running ? COLOR_ACCENT_GREEN : COLOR_ACCENT_AMBER);

    draw_text(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 70), "TRIGGER", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, card_x + card_w - scale_x(dev, 12), card_y + scale_y(dev, 66),
                    scope->triggered ? "LOCKED" : "SEARCH", font_small,
                    scope->triggered ? COLOR_ACCENT_BLUE : COLOR_ACCENT_AMBER);

    draw_text(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 92), "EDGE", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, card_x + card_w - scale_x(dev, 12), card_y + scale_y(dev, 88),
                    (scope->edge == SCOPE_TRIGGER_RISING) ? "RISING" : "FALLING",
                    font_small, COLOR_TEXT_PRIMARY);

    draw_text(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 114), "SAMPLE", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, card_x + card_w - scale_x(dev, 12), card_y + scale_y(dev, 110),
                    "1.000 MHZ", font_small, COLOR_TEXT_PRIMARY);

    draw_text(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 136), "CH1 RMS", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, card_x + card_w - scale_x(dev, 12), card_y + scale_y(dev, 132),
                    scope->voltage_rms, font_medium, COLOR_ACCENT_BLUE);
    draw_text(dev, card_x + scale_x(dev, 12), card_y + scale_y(dev, 156), "CH2 RMS", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, card_x + card_w - scale_x(dev, 12), card_y + scale_y(dev, 152),
                    scope->current_rms, font_medium, COLOR_ACCENT_GREEN);
    draw_text_right(dev, card_x + card_w - scale_x(dev, 12), card_y + scale_y(dev, 170),
                    scope->debug_info, font_micro, COLOR_TEXT_MUTED);

    fill_rect(dev, graph_x, footer_y, graph_w + (card_x + card_w - graph_x), footer_h, COLOR_CARD);
    stroke_rect(dev, graph_x, footer_y, graph_w + (card_x + card_w - graph_x), footer_h, border, COLOR_BORDER);
    draw_text(dev, graph_x + scale_x(dev, 12), footer_y + scale_y(dev, 10), "CH1 VPP", font_micro, COLOR_TEXT_SECONDARY);
    draw_text(dev, graph_x + scale_x(dev, 72), footer_y + scale_y(dev, 8), scope->voltage_pkpk, font_small, COLOR_ACCENT_BLUE);
    draw_text(dev, graph_x + scale_x(dev, 196), footer_y + scale_y(dev, 10), "CH2 VPP", font_micro, COLOR_TEXT_SECONDARY);
    draw_text(dev, graph_x + scale_x(dev, 256), footer_y + scale_y(dev, 8), scope->current_pkpk, font_small, COLOR_ACCENT_GREEN);
    draw_text(dev, graph_x + scale_x(dev, 384), footer_y + scale_y(dev, 10), "TIME WIN", font_micro, COLOR_TEXT_SECONDARY);
    draw_text(dev, graph_x + scale_x(dev, 452), footer_y + scale_y(dev, 8), "20.0 MS", font_small, COLOR_TEXT_PRIMARY);

    fill_rect(dev, scale_x(dev, 48), btn_y, btn_w, btn_h, COLOR_NAV_OFF);
    fill_rect(dev, scale_x(dev, 290), btn_y, btn_w, btn_h,
              (scope->edge == SCOPE_TRIGGER_RISING) ? COLOR_NAV_ON : COLOR_NAV_OFF);
    fill_rect(dev, scale_x(dev, 532), btn_y, btn_w, btn_h,
              scope->running ? COLOR_NAV_ON : COLOR_NAV_OFF);
    stroke_rect(dev, scale_x(dev, 48), btn_y, btn_w, btn_h, border, COLOR_BORDER);
    stroke_rect(dev, scale_x(dev, 290), btn_y, btn_w, btn_h, border, COLOR_BORDER);
    stroke_rect(dev, scale_x(dev, 532), btn_y, btn_w, btn_h, border, COLOR_BORDER);
    draw_text_center(dev, scale_x(dev, 48), btn_y + scale_y(dev, 10), btn_w, "TRIG LEVEL FIXED", font_small, COLOR_TEXT_PRIMARY);
    draw_text_center(dev, scale_x(dev, 290), btn_y + scale_y(dev, 10), btn_w,
                     (scope->edge == SCOPE_TRIGGER_RISING) ? "EDGE RISING" : "EDGE FALLING",
                     font_small, COLOR_TEXT_PRIMARY);
    draw_text_center(dev, scale_x(dev, 532), btn_y + scale_y(dev, 10), btn_w,
                     scope->running ? "FREEZE" : "RUN", font_small, COLOR_TEXT_PRIMARY);
}

static const uint8_t *glyph_for_char(char c)
{
    switch(toupper((unsigned char)c)) {
    case 'A': return GLYPH_A;
    case 'B': return GLYPH_B;
    case 'C': return GLYPH_C;
    case 'D': return GLYPH_D;
    case 'E': return GLYPH_E;
    case 'F': return GLYPH_F;
    case 'G': return GLYPH_G;
    case 'H': return GLYPH_H;
    case 'I': return GLYPH_I;
    case 'J': return GLYPH_J;
    case 'K': return GLYPH_K;
    case 'L': return GLYPH_L;
    case 'M': return GLYPH_M;
    case 'N': return GLYPH_N;
    case 'O': return GLYPH_O;
    case 'P': return GLYPH_P;
    case 'Q': return GLYPH_Q;
    case 'R': return GLYPH_R;
    case 'S': return GLYPH_S;
    case 'T': return GLYPH_T;
    case 'U': return GLYPH_U;
    case 'V': return GLYPH_V;
    case 'W': return GLYPH_W;
    case 'X': return GLYPH_X;
    case 'Y': return GLYPH_Y;
    case 'Z': return GLYPH_Z;
    case '0': return GLYPH_0;
    case '1': return GLYPH_1;
    case '2': return GLYPH_2;
    case '3': return GLYPH_3;
    case '4': return GLYPH_4;
    case '5': return GLYPH_5;
    case '6': return GLYPH_6;
    case '7': return GLYPH_7;
    case '8': return GLYPH_8;
    case '9': return GLYPH_9;
    case '.': return GLYPH_DOT;
    case '-': return GLYPH_MINUS;
    case '/': return GLYPH_SLASH;
    case '%': return GLYPH_PERCENT;
    case ' ': return GLYPH_SPACE;
    default:  return GLYPH_SPACE;
    }
}

static int scale_x(const struct modeset_dev *dev, int x)
{
    return (int)((int64_t)x * (int64_t)dev->width / BASE_W);
}

static int scale_y(const struct modeset_dev *dev, int y)
{
    return (int)((int64_t)y * (int64_t)dev->height / BASE_H);
}

static void put_pixel(struct modeset_dev *dev, int x, int y, uint32_t color)
{
    struct modeset_buf *buf;
    uint8_t *dst;

    if(x < 0 || y < 0 || x >= (int)dev->width || y >= (int)dev->height) {
        return;
    }

    buf = &dev->bufs[dev->draw_buf];
    dst = buf->map + ((size_t)y * buf->stride) + ((size_t)x * 3U);
    dst[0] = (uint8_t)(color & 0xffU);
    dst[1] = (uint8_t)((color >> 8) & 0xffU);
    dst[2] = (uint8_t)((color >> 16) & 0xffU);
}

static void fill_rect(struct modeset_dev *dev, int x, int y, int w, int h, uint32_t color)
{
    int xi;
    int yi;

    if(w <= 0 || h <= 0) {
        return;
    }

    for(yi = 0; yi < h; ++yi) {
        int yy = y + yi;
        if(yy < 0 || yy >= (int)dev->height) {
            continue;
        }

        for(xi = 0; xi < w; ++xi) {
            int xx = x + xi;
            if(xx < 0 || xx >= (int)dev->width) {
                continue;
            }
            put_pixel(dev, xx, yy, color);
        }
    }
}

static void stroke_rect(struct modeset_dev *dev, int x, int y, int w, int h, int thickness, uint32_t color)
{
    fill_rect(dev, x, y, w, thickness, color);
    fill_rect(dev, x, y + h - thickness, w, thickness, color);
    fill_rect(dev, x, y, thickness, h, color);
    fill_rect(dev, x + w - thickness, y, thickness, h, color);
}

static int text_width(const char *text, int scale)
{
    size_t len = strlen(text);
    if(len == 0U) {
        return 0;
    }
    return (int)(len * (size_t)(6 * scale) - (size_t)scale);
}

static void draw_char(struct modeset_dev *dev, int x, int y, char c, int scale, uint32_t color)
{
    const uint8_t *glyph = glyph_for_char(c);
    int row;
    int col;
    int sy;
    int sx;

    for(row = 0; row < 7; ++row) {
        for(col = 0; col < 5; ++col) {
            if(glyph[row] & (1U << (4 - col))) {
                for(sy = 0; sy < scale; ++sy) {
                    for(sx = 0; sx < scale; ++sx) {
                        put_pixel(dev, x + col * scale + sx, y + row * scale + sy, color);
                    }
                }
            }
        }
    }
}

static void draw_text(struct modeset_dev *dev, int x, int y, const char *text, int scale, uint32_t color)
{
    while(*text) {
        draw_char(dev, x, y, *text, scale, color);
        x += 6 * scale;
        ++text;
    }
}

static void draw_text_right(struct modeset_dev *dev, int right, int y, const char *text, int scale, uint32_t color)
{
    draw_text(dev, right - text_width(text, scale), y, text, scale, color);
}

static void draw_text_center(struct modeset_dev *dev, int x, int y, int w, const char *text, int scale, uint32_t color)
{
    int tw = text_width(text, scale);
    draw_text(dev, x + (w - tw) / 2, y, text, scale, color);
}

static void draw_rule(struct modeset_dev *dev, int x, int y, int w, int thickness, uint32_t color)
{
    fill_rect(dev, x, y, w, thickness, color);
}

static void draw_main_page(struct modeset_dev *dev, const monitor_view_t *view)
{
    int shell_x = scale_x(dev, 18);
    int shell_y = scale_y(dev, 18);
    int shell_w = scale_x(dev, 764);
    int shell_h = scale_y(dev, 444);
    int header_x = scale_x(dev, 34);
    int header_y = scale_y(dev, 32);
    int header_w = scale_x(dev, 732);
    int header_h = scale_y(dev, 82);
    int body_x = scale_x(dev, 34);
    int body_y = scale_y(dev, 126);
    int body_w = scale_x(dev, 732);
    int body_h = scale_y(dev, 276);
    int footer_x = scale_x(dev, 34);
    int footer_y = scale_y(dev, 414);
    int footer_w = scale_x(dev, 732);
    int footer_h = scale_y(dev, 36);
    int power_x = scale_x(dev, 48);
    int power_y = scale_y(dev, 142);
    int power_w = scale_x(dev, 392);
    int power_h = scale_y(dev, 244);
    int right_x = scale_x(dev, 456);
    int stat_y = scale_y(dev, 142);
    int stat_w = scale_x(dev, 138);
    int stat_h = scale_y(dev, 92);
    int phase_x = scale_x(dev, 608);
    int params_y = scale_y(dev, 142);
    int params_w = scale_x(dev, 144);
    int params_h = scale_y(dev, 244);
    int tile_y = scale_y(dev, 316);
    int tile_h = scale_y(dev, 70);
    int volt_x = scale_x(dev, 64);
    int tile_w = scale_x(dev, 164);
    int curr_x = scale_x(dev, 244);
    int font_micro = 1;
    int font_small = 2;
    int font_medium = 2;
    int font_large = 3;
    int font_huge = 4;
    int border = (dev->width >= 1024U) ? 3 : 2;
    int nav_gap = scale_x(dev, 12);
    int nav_btn_w = (footer_w - nav_gap * 5) / 4;
    int nav_btn_h = scale_y(dev, 28);
    int nav_btn_y = footer_y + (footer_h - nav_btn_h) / 2;

    fill_rect(dev, 0, 0, (int)dev->width, (int)dev->height, COLOR_BG);

    fill_rect(dev, shell_x, shell_y, shell_w, shell_h, COLOR_SHELL);
    stroke_rect(dev, shell_x, shell_y, shell_w, shell_h, border, COLOR_BORDER);

    fill_rect(dev, header_x, header_y, header_w, header_h, COLOR_PANEL);
    fill_rect(dev, body_x, body_y, body_w, body_h, COLOR_PANEL);
    fill_rect(dev, footer_x, footer_y, footer_w, footer_h, COLOR_PANEL);

    fill_rect(dev, scale_x(dev, 48), scale_y(dev, 42), scale_x(dev, 52), scale_y(dev, 52), COLOR_LOGO_BG);
    stroke_rect(dev, scale_x(dev, 48), scale_y(dev, 42), scale_x(dev, 52), scale_y(dev, 52), border, COLOR_BORDER);
    draw_text(dev, scale_x(dev, 66), scale_y(dev, 59), "P", font_huge, COLOR_LOGO_TEXT);
    draw_text(dev, scale_x(dev, 118), scale_y(dev, 44), "POWER ANALYZER", font_large, COLOR_TEXT_HEADING);
    draw_text(dev, scale_x(dev, 120), scale_y(dev, 76), "LIVE POWER MEASUREMENT CONSOLE", font_micro, COLOR_TEXT_MUTED);
    draw_rule(dev, scale_x(dev, 118), scale_y(dev, 94), scale_x(dev, 250), border, COLOR_DIVIDER);

    fill_rect(dev, scale_x(dev, 596), scale_y(dev, 46), scale_x(dev, 72), scale_y(dev, 22), COLOR_ACCENT_GREEN);
    draw_text_center(dev, scale_x(dev, 596), scale_y(dev, 51), scale_x(dev, 72), "LIVE", font_micro, COLOR_BG);
    fill_rect(dev, scale_x(dev, 682), scale_y(dev, 46), scale_x(dev, 68), scale_y(dev, 22), COLOR_CARD_ALT);
    stroke_rect(dev, scale_x(dev, 682), scale_y(dev, 46), scale_x(dev, 68), scale_y(dev, 22), border, COLOR_BORDER);
    draw_text_center(dev, scale_x(dev, 682), scale_y(dev, 51), scale_x(dev, 68), "PAGE 1", font_micro, COLOR_TEXT_PRIMARY);
    draw_text(dev, scale_x(dev, 596), scale_y(dev, 82), "PURE DRM RENDERER", font_micro, COLOR_ACCENT_BLUE);

    fill_rect(dev, power_x, power_y, power_w, power_h, COLOR_CARD);
    stroke_rect(dev, power_x, power_y, power_w, power_h, border, COLOR_BORDER);
    draw_text(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 14), "POWER METRICS", font_small, COLOR_TEXT_PRIMARY);
    draw_text(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 34), "real-time power summary", font_micro, COLOR_TEXT_SECONDARY);
    draw_rule(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 52), power_w - scale_x(dev, 32), border, COLOR_DIVIDER);

    draw_text(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 72), "ACTIVE", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, power_x + power_w - scale_x(dev, 16), power_y + scale_y(dev, 66), view->p, font_large, COLOR_TEXT_PRIMARY);
    draw_rule(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 98), power_w - scale_x(dev, 32), border, COLOR_DIVIDER);

    draw_text(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 118), "APPARENT", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, power_x + power_w - scale_x(dev, 16), power_y + scale_y(dev, 112), view->s, font_large, COLOR_TEXT_PRIMARY);
    draw_rule(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 144), power_w - scale_x(dev, 32), border, COLOR_DIVIDER);

    draw_text(dev, power_x + scale_x(dev, 16), power_y + scale_y(dev, 150), "REACTIVE", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, power_x + power_w - scale_x(dev, 16), power_y + scale_y(dev, 144), view->q, font_large, COLOR_TEXT_PRIMARY);

    fill_rect(dev, volt_x, tile_y, tile_w, tile_h, COLOR_CARD_ALT);
    fill_rect(dev, curr_x, tile_y, tile_w, tile_h, COLOR_CARD_ALT);
    stroke_rect(dev, volt_x, tile_y, tile_w, tile_h, border, COLOR_BORDER);
    stroke_rect(dev, curr_x, tile_y, tile_w, tile_h, border, COLOR_BORDER);

    draw_text(dev, volt_x + scale_x(dev, 12), tile_y + scale_y(dev, 10), "VOLTAGE RMS", font_micro, COLOR_TEXT_SECONDARY);
    draw_text(dev, volt_x + scale_x(dev, 12), tile_y + scale_y(dev, 34), view->v0_rms_value, font_medium, COLOR_TEXT_PRIMARY);
    draw_text(dev, volt_x + tile_w - scale_x(dev, 18), tile_y + scale_y(dev, 37), "V", font_small, COLOR_ACCENT_AMBER);

    draw_text(dev, curr_x + scale_x(dev, 12), tile_y + scale_y(dev, 10), "CURRENT RMS", font_micro, COLOR_TEXT_SECONDARY);
    draw_text(dev, curr_x + scale_x(dev, 12), tile_y + scale_y(dev, 34), view->i4_rms_value, font_medium, COLOR_TEXT_PRIMARY);
    draw_text(dev, curr_x + tile_w - scale_x(dev, 18), tile_y + scale_y(dev, 37), "A", font_small, COLOR_ACCENT_BLUE);

    fill_rect(dev, right_x, stat_y, stat_w, stat_h, COLOR_CARD);
    fill_rect(dev, phase_x, stat_y, stat_w, stat_h, COLOR_CARD);
    stroke_rect(dev, right_x, stat_y, stat_w, stat_h, border, COLOR_BORDER);
    stroke_rect(dev, phase_x, stat_y, stat_w, stat_h, border, COLOR_BORDER);

    draw_text(dev, right_x + scale_x(dev, 12), stat_y + scale_y(dev, 12), "FREQUENCY", font_micro, COLOR_TEXT_SECONDARY);
    draw_text(dev, right_x + scale_x(dev, 12), stat_y + scale_y(dev, 42), view->freq_value, font_medium, COLOR_ACCENT_BLUE);
    draw_text(dev, right_x + scale_x(dev, 12), stat_y + scale_y(dev, 66), "KHZ", font_micro, COLOR_ACCENT_BLUE);

    draw_text(dev, phase_x + scale_x(dev, 12), stat_y + scale_y(dev, 12), "PHASE", font_micro, COLOR_TEXT_SECONDARY);
    draw_text(dev, phase_x + scale_x(dev, 12), stat_y + scale_y(dev, 42), view->phi_value, font_medium, COLOR_ACCENT_PURPLE);
    draw_text(dev, phase_x + scale_x(dev, 12), stat_y + scale_y(dev, 66), "DEG", font_micro, COLOR_ACCENT_PURPLE);

    fill_rect(dev, right_x, stat_y + scale_y(dev, 106), scale_x(dev, 296), scale_y(dev, 138), COLOR_CARD);
    stroke_rect(dev, right_x, stat_y + scale_y(dev, 106), scale_x(dev, 296), scale_y(dev, 138), border, COLOR_BORDER);
    draw_text(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 120), "KEY PARAMETERS", font_small, COLOR_TEXT_PRIMARY);
    draw_rule(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 140), scale_x(dev, 268), border, COLOR_DIVIDER);

    draw_text(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 160), "POWER FACTOR", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, right_x + scale_x(dev, 282), stat_y + scale_y(dev, 156), view->pf, font_small, COLOR_TEXT_PRIMARY);
    draw_rule(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 178), scale_x(dev, 268), border, COLOR_DIVIDER);

    draw_text(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 194), "VOLTAGE PEAK", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, right_x + scale_x(dev, 282), stat_y + scale_y(dev, 190), view->v0_peak, font_small, COLOR_TEXT_PRIMARY);
    draw_rule(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 212), scale_x(dev, 268), border, COLOR_DIVIDER);

    draw_text(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 228), "CURRENT PEAK", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, right_x + scale_x(dev, 282), stat_y + scale_y(dev, 224), view->i4_peak, font_small, COLOR_TEXT_PRIMARY);
    draw_rule(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 246), scale_x(dev, 268), border, COLOR_DIVIDER);

    draw_text(dev, right_x + scale_x(dev, 14), stat_y + scale_y(dev, 262), "LOAD MODE", font_micro, COLOR_TEXT_SECONDARY);
    draw_text_right(dev, right_x + scale_x(dev, 282), stat_y + scale_y(dev, 258), view->mode, font_small, COLOR_ACCENT_GREEN);

    fill_rect(dev, footer_x + nav_gap, nav_btn_y, nav_btn_w, nav_btn_h, COLOR_NAV_ON);
    fill_rect(dev, footer_x + nav_gap * 2 + nav_btn_w, nav_btn_y, nav_btn_w, nav_btn_h, COLOR_NAV_OFF);
    fill_rect(dev, footer_x + nav_gap * 3 + nav_btn_w * 2, nav_btn_y, nav_btn_w, nav_btn_h, COLOR_NAV_OFF);
    fill_rect(dev, footer_x + nav_gap * 4 + nav_btn_w * 3, nav_btn_y, nav_btn_w, nav_btn_h, COLOR_NAV_OFF);
    stroke_rect(dev, footer_x + nav_gap, nav_btn_y, nav_btn_w, nav_btn_h, border, COLOR_BORDER);
    stroke_rect(dev, footer_x + nav_gap * 2 + nav_btn_w, nav_btn_y, nav_btn_w, nav_btn_h, border, COLOR_BORDER);
    stroke_rect(dev, footer_x + nav_gap * 3 + nav_btn_w * 2, nav_btn_y, nav_btn_w, nav_btn_h, border, COLOR_BORDER);
    stroke_rect(dev, footer_x + nav_gap * 4 + nav_btn_w * 3, nav_btn_y, nav_btn_w, nav_btn_h, border, COLOR_BORDER);

    draw_text_center(dev, footer_x + nav_gap, nav_btn_y + scale_y(dev, 8), nav_btn_w, "MAIN", font_small, COLOR_TEXT_HEADING);
    draw_text_center(dev, footer_x + nav_gap * 2 + nav_btn_w, nav_btn_y + scale_y(dev, 8), nav_btn_w, "FFT", font_small, COLOR_TEXT_PRIMARY);
    draw_text_center(dev, footer_x + nav_gap * 3 + nav_btn_w * 2, nav_btn_y + scale_y(dev, 8), nav_btn_w, "HOLD", font_small, COLOR_TEXT_PRIMARY);
    draw_text_center(dev, footer_x + nav_gap * 4 + nav_btn_w * 3, nav_btn_y + scale_y(dev, 8), nav_btn_w, "SCOPE", font_small, COLOR_TEXT_PRIMARY);
}

static void draw_footer_nav(struct modeset_dev *dev, ui_page_t active_page)
{
    static const char *nav_names[UI_PAGE_COUNT] = { "MAIN", "FFT", "HOLD", "SCOPE" };
    int footer_x = scale_x(dev, 34);
    int footer_y = scale_y(dev, 414);
    int footer_w = scale_x(dev, 732);
    int footer_h = scale_y(dev, 36);
    int nav_gap = scale_x(dev, 12);
    int nav_btn_w = (footer_w - nav_gap * 5) / 4;
    int nav_btn_h = scale_y(dev, 28);
    int nav_btn_y = footer_y + (footer_h - nav_btn_h) / 2;
    int font_small = 2;
    int border = (dev->width >= 1024U) ? 3 : 2;
    uint32_t i;

    fill_rect(dev, footer_x, footer_y, footer_w, footer_h, COLOR_PANEL);

    for(i = 0; i < UI_PAGE_COUNT; i++) {
        int x = footer_x + nav_gap * (int)(i + 1U) + nav_btn_w * (int)i;
        int active = ((ui_page_t)i == active_page);
        fill_rect(dev, x, nav_btn_y, nav_btn_w, nav_btn_h, active ? COLOR_NAV_ON : COLOR_NAV_OFF);
        stroke_rect(dev, x, nav_btn_y, nav_btn_w, nav_btn_h, border, COLOR_BORDER);
        draw_text_center(dev, x, nav_btn_y + scale_y(dev, 8), nav_btn_w, nav_names[i], font_small,
                         active ? COLOR_TEXT_HEADING : COLOR_TEXT_PRIMARY);
    }
}

static void draw_page_chrome(struct modeset_dev *dev, ui_page_t active_page,
                             const char *page_text, const char *subtitle)
{
    int shell_x = scale_x(dev, 18);
    int shell_y = scale_y(dev, 18);
    int shell_w = scale_x(dev, 764);
    int shell_h = scale_y(dev, 444);
    int header_x = scale_x(dev, 34);
    int header_y = scale_y(dev, 32);
    int header_w = scale_x(dev, 732);
    int header_h = scale_y(dev, 82);
    int body_x = scale_x(dev, 34);
    int body_y = scale_y(dev, 126);
    int body_w = scale_x(dev, 732);
    int body_h = scale_y(dev, 276);
    int font_micro = 1;
    int font_large = 3;
    int font_huge = 4;
    int border = (dev->width >= 1024U) ? 3 : 2;

    fill_rect(dev, 0, 0, (int)dev->width, (int)dev->height, COLOR_BG);
    fill_rect(dev, shell_x, shell_y, shell_w, shell_h, COLOR_SHELL);
    stroke_rect(dev, shell_x, shell_y, shell_w, shell_h, border, COLOR_BORDER);
    fill_rect(dev, header_x, header_y, header_w, header_h, COLOR_PANEL);
    fill_rect(dev, body_x, body_y, body_w, body_h, COLOR_PANEL);

    fill_rect(dev, scale_x(dev, 48), scale_y(dev, 42), scale_x(dev, 52), scale_y(dev, 52), COLOR_LOGO_BG);
    stroke_rect(dev, scale_x(dev, 48), scale_y(dev, 42), scale_x(dev, 52), scale_y(dev, 52), border, COLOR_BORDER);
    draw_text(dev, scale_x(dev, 66), scale_y(dev, 59), "P", font_huge, COLOR_LOGO_TEXT);
    draw_text(dev, scale_x(dev, 118), scale_y(dev, 44), "POWER ANALYZER", font_large, COLOR_TEXT_HEADING);
    draw_text(dev, scale_x(dev, 120), scale_y(dev, 76), subtitle, font_micro, COLOR_TEXT_MUTED);
    draw_rule(dev, scale_x(dev, 118), scale_y(dev, 94), scale_x(dev, 250), border, COLOR_DIVIDER);

    fill_rect(dev, scale_x(dev, 596), scale_y(dev, 46), scale_x(dev, 72), scale_y(dev, 22), COLOR_ACCENT_GREEN);
    draw_text_center(dev, scale_x(dev, 596), scale_y(dev, 51), scale_x(dev, 72), "LIVE", font_micro, COLOR_BG);
    fill_rect(dev, scale_x(dev, 682), scale_y(dev, 46), scale_x(dev, 68), scale_y(dev, 22), COLOR_CARD_ALT);
    stroke_rect(dev, scale_x(dev, 682), scale_y(dev, 46), scale_x(dev, 68), scale_y(dev, 22), border, COLOR_BORDER);
    draw_text_center(dev, scale_x(dev, 682), scale_y(dev, 51), scale_x(dev, 68), page_text, font_micro, COLOR_TEXT_PRIMARY);
    draw_text(dev, scale_x(dev, 596), scale_y(dev, 82), "PURE DRM RENDERER", font_micro, COLOR_ACCENT_BLUE);

    draw_footer_nav(dev, active_page);
}

static const char *fft_value_at(const fft_view_t *view, uint8_t tab, uint32_t index)
{
    if(tab == 0U) {
        switch(index) {
        case 0: return view->voltage_h1;
        case 1: return view->voltage_h2;
        case 2: return view->voltage_h3;
        case 3: return view->voltage_h4;
        case 4: return view->voltage_h5;
        case 5: return view->voltage_h6;
        default: return view->voltage_h7;
        }
    }

    switch(index) {
    case 0: return view->current_h1;
    case 1: return view->current_h2;
    case 2: return view->current_h3;
    case 3: return view->current_h4;
    case 4: return view->current_h5;
    case 5: return view->current_h6;
    default: return view->current_h7;
    }
}

static void draw_fft_page(struct modeset_dev *dev, const fft_view_t *view, uint8_t tab)
{
    static const char *harmonic_names[FFT_HARMONIC_COUNT] = { "H1", "H2", "H3", "H4", "H5", "H6", "H7" };
    const uint16_t *bars = (tab == 0U) ? view->voltage_bar_height : view->current_bar_height;
    const char *metric_title = (tab == 0U) ? "THD VOLTAGE" : "THD CURRENT";
    const char *metric_value = (tab == 0U) ? view->voltage_thd : view->current_thd;
    const char *metric_tag = (tab == 0U) ? "VOLTAGE HARMONICS" : "CURRENT HARMONICS";
    int shell_x = scale_x(dev, 48);
    int shell_y = scale_y(dev, 142);
    int shell_w = scale_x(dev, 704);
    int shell_h = scale_y(dev, 244);
    int graph_x = scale_x(dev, 64);
    int graph_y = scale_y(dev, 194);
    int graph_w = scale_x(dev, 672);
    int graph_h = scale_y(dev, 174);
    int baseline_y = scale_y(dev, 350);
    int bar_max_h = scale_y(dev, 120);
    int bar_min_h = scale_y(dev, FFT_BAR_MIN_BASE);
    int bar_w = scale_x(dev, 48);
    int slot_w = scale_x(dev, 94);
    int first_x = scale_x(dev, 82);
    int font_micro = 1;
    int font_small = 2;
    int font_medium = 2;
    int border = (dev->width >= 1024U) ? 3 : 2;
    uint32_t i;

    draw_page_chrome(dev, UI_PAGE_FFT, "PAGE 2", "HARMONIC SPECTRUM ANALYSIS");

    fill_rect(dev, shell_x, shell_y, shell_w, shell_h, COLOR_CARD);
    stroke_rect(dev, shell_x, shell_y, shell_w, shell_h, border, COLOR_BORDER);
    draw_text(dev, shell_x + scale_x(dev, 16), shell_y + scale_y(dev, 14), metric_title, font_small, COLOR_TEXT_SECONDARY);
    draw_text(dev, shell_x + scale_x(dev, 160), shell_y + scale_y(dev, 10), metric_value, font_medium, COLOR_TEXT_PRIMARY);
    fill_rect(dev, shell_x + scale_x(dev, 300), shell_y + scale_y(dev, 10), scale_x(dev, 170), scale_y(dev, 24), COLOR_ACCENT_AMBER);
    draw_text_center(dev, shell_x + scale_x(dev, 300), shell_y + scale_y(dev, 16), scale_x(dev, 170), metric_tag, font_micro, COLOR_BG);

    fill_rect(dev, shell_x + scale_x(dev, 516), shell_y + scale_y(dev, 8), scale_x(dev, 92), scale_y(dev, 30),
              (tab == 0U) ? COLOR_NAV_ON : COLOR_NAV_OFF);
    fill_rect(dev, shell_x + scale_x(dev, 616), shell_y + scale_y(dev, 8), scale_x(dev, 92), scale_y(dev, 30),
              (tab == 1U) ? COLOR_NAV_ON : COLOR_NAV_OFF);
    stroke_rect(dev, shell_x + scale_x(dev, 516), shell_y + scale_y(dev, 8), scale_x(dev, 92), scale_y(dev, 30), border, COLOR_BORDER);
    stroke_rect(dev, shell_x + scale_x(dev, 616), shell_y + scale_y(dev, 8), scale_x(dev, 92), scale_y(dev, 30), border, COLOR_BORDER);
    draw_text_center(dev, shell_x + scale_x(dev, 516), shell_y + scale_y(dev, 17), scale_x(dev, 92), "VOLTAGE", font_micro,
                     (tab == 0U) ? COLOR_TEXT_HEADING : COLOR_TEXT_PRIMARY);
    draw_text_center(dev, shell_x + scale_x(dev, 616), shell_y + scale_y(dev, 17), scale_x(dev, 92), "CURRENT", font_micro,
                     (tab == 1U) ? COLOR_TEXT_HEADING : COLOR_TEXT_PRIMARY);

    fill_rect(dev, graph_x, graph_y, graph_w, graph_h, COLOR_CARD_ALT);
    stroke_rect(dev, graph_x, graph_y, graph_w, graph_h, border, COLOR_BORDER);
    draw_rule(dev, graph_x + scale_x(dev, 14), baseline_y, graph_w - scale_x(dev, 28), border, COLOR_DIVIDER);

    for(i = 0; i < FFT_HARMONIC_COUNT; i++) {
        uint16_t raw_h = bars[i];
        int bar_h;
        int x = first_x + slot_w * (int)i;
        int y;

        if(raw_h > FFT_BAR_SRC_MAX) {
            raw_h = FFT_BAR_SRC_MAX;
        }
        bar_h = (int)((uint32_t)raw_h * (uint32_t)bar_max_h / FFT_BAR_SRC_MAX);
        if(bar_h < bar_min_h) {
            bar_h = bar_min_h;
        }
        y = baseline_y - bar_h;

        fill_rect(dev, x, y, bar_w, bar_h, (tab == 0U) ? COLOR_ACCENT_BLUE : COLOR_ACCENT_GREEN);
        stroke_rect(dev, x, y, bar_w, bar_h, border, COLOR_BORDER);
        draw_text_center(dev, x - scale_x(dev, 22), y - scale_y(dev, 14), scale_x(dev, 92),
                         fft_value_at(view, tab, i), font_micro, COLOR_TEXT_PRIMARY);
        draw_text_center(dev, x, baseline_y + scale_y(dev, 8), bar_w, harmonic_names[i], font_micro, COLOR_TEXT_MUTED);
    }
}

static void draw_hold_page(struct modeset_dev *dev, const hold_view_t *view)
{
    static const char *hold_titles[8] = {
        "V RMS MAX", "V RMS MIN", "I RMS MAX", "I RMS MIN",
        "P MAX", "FREQ MAX", "FREQ MIN", "EVENT COUNT"
    };
    const char *hold_values[8];
    int card_x = scale_x(dev, 48);
    int card_y = scale_y(dev, 142);
    int card_w = scale_x(dev, 704);
    int card_h = scale_y(dev, 204);
    int btn_y = scale_y(dev, 356);
    int btn_w = scale_x(dev, 220);
    int btn_h = scale_y(dev, 34);
    int font_micro = 1;
    int font_small = 2;
    int border = (dev->width >= 1024U) ? 3 : 2;
    uint32_t i;

    hold_values[0] = view->vrms_max;
    hold_values[1] = view->vrms_min;
    hold_values[2] = view->irms_max;
    hold_values[3] = view->irms_min;
    hold_values[4] = view->p_max;
    hold_values[5] = view->freq_max;
    hold_values[6] = view->freq_min;
    hold_values[7] = view->event_count;

    draw_page_chrome(dev, UI_PAGE_HOLD, "PAGE 3", "HOLD RECORD AND LIMIT TRACKING");

    fill_rect(dev, card_x, card_y, card_w, card_h, COLOR_CARD);
    stroke_rect(dev, card_x, card_y, card_w, card_h, border, COLOR_BORDER);
    draw_text(dev, card_x + scale_x(dev, 16), card_y + scale_y(dev, 14), "HOLD RECORD", font_small, COLOR_TEXT_PRIMARY);
    if(view->frozen) {
        fill_rect(dev, card_x + scale_x(dev, 560), card_y + scale_y(dev, 12), scale_x(dev, 110), scale_y(dev, 24), COLOR_ACCENT_GREEN);
        draw_text_center(dev, card_x + scale_x(dev, 560), card_y + scale_y(dev, 18), scale_x(dev, 110), "FROZEN", font_micro, COLOR_BG);
    }

    for(i = 0; i < 8U; i++) {
        int y = card_y + scale_y(dev, 44 + (int)i * 20);
        draw_text(dev, card_x + scale_x(dev, 22), y, hold_titles[i], font_micro, COLOR_TEXT_SECONDARY);
        draw_text_right(dev, card_x + card_w - scale_x(dev, 28), y - scale_y(dev, 2), hold_values[i], font_small, COLOR_TEXT_PRIMARY);
        if(i < 7U) {
            draw_rule(dev, card_x + scale_x(dev, 20), y + scale_y(dev, 14), card_w - scale_x(dev, 40), border, COLOR_DIVIDER);
        }
    }

    fill_rect(dev, scale_x(dev, 48), btn_y, btn_w, btn_h, COLOR_NAV_OFF);
    fill_rect(dev, scale_x(dev, 290), btn_y, btn_w, btn_h, COLOR_NAV_OFF);
    fill_rect(dev, scale_x(dev, 532), btn_y, btn_w, btn_h, view->frozen ? COLOR_NAV_ON : COLOR_NAV_OFF);
    stroke_rect(dev, scale_x(dev, 48), btn_y, btn_w, btn_h, border, COLOR_BORDER);
    stroke_rect(dev, scale_x(dev, 290), btn_y, btn_w, btn_h, border, COLOR_BORDER);
    stroke_rect(dev, scale_x(dev, 532), btn_y, btn_w, btn_h, border, COLOR_BORDER);
    draw_text_center(dev, scale_x(dev, 48), btn_y + scale_y(dev, 10), btn_w, "CLEAR MAX", font_small, COLOR_TEXT_PRIMARY);
    draw_text_center(dev, scale_x(dev, 290), btn_y + scale_y(dev, 10), btn_w, "CLEAR MIN", font_small, COLOR_TEXT_PRIMARY);
    draw_text_center(dev, scale_x(dev, 532), btn_y + scale_y(dev, 10), btn_w,
                     view->frozen ? "UNFREEZE" : "FREEZE HOLD", font_small, COLOR_TEXT_PRIMARY);
}

static void draw_page(struct modeset_dev *dev, const ui_state_t *ui, const monitor_view_t *view)
{
    switch(ui->page) {
    case UI_PAGE_FFT:
        draw_fft_page(dev, &ui->fft_view, ui->fft_tab);
        break;
    case UI_PAGE_HOLD:
        draw_hold_page(dev, &ui->hold_view);
        break;
    case UI_PAGE_SCOPE:
        draw_scope_page(dev, &ui->scope_view);
        break;
    case UI_PAGE_MAIN:
    default:
        draw_main_page(dev, view);
        break;
    }
}

static uint32_t ticks_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec * 1000ULL) + (ts.tv_nsec / 1000000ULL));
}

static int input_has_touch_axes(int input_fd)
{
    unsigned long ev_bits[NBITS_LOCAL(EV_MAX)] = { 0UL };
    unsigned long abs_bits[NBITS_LOCAL(ABS_MAX)] = { 0UL };

    if(ioctl(input_fd, EVIOCGBIT(0, sizeof(ev_bits)), ev_bits) < 0) {
        return 0;
    }
    if(!test_bit_local(EV_ABS, ev_bits)) {
        return 0;
    }
    if(ioctl(input_fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        return 0;
    }

    return (test_bit_local(ABS_X, abs_bits) && test_bit_local(ABS_Y, abs_bits)) ||
           (test_bit_local(ABS_MT_POSITION_X, abs_bits) && test_bit_local(ABS_MT_POSITION_Y, abs_bits));
}

static void input_load_abs_range(int input_fd, ui_state_t *ui)
{
    struct input_absinfo abs_x;
    struct input_absinfo abs_y;

    ui->input_x_min = 0;
    ui->input_x_max = BASE_W - 1;
    ui->input_y_min = 0;
    ui->input_y_max = BASE_H - 1;

    if(ioctl(input_fd, EVIOCGABS(ABS_MT_POSITION_X), &abs_x) == 0 &&
       ioctl(input_fd, EVIOCGABS(ABS_MT_POSITION_Y), &abs_y) == 0) {
        ui->input_x_min = abs_x.minimum;
        ui->input_x_max = abs_x.maximum;
        ui->input_y_min = abs_y.minimum;
        ui->input_y_max = abs_y.maximum;
        return;
    }

    if(ioctl(input_fd, EVIOCGABS(ABS_X), &abs_x) == 0 &&
       ioctl(input_fd, EVIOCGABS(ABS_Y), &abs_y) == 0) {
        ui->input_x_min = abs_x.minimum;
        ui->input_x_max = abs_x.maximum;
        ui->input_y_min = abs_y.minimum;
        ui->input_y_max = abs_y.maximum;
    }
}

static int input_open_path(ui_state_t *ui, const char *path)
{
    int input_fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);

    if(input_fd < 0) {
        return -1;
    }

    if(!input_has_touch_axes(input_fd)) {
        close(input_fd);
        return -1;
    }

    input_load_abs_range(input_fd, ui);
    fprintf(stderr, "drm_page1: touch input %s range x=%d..%d y=%d..%d\n",
            path, ui->input_x_min, ui->input_x_max, ui->input_y_min, ui->input_y_max);
    return input_fd;
}

static void ui_input_init(ui_state_t *ui)
{
    const char *env_path = getenv(UI_INPUT_ENV);
    int i;

    ui->input_fd = -1;

    if(env_path != NULL && env_path[0] != '\0') {
        ui->input_fd = input_open_path(ui, env_path);
        if(ui->input_fd < 0) {
            fprintf(stderr, "drm_page1: cannot use %s=%s for touch input\n", UI_INPUT_ENV, env_path);
        }
        return;
    }

    for(i = 0; i < 16; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        ui->input_fd = input_open_path(ui, path);
        if(ui->input_fd >= 0) {
            return;
        }
    }

    fprintf(stderr, "drm_page1: no touch input opened, set %s=/dev/input/eventX if needed\n", UI_INPUT_ENV);
}

static int input_abs_to_screen(int value, int min_value, int max_value, int screen_size)
{
    int range = max_value - min_value;
    int clamped = clamp_int(value, min_value, max_value);

    if(range <= 0 || screen_size <= 1) {
        return 0;
    }

    return (int)(((int64_t)(clamped - min_value) * (int64_t)(screen_size - 1)) / range);
}

static int hit_rect_base(int x, int y, int rx, int ry, int rw, int rh)
{
    return x >= rx && x < (rx + rw) && y >= ry && y < (ry + rh);
}

static void ui_handle_touch_base(ui_state_t *ui, int x, int y, const live_metrics_t *metrics)
{
    const int nav_x0 = 46;
    const int nav_y = 416;
    const int nav_gap = 12;
    const int nav_btn_w = (732 - nav_gap * 5) / 4;

    if(hit_rect_base(x, y, nav_x0, nav_y, nav_btn_w, 28)) {
        ui->page = UI_PAGE_MAIN;
        ui->dirty = 1;
        return;
    }
    if(hit_rect_base(x, y, nav_x0 + nav_gap + nav_btn_w, nav_y, nav_btn_w, 28)) {
        ui->page = UI_PAGE_FFT;
        ui->dirty = 1;
        return;
    }
    if(hit_rect_base(x, y, nav_x0 + nav_gap * 2 + nav_btn_w * 2, nav_y, nav_btn_w, 28)) {
        ui->page = UI_PAGE_HOLD;
        ui->dirty = 1;
        return;
    }
    if(hit_rect_base(x, y, nav_x0 + nav_gap * 3 + nav_btn_w * 3, nav_y, nav_btn_w, 28)) {
        ui->page = UI_PAGE_SCOPE;
        ui->dirty = 1;
        return;
    }

    if(ui->page == UI_PAGE_FFT) {
        if(hit_rect_base(x, y, 564, 150, 92, 30)) {
            ui->fft_tab = 0U;
            ui->dirty = 1;
            return;
        }
        if(hit_rect_base(x, y, 664, 150, 92, 30)) {
            ui->fft_tab = 1U;
            ui->dirty = 1;
            return;
        }
    }

    if(ui->page == UI_PAGE_HOLD) {
        if(hit_rect_base(x, y, 48, 356, 220, 34)) {
            hold_apply_actions(&ui->hold_state, metrics, POWER_UI_HOLD_ACTION_CLEAR_MAX);
            hold_build_view(&ui->hold_state, &ui->hold_view);
            ui->dirty = 1;
            return;
        }
        if(hit_rect_base(x, y, 290, 356, 220, 34)) {
            hold_apply_actions(&ui->hold_state, metrics, POWER_UI_HOLD_ACTION_CLEAR_MIN);
            hold_build_view(&ui->hold_state, &ui->hold_view);
            ui->dirty = 1;
            return;
        }
        if(hit_rect_base(x, y, 532, 356, 220, 34)) {
            ui->hold_state.frozen = ui->hold_state.frozen ? 0 : 1;
            hold_build_view(&ui->hold_state, &ui->hold_view);
            ui->dirty = 1;
        }
    }

    if(ui->page == UI_PAGE_SCOPE) {
        if(hit_rect_base(x, y, 532, 356, 220, 34)) {
            ui->scope_view.running = ui->scope_view.running ? 0 : 1;
            ui->dirty = 1;
            return;
        }
        if(hit_rect_base(x, y, 290, 356, 220, 34)) {
            ui->scope_view.edge = (ui->scope_view.edge == SCOPE_TRIGGER_RISING) ?
                                  SCOPE_TRIGGER_FALLING : SCOPE_TRIGGER_RISING;
            ui->dirty = 1;
        }
    }
}

static void ui_poll_input(ui_state_t *ui, const struct modeset_dev *dev, const live_metrics_t *metrics)
{
    struct input_event ev;
    int current_down = ui->touch_down;

    if(ui->input_fd < 0) {
        return;
    }

    while(read(ui->input_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
        if(ev.type == EV_ABS) {
            if(ev.code == ABS_X || ev.code == ABS_MT_POSITION_X) {
                ui->input_raw_x = ev.value;
                ui->input_have_x = 1;
            }
            else if(ev.code == ABS_Y || ev.code == ABS_MT_POSITION_Y) {
                ui->input_raw_y = ev.value;
                ui->input_have_y = 1;
            }
            else if(ev.code == ABS_PRESSURE) {
                current_down = ev.value > 0;
            }
            else if(ev.code == ABS_MT_TRACKING_ID) {
                current_down = ev.value >= 0;
            }
        }
        else if(ev.type == EV_KEY && ev.code == BTN_TOUCH) {
            current_down = ev.value != 0;
        }
        else if(ev.type == EV_SYN && ev.code == SYN_REPORT) {
            if(current_down && !ui->touch_down && ui->input_have_x && ui->input_have_y) {
                int sx = input_abs_to_screen(ui->input_raw_x, ui->input_x_min, ui->input_x_max, (int)dev->width);
                int sy = input_abs_to_screen(ui->input_raw_y, ui->input_y_min, ui->input_y_max, (int)dev->height);
                int bx = (int)((int64_t)sx * BASE_W / (int64_t)dev->width);
                int by = (int)((int64_t)sy * BASE_H / (int64_t)dev->height);
                ui_handle_touch_base(ui, bx, by, metrics);
            }
            ui->touch_down = current_down;
        }
    }
}

static int modeset_open(int *out, const char *node)
{
    int ret;
    uint64_t has_dumb;

    fd = open(node, O_RDWR | O_CLOEXEC);
    if(fd < 0) {
        ret = -errno;
        fprintf(stderr, "cannot open '%s': %m\n", node);
        return ret;
    }

    if(drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &has_dumb) < 0 || !has_dumb) {
        fprintf(stderr, "drm device '%s' does not support dumb buffers\n", node);
        close(fd);
        return -EOPNOTSUPP;
    }

    *out = fd;
    return 0;
}

static int modeset_prepare(int fd)
{
    drmModeRes *res;
    drmModeConnector *conn;
    unsigned int i;
    struct modeset_dev *dev;
    int ret;

    res = drmModeGetResources(fd);
    if(!res) {
        fprintf(stderr, "cannot retrieve DRM resources (%d): %m\n", errno);
        return -errno;
    }

    for(i = 0; i < res->count_connectors; ++i) {
        conn = drmModeGetConnector(fd, res->connectors[i]);
        if(!conn) {
            fprintf(stderr, "cannot retrieve DRM connector %u:%u (%d): %m\n",
                i, res->connectors[i], errno);
            continue;
        }

        dev = (struct modeset_dev *)malloc(sizeof(*dev));
        memset(dev, 0, sizeof(*dev));
        dev->conn = conn->connector_id;

        ret = modeset_setup_dev(fd, res, conn, dev);
        if(ret) {
            if(ret != -ENOENT) {
                errno = -ret;
                fprintf(stderr, "cannot setup device for connector %u:%u (%d): %m\n",
                    i, res->connectors[i], errno);
            }
            free(dev);
            drmModeFreeConnector(conn);
            continue;
        }

        drmModeFreeConnector(conn);
        dev->next = modeset_list;
        modeset_list = dev;
    }

    drmModeFreeResources(res);
    return 0;
}

static int modeset_setup_dev(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev)
{
    int ret;

    if(conn->connection != DRM_MODE_CONNECTED) {
        fprintf(stderr, "ignoring unused connector %u\n", conn->connector_id);
        return -ENOENT;
    }

    if(conn->count_modes == 0) {
        fprintf(stderr, "no valid mode for connector %u\n", conn->connector_id);
        return -EFAULT;
    }

    memcpy(&dev->mode, &conn->modes[0], sizeof(dev->mode));
    dev->width = conn->modes[0].hdisplay;
    dev->height = conn->modes[0].vdisplay;
    fprintf(stderr, "mode for connector %u is %ux%u\n", conn->connector_id, dev->width, dev->height);

    ret = modeset_find_crtc(fd, res, conn, dev);
    if(ret) {
        fprintf(stderr, "no valid crtc for connector %u\n", conn->connector_id);
        return ret;
    }

    ret = modeset_create_fb(fd, dev);
    if(ret) {
        fprintf(stderr, "cannot create framebuffer for connector %u\n", conn->connector_id);
        return ret;
    }

    return 0;
}

static int modeset_find_crtc(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev)
{
    drmModeEncoder *enc;
    unsigned int i, j;
    int32_t crtc;
    struct modeset_dev *iter;

    if(conn->encoder_id)
        enc = drmModeGetEncoder(fd, conn->encoder_id);
    else
        enc = NULL;

    if(enc) {
        if(enc->crtc_id) {
            crtc = enc->crtc_id;
            for(iter = modeset_list; iter; iter = iter->next) {
                if(iter->crtc == (uint32_t)crtc) {
                    crtc = -1;
                    break;
                }
            }

            if(crtc >= 0) {
                drmModeFreeEncoder(enc);
                dev->crtc = (uint32_t)crtc;
                return 0;
            }
        }

        drmModeFreeEncoder(enc);
    }

    for(i = 0; i < conn->count_encoders; ++i) {
        enc = drmModeGetEncoder(fd, conn->encoders[i]);
        if(!enc) {
            fprintf(stderr, "cannot retrieve encoder %u:%u (%d): %m\n", i, conn->encoders[i], errno);
            continue;
        }

        for(j = 0; j < res->count_crtcs; ++j) {
            if(!(enc->possible_crtcs & (1U << j)))
                continue;

            crtc = (int32_t)res->crtcs[j];
            for(iter = modeset_list; iter; iter = iter->next) {
                if(iter->crtc == (uint32_t)crtc) {
                    crtc = -1;
                    break;
                }
            }

            if(crtc >= 0) {
                drmModeFreeEncoder(enc);
                dev->crtc = (uint32_t)crtc;
                return 0;
            }
        }

        drmModeFreeEncoder(enc);
    }

    fprintf(stderr, "cannot find suitable CRTC for connector %u\n", conn->connector_id);
    return -ENOENT;
}

static int modeset_create_single_fb(int fd, struct modeset_dev *dev, struct modeset_buf *buf)
{
    struct drm_mode_create_dumb creq;
    struct drm_mode_destroy_dumb dreq;
    struct drm_mode_map_dumb mreq;
    int ret;

    memset(&creq, 0, sizeof(creq));
    creq.width = dev->width;
    creq.height = dev->height;
    creq.bpp = 24;
    ret = drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);
    if(ret < 0) {
        fprintf(stderr, "cannot create dumb buffer (%d): %m\n", errno);
        return -errno;
    }
    buf->stride = creq.pitch;
    buf->size = creq.size;
    buf->handle = creq.handle;

    ret = drmModeAddFB(fd, dev->width, dev->height, 24, 24, buf->stride, buf->handle, &buf->fb);
    if(ret) {
        fprintf(stderr, "cannot create framebuffer (%d): %m\n", errno);
        ret = -errno;
        goto err_destroy;
    }

    memset(&mreq, 0, sizeof(mreq));
    mreq.handle = buf->handle;
    ret = drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);
    if(ret) {
        fprintf(stderr, "cannot map dumb buffer (%d): %m\n", errno);
        ret = -errno;
        goto err_fb;
    }

    buf->map = mmap(0, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mreq.offset);
    if(buf->map == MAP_FAILED) {
        fprintf(stderr, "cannot mmap dumb buffer (%d): %m\n", errno);
        ret = -errno;
        goto err_fb;
    }

    memset(buf->map, 0, buf->size);
    return 0;

err_fb:
    drmModeRmFB(fd, buf->fb);
err_destroy:
    memset(&dreq, 0, sizeof(dreq));
    dreq.handle = buf->handle;
    drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    return ret;
}

static int modeset_create_fb(int fd, struct modeset_dev *dev)
{
    int ret;

    memset(dev->bufs, 0, sizeof(dev->bufs));
    ret = modeset_create_single_fb(fd, dev, &dev->bufs[0]);
    if(ret) {
        return ret;
    }

    ret = modeset_create_single_fb(fd, dev, &dev->bufs[1]);
    if(ret) {
        struct drm_mode_destroy_dumb dreq;

        munmap(dev->bufs[0].map, dev->bufs[0].size);
        drmModeRmFB(fd, dev->bufs[0].fb);
        memset(&dreq, 0, sizeof(dreq));
        dreq.handle = dev->bufs[0].handle;
        drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
        memset(dev->bufs, 0, sizeof(dev->bufs));
        return ret;
    }

    dev->front_buf = 0;
    dev->draw_buf = 0;
    return 0;
}

static void modeset_draw(void)
{
    struct modeset_dev *iter;
    monitor_view_t view;
    live_metrics_t metrics;
    ui_state_t ui;
    uint64_t latest_fft_frame[DMA_S2MM_FRAME_WORDS];
    uint64_t latest_scope_frame[DMA_S2MM_FRAME_WORDS];
    int fft_valid = 0;
    int scope_valid = 0;
    uint32_t last_update;
    uint32_t last_scope_rebuild = 0U;
    unsigned long long last_main_dma_completed = 0ULL;
    unsigned long long last_scope_dma_completed = 0ULL;
    int main_dma_warned = 0;
    int scope_dma_warned = 0;

    iter = modeset_list;
    if(!iter) {
        fprintf(stderr, "no active modeset device\n");
        return;
    }

    memset(&metrics, 0, sizeof(metrics));
    memset(latest_fft_frame, 0, sizeof(latest_fft_frame));
    memset(latest_scope_frame, 0, sizeof(latest_scope_frame));
    memset(&ui, 0, sizeof(ui));
    ui.input_fd = -1;
    ui.page = UI_PAGE_MAIN;
    ui.fft_tab = 0U;
    hold_state_defaults(&ui.hold_state);
    fft_view_defaults(&ui.fft_view);
    hold_view_defaults(&ui.hold_view);
    scope_view_defaults(&ui.scope_view);

    if(dma_s2mm_linux_init() != 0) {
        fprintf(stderr, "drm_page1: dma0 startup not active, old pages may stay zero\n");
    }
    if(dma_s2mm_linux_scope_init(SCOPE_DEFAULT_RX_INDEX) != 0) {
        fprintf(stderr, "drm_page1: dma1 startup not active, scope page may stay zero\n");
    }

    power_model_init();
    monitor_view_defaults(&view);
    if(power_model_step(&view, &metrics) != 0) {
        if(dma_s2mm_linux_copy_latest_frame(latest_fft_frame)) {
            fft_valid = 1;
        }
        if(fft_valid) {
            power_model_build_fft_view_from_frame(&ui.fft_view, latest_fft_frame,
                                                  DMA_S2MM_FRAME_WORDS, &metrics);
        }
        else {
            power_model_build_fft_view(&ui.fft_view, &metrics);
        }
        if(dma_s2mm_linux_scope_copy_latest_frame(latest_scope_frame)) {
            scope_valid = scope_build_view_from_dma(&ui.scope_view, latest_scope_frame);
        }
        hold_update(&ui.hold_state, &metrics);
        hold_build_view(&ui.hold_state, &ui.hold_view);
    }
    ui_input_init(&ui);

    iter->draw_buf = iter->front_buf;
    draw_page(iter, &ui, &view);
    last_update = ticks_ms();
    last_scope_rebuild = last_update;
    last_main_dma_completed = dma_s2mm_linux_completed_count();
    last_scope_dma_completed = dma_s2mm_linux_scope_completed_count();

    while(!g_stop_requested) {
        uint32_t now = ticks_ms();
        unsigned long long main_dma_completed = dma_s2mm_linux_completed_count();
        unsigned long long scope_dma_completed = dma_s2mm_linux_scope_completed_count();
        int need_redraw = 0;

        if(dma_s2mm_linux_service() != 0) {
            if(!main_dma_warned) {
                fprintf(stderr, "drm_page1: dma0 service failed, statistics may freeze\n");
                main_dma_warned = 1;
            }
        } else {
            main_dma_warned = 0;
        }

        if(dma_s2mm_linux_scope_service() != 0) {
            if(!scope_dma_warned) {
                fprintf(stderr, "drm_page1: dma1 service failed, scope may freeze\n");
                scope_dma_warned = 1;
            }
        } else {
            scope_dma_warned = 0;
        }

        ui_poll_input(&ui, iter, &metrics);
        if(ui.dirty) {
            if(ui.page == UI_PAGE_SCOPE && ui.scope_view.history_count > 1U) {
                scope_valid = scope_build_view_from_history(&ui.scope_view);
            }
            need_redraw = 1;
            ui.dirty = 0;
        }

        if(now - last_update >= 1000U) {
            if(power_model_step(&view, &metrics) != 0) {
                if(dma_s2mm_linux_copy_latest_frame(latest_fft_frame)) {
                    fft_valid = 1;
                }
                if(fft_valid) {
                    power_model_build_fft_view_from_frame(&ui.fft_view, latest_fft_frame,
                                                          DMA_S2MM_FRAME_WORDS, &metrics);
                }
                else {
                    power_model_build_fft_view(&ui.fft_view, &metrics);
                }
                hold_update(&ui.hold_state, &metrics);
                hold_build_view(&ui.hold_state, &ui.hold_view);
                need_redraw = 1;
            }
            if(main_dma_completed != last_main_dma_completed) {
                last_main_dma_completed = main_dma_completed;
            }
            last_update = now;
        }

        if(ui.scope_view.running && scope_dma_completed != last_scope_dma_completed) {
            if(dma_s2mm_linux_scope_copy_latest_frame(latest_scope_frame)) {
                scope_push_frame(&ui.scope_view, latest_scope_frame);
                if(ui.page == UI_PAGE_SCOPE && (now - last_scope_rebuild) >= 20U) {
                    scope_valid = scope_build_view_from_history(&ui.scope_view);
                    if(scope_valid) {
                        need_redraw = 1;
                    }
                    last_scope_rebuild = now;
                }
            }
            last_scope_dma_completed = scope_dma_completed;
        }

        if(need_redraw) {
            int back_buf = 1 - iter->front_buf;
            int ret;

            iter->draw_buf = back_buf;
            draw_page(iter, &ui, &view);
            ret = drmModeSetCrtc(fd,
                iter->crtc,
                iter->bufs[back_buf].fb,
                0,
                0,
                &iter->conn,
                1,
                &iter->mode);
            if(ret) {
                fprintf(stderr, "swap framebuffer failed for connector %u (%d): %m\n", iter->conn, errno);
            }
            else {
                iter->front_buf = back_buf;
            }
        }

        usleep((ui.page == UI_PAGE_SCOPE && ui.scope_view.running) ? 1000 : 16000);
    }

    if(ui.input_fd >= 0) {
        close(ui.input_fd);
    }
}

static void modeset_cleanup(int fd)
{
    struct modeset_dev *iter;
    struct drm_mode_destroy_dumb dreq;
    int i;

    while(modeset_list) {
        iter = modeset_list;
        modeset_list = iter->next;

        if(iter->saved_crtc) {
            drmModeSetCrtc(fd,
                iter->saved_crtc->crtc_id,
                iter->saved_crtc->buffer_id,
                iter->saved_crtc->x,
                iter->saved_crtc->y,
                &iter->conn,
                1,
                &iter->saved_crtc->mode);
            drmModeFreeCrtc(iter->saved_crtc);
        }

        for(i = 0; i < 2; ++i) {
            if(iter->bufs[i].map && iter->bufs[i].size) {
                munmap(iter->bufs[i].map, iter->bufs[i].size);
            }
            if(iter->bufs[i].fb) {
                drmModeRmFB(fd, iter->bufs[i].fb);
            }
            if(iter->bufs[i].handle) {
                memset(&dreq, 0, sizeof(dreq));
                dreq.handle = iter->bufs[i].handle;
                drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
            }
        }

        free(iter);
    }
}

int main(int argc, char **argv)
{
    int ret;
    const char *card;
    struct modeset_dev *iter;
    sighandler_t sig_ret = NULL;

    if(argc != 2) {
        printf("Usage: drm_page1 lcd|hdmi|/dev/dri/cardX\n");
        card = DRM_PAGE1_CARD;
    }
    else if(strcmp(argv[1], "hdmi") == 0) {
        card = "/dev/dri/card1";
    }
    else if(strcmp(argv[1], "lcd") == 0) {
        card = "/dev/dri/card0";
    }
    else {
        card = argv[1];
    }

    fprintf(stderr, "drm_page1 build: %s\n", DRM_PAGE1_BUILD_TAG);
    fprintf(stderr, "using card '%s'\n", card);

    sig_ret = signal(SIGINT, (sighandler_t)sig_handler);
    if(SIG_ERR == sig_ret) {
        perror("signal error");
        exit(-1);
    }

    ret = modeset_open(&fd, card);
    if(ret)
        goto out_return;

    ret = modeset_prepare(fd);
    if(ret)
        goto out_close;

    for(iter = modeset_list; iter; iter = iter->next) {
        iter->saved_crtc = drmModeGetCrtc(fd, iter->crtc);
        fprintf(stderr,
            "setcrtc try: conn=%u crtc=%u fb=%u mode=%s %ux%u\n",
            iter->conn,
            iter->crtc,
            iter->bufs[iter->front_buf].fb,
            iter->mode.name,
            iter->width,
            iter->height);
        ret = drmModeSetCrtc(fd, iter->crtc, iter->bufs[iter->front_buf].fb, 0, 0, &iter->conn, 1, &iter->mode);
        if(ret)
            fprintf(stderr, "cannot set CRTC for connector %u (%d): %m\n", iter->conn, errno);
    }

    modeset_draw();
    dma_s2mm_linux_scope_shutdown();
    dma_s2mm_linux_shutdown();
    modeset_cleanup(fd);
    ret = 0;

out_close:
    dma_s2mm_linux_scope_shutdown();
    dma_s2mm_linux_shutdown();
    close(fd);
out_return:
    if(ret) {
        errno = -ret;
        fprintf(stderr, "modeset failed with error %d: %m\n", errno);
    }
    else {
        fprintf(stderr, "exiting\n");
    }
    return ret;
}
