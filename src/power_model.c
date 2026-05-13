#include "power_model.h"
#include "amp_comp.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#ifndef TD_STATS_BASE_ADDR
#define TD_STATS_BASE_ADDR 0x40000000UL
#endif

#ifndef PLL_PHASE_BASE_ADDR
#define PLL_PHASE_BASE_ADDR 0x40010000UL
#endif

#ifndef POWER_MODEL_MAP_SPAN
#define POWER_MODEL_MAP_SPAN 0x10000UL
#endif

#define FS_COUNTS 32768U
#define V_FS_MV 2500U
#define I_FS_MA 2500U

#define TD_REG_WORDS 42
#define TD_SNAPSHOT_MAX_RETRIES 1000

#define TD_OFF_SEQ_DATA 0x010U
#define TD_OFF_ACTIVE_BANK_DATA 0x020U
#define TD_OFF_BANK0_BASE 0x100U
#define TD_OFF_BANK1_BASE 0x200U

#define PLL_BANK_WORDS 10
#define PLL_OFF_SEQ 0x10U
#define PLL_OFF_ACTIVE_BANK 0x18U
#define PLL_OFF_BANK0 0x40U
#define PLL_OFF_BANK1 0x80U

#define PLL_WORD_WIN_ID 0
#define PLL_WORD_DPHI_FILT 1
#define PLL_WORD_INC_SMOOTH 2
#define PLL_WORD_PHASE_ACC 3
#define PLL_WORD_ENV_STATUS 4
#define PLL_WORD_DPHI_RAW_LAST 5
#define PLL_WORD_EF_LP 6
#define PLL_WORD_ENV_VI 7

#define IDX_N_SAMPLES 1
#define IDX_SUM_V2_BASE 2
#define IDX_SUM_I2_BASE 10
#define IDX_SUM_VI_BASE 14
#define IDX_VMAX_BASE 30
#define IDX_IMAX_BASE 34

#define FFT_POINTS 512U
#define FFT_POSITIVE_BINS (FFT_POINTS / 2U)
#define FFT_SEARCH_RADIUS 3U
#define FFT_BAND_RADIUS 2U
#define FFT_BAR_MAX_HEIGHT 136U

typedef enum {
    POWER_BACKEND_DEMO = 0,
    POWER_BACKEND_LIVE = 1
} power_backend_t;

typedef struct {
    off_t phys_addr;
    size_t span;
    off_t page_base;
    size_t page_offset;
    size_t map_len;
    volatile uint8_t *map;
} mapped_region_t;

typedef struct {
    uint32_t seq;
    uint32_t dphi;
    uint32_t inc;
    uint32_t phase;
    uint32_t misc;
    uint8_t signal_valid;
    uint8_t pll_locked;
    uint8_t dphi_valid;
    uint32_t win_id;
    uint32_t commit_seq;
    uint32_t active_bank;
    int32_t dphi_raw_last;
    int32_t ef_lp;
    uint16_t env_v;
    uint16_t env_i;
} pll_phase_diff_snapshot_t;

typedef struct {
    int mem_fd;
    int live_warned;
    power_backend_t backend;
    int have_last_td_seq;
    uint32_t last_td_seq;
    uint32_t repeated_td_seq_count;
    mapped_region_t td;
    mapped_region_t pll;
} power_hw_state_t;

static power_hw_state_t g_hw = {
    .mem_fd = -1,
    .backend = POWER_BACKEND_DEMO
};
static uint32_t g_tick;

static void power_log_once(const char *fmt, ...)
{
    va_list ap;

    if(g_hw.live_warned) {
        return;
    }

    g_hw.live_warned = 1;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static int64_t app_div_round_s64(int64_t x, int64_t d)
{
    if(d <= 0) {
        return 0;
    }
    if(x >= 0) {
        return (x + d / 2) / d;
    }
    return -(((-x) + d / 2) / d);
}

static uint64_t app_div_round_u64(uint64_t x, uint64_t d)
{
    if(d == 0U) {
        return 0U;
    }
    return (x + d / 2U) / d;
}

static uint64_t app_abs_s64_to_u64(int64_t x)
{
    return (x >= 0) ? (uint64_t)x : (uint64_t)(-x);
}

static uint64_t app_rd_u64_lohi(const uint32_t *w, int lo_idx)
{
    return ((uint64_t)w[lo_idx + 1] << 32) | (uint64_t)w[lo_idx];
}

static int64_t app_rd_s64_lohi(const uint32_t *w, int lo_idx)
{
    return ((int64_t)(int32_t)w[lo_idx + 1] << 32) | (uint32_t)w[lo_idx];
}

static uint32_t app_isqrt_u64(uint64_t x)
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

static void app_fmt_u_milli(char *buf, size_t len, uint32_t milli, const char *unit)
{
    snprintf(buf, len, "%lu.%03lu %s",
             (unsigned long)(milli / 1000U),
             (unsigned long)(milli % 1000U),
             unit);
}

static void app_fmt_s_milli(char *buf, size_t len, int64_t milli, const char *unit)
{
    unsigned long whole;
    unsigned long frac;

    if(milli < 0) {
        milli = -milli;
        whole = (unsigned long)((uint64_t)milli / 1000U);
        frac = (unsigned long)((uint64_t)milli % 1000U);
        snprintf(buf, len, "-%lu.%03lu %s", whole, frac, unit);
        return;
    }

    whole = (unsigned long)((uint64_t)milli / 1000U);
    frac = (unsigned long)((uint64_t)milli % 1000U);
    snprintf(buf, len, "%lu.%03lu %s", whole, frac, unit);
}

static void app_fmt_fixed_3(char *buf, size_t len, int64_t milli)
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

static void app_fmt_fixed_2(char *buf, size_t len, int64_t centi, const char *unit)
{
    unsigned long whole;
    unsigned long frac;

    if(centi < 0) {
        centi = -centi;
        whole = (unsigned long)((uint64_t)centi / 100U);
        frac = (unsigned long)((uint64_t)centi % 100U);
        snprintf(buf, len, "-%lu.%02lu %s", whole, frac, unit);
        return;
    }

    whole = (unsigned long)((uint64_t)centi / 100U);
    frac = (unsigned long)((uint64_t)centi % 100U);
    snprintf(buf, len, "%lu.%02lu %s", whole, frac, unit);
}

static void app_fmt_value_3(char *buf, size_t len, uint32_t milli)
{
    snprintf(buf, len, "%lu.%03lu",
             (unsigned long)(milli / 1000U),
             (unsigned long)(milli % 1000U));
}

static void app_fmt_signed_value_2(char *buf, size_t len, int32_t centi)
{
    unsigned long whole;
    unsigned long frac;

    if(centi < 0) {
        centi = -centi;
        whole = (unsigned long)((uint32_t)centi / 100U);
        frac = (unsigned long)((uint32_t)centi % 100U);
        snprintf(buf, len, "-%lu.%02lu", whole, frac);
        return;
    }

    whole = (unsigned long)((uint32_t)centi / 100U);
    frac = (unsigned long)((uint32_t)centi % 100U);
    snprintf(buf, len, "%lu.%02lu", whole, frac);
}

static uint32_t abs_counts_to_milli(uint32_t abs_counts, uint32_t fs_milli)
{
    return (uint32_t)(((uint64_t)abs_counts * (uint64_t)fs_milli + (FS_COUNTS / 2U)) /
                      (uint64_t)FS_COUNTS);
}

static int64_t p_counts_to_uW(int64_t p_counts)
{
    uint64_t den;
    int64_t num = p_counts;

    num = num * (int64_t)V_FS_MV;
    num = num * (int64_t)I_FS_MA;
    den = (uint64_t)FS_COUNTS * (uint64_t)FS_COUNTS;
    if(den == 0U) {
        return 0;
    }

    return (int64_t)(num / (int64_t)den);
}

static uint64_t s_counts_to_uVA(uint32_t vrms_counts, uint32_t irms_counts)
{
    uint64_t num = (uint64_t)vrms_counts * (uint64_t)irms_counts;
    uint64_t den;

    num = num * (uint64_t)V_FS_MV;
    num = num * (uint64_t)I_FS_MA;
    den = (uint64_t)FS_COUNTS * (uint64_t)FS_COUNTS;
    if(den == 0U) {
        return 0;
    }

    return num / den;
}

static int32_t pll_div_round_s64_local(int64_t x, int64_t d)
{
    if(d <= 0) {
        return 0;
    }
    if(x >= 0) {
        return (int32_t)((x + d / 2) / d);
    }
    return (int32_t)(-(((-x) + d / 2) / d));
}

static uint32_t pll_phase_diff_freq_centi_khz(uint32_t inc_word)
{
    uint64_t scaled = (uint64_t)inc_word * 100000ULL + 0x80000000ULL;
    return (uint32_t)(scaled >> 32);
}

static int32_t pll_phase_diff_phi_centi_deg(uint32_t dphi_word)
{
    int32_t signed_pu = (int32_t)dphi_word;
    int64_t num = (int64_t)signed_pu * 18000LL;
    return pll_div_round_s64_local(num, 2147483648LL);
}

static int map_region(int mem_fd, mapped_region_t *region, off_t phys_addr, size_t span)
{
    long page_size = sysconf(_SC_PAGESIZE);
    off_t page_base;
    size_t page_offset;
    size_t map_len;
    void *map;

    if(page_size <= 0) {
        errno = EINVAL;
        return -1;
    }

    page_base = phys_addr & ~((off_t)page_size - 1);
    page_offset = (size_t)(phys_addr - page_base);
    map_len = page_offset + span;

    map = mmap(NULL, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, page_base);
    if(map == MAP_FAILED) {
        return -1;
    }

    region->phys_addr = phys_addr;
    region->span = span;
    region->page_base = page_base;
    region->page_offset = page_offset;
    region->map_len = map_len;
    region->map = (volatile uint8_t *)map;
    return 0;
}

static void unmap_region(mapped_region_t *region)
{
    if(region->map != NULL) {
        munmap((void *)region->map, region->map_len);
    }
    memset(region, 0, sizeof(*region));
}

static uint32_t reg_read32(const mapped_region_t *region, uint32_t off)
{
    volatile const uint32_t *addr = (volatile const uint32_t *)(region->map + region->page_offset + off);
    return *addr;
}

static int td_stats_read_snapshot(uint32_t out[TD_REG_WORDS], uint32_t *seq_out, uint32_t *bank_out)
{
    int retry;

    for(retry = 0; retry < TD_SNAPSHOT_MAX_RETRIES; retry++) {
        uint32_t seq1 = reg_read32(&g_hw.td, TD_OFF_SEQ_DATA);
        uint32_t bank1 = reg_read32(&g_hw.td, TD_OFF_ACTIVE_BANK_DATA) & 1U;
        uint32_t base = bank1 ? TD_OFF_BANK1_BASE : TD_OFF_BANK0_BASE;
        int i;

        for(i = 0; i < TD_REG_WORDS; i++) {
            out[i] = reg_read32(&g_hw.td, base + 4U * (uint32_t)i);
        }

        if(seq1 == reg_read32(&g_hw.td, TD_OFF_SEQ_DATA) &&
           bank1 == (reg_read32(&g_hw.td, TD_OFF_ACTIVE_BANK_DATA) & 1U)) {
            if(seq_out != NULL) {
                *seq_out = seq1;
            }
            if(bank_out != NULL) {
                *bank_out = bank1;
            }
            return 0;
        }
    }

    return -1;
}

static int pll_phase_diff_read_snapshot(pll_phase_diff_snapshot_t *out)
{
    uint32_t words[PLL_BANK_WORDS];
    int retry;

    if(out == NULL) {
        return -1;
    }

    for(retry = 0; retry < 16; retry++) {
        uint32_t seq1 = reg_read32(&g_hw.pll, PLL_OFF_SEQ);
        uint32_t active1 = reg_read32(&g_hw.pll, PLL_OFF_ACTIVE_BANK) & 1U;
        uint32_t bank_base = active1 ? PLL_OFF_BANK1 : PLL_OFF_BANK0;
        uint32_t st;
        uint32_t env_status;
        uint32_t env_vi;
        uint32_t seq2;
        uint32_t active2;
        int i;

        for(i = 0; i < PLL_BANK_WORDS; i++) {
            words[i] = reg_read32(&g_hw.pll, bank_base + (uint32_t)i * 4U);
        }

        active2 = reg_read32(&g_hw.pll, PLL_OFF_ACTIVE_BANK) & 1U;
        seq2 = reg_read32(&g_hw.pll, PLL_OFF_SEQ);

        if(seq1 != seq2 || active1 != active2) {
            continue;
        }

        env_status = words[PLL_WORD_ENV_STATUS];
        env_vi = words[PLL_WORD_ENV_VI];
        st = (env_status >> 8) & 0xffU;

        out->win_id = words[PLL_WORD_WIN_ID];
        out->commit_seq = seq2;
        out->active_bank = active2;
        out->seq = out->win_id;
        out->dphi = words[PLL_WORD_DPHI_FILT];
        out->inc = words[PLL_WORD_INC_SMOOTH];
        out->phase = words[PLL_WORD_PHASE_ACC];
        out->misc = env_status;
        out->dphi_raw_last = (int32_t)words[PLL_WORD_DPHI_RAW_LAST];
        out->ef_lp = (int32_t)words[PLL_WORD_EF_LP];
        out->env_v = (uint16_t)(env_vi >> 16);
        out->env_i = (uint16_t)(env_vi & 0xffffU);
        out->signal_valid = (uint8_t)(st & 0x01U);
        out->pll_locked = (uint8_t)((st >> 1) & 0x01U);
        out->dphi_valid = (uint8_t)((st >> 3) & 0x01U);
        return 0;
    }

    return -1;
}

static void build_demo_metrics(live_metrics_t *metrics)
{
    static const int32_t phase_pattern[] = {1520, 1540, 1570, 1510, 1480, 1500, 1530, 1560};
    static const int64_t power_pattern[] = {12480, 12610, 12540, 12390, 12280, 12410, 12590, 12640};
    static const uint32_t apparent_pattern[] = {12930, 13050, 12980, 12870, 12790, 12880, 13010, 13080};
    static const uint32_t vrms_pattern[] = {220500, 220860, 220720, 220180, 219940, 220240, 220690, 220910};
    static const uint32_t irms_pattern[] = {58, 59, 58, 57, 57, 58, 59, 59};
    static const uint32_t vpk_pattern[] = {311800, 312210, 312040, 311460, 311130, 311520, 312000, 312260};
    static const uint32_t ipk_pattern[] = {82, 83, 82, 81, 80, 81, 83, 83};
    static const uint32_t freq_pattern[] = {1000, 1001, 1002, 1001, 999, 998, 999, 1000};
    size_t idx = (size_t)(g_tick % (sizeof(phase_pattern) / sizeof(phase_pattern[0])));
    int64_t q2;

    memset(metrics, 0, sizeof(*metrics));

    metrics->v_rms_mV = vrms_pattern[idx];
    metrics->i_rms_mA = irms_pattern[idx];
    metrics->v_peak_mV = vpk_pattern[idx];
    metrics->i_peak_mA = ipk_pattern[idx];
    metrics->p_mW = power_pattern[idx];
    metrics->s_mVA = apparent_pattern[idx];
    metrics->pf_milli = app_div_round_s64(metrics->p_mW * 1000LL, (int64_t)metrics->s_mVA);
    metrics->freq_centi_khz = freq_pattern[idx];
    metrics->phi_centi_deg = phase_pattern[idx];
    metrics->freq_ok = 1;
    metrics->phi_ok = 1;

    q2 = (int64_t)metrics->s_mVA * (int64_t)metrics->s_mVA - metrics->p_mW * metrics->p_mW;
    if(q2 < 0) {
        q2 = 0;
    }
    metrics->q_mVAR = (int64_t)app_isqrt_u64((uint64_t)q2);

    if(metrics->phi_centi_deg < 0) {
        metrics->q_mVAR = -metrics->q_mVAR;
    }
}

static void format_view(const live_metrics_t *metrics, monitor_view_t *view)
{
    app_fmt_u_milli(view->v0_peak, sizeof(view->v0_peak), metrics->v_peak_mV, "V");
    app_fmt_u_milli(view->v0_rms, sizeof(view->v0_rms), metrics->v_rms_mV, "V");
    app_fmt_u_milli(view->i4_peak, sizeof(view->i4_peak), metrics->i_peak_mA, "A");
    app_fmt_u_milli(view->i4_rms, sizeof(view->i4_rms), metrics->i_rms_mA, "A");
    app_fmt_value_3(view->v0_rms_value, sizeof(view->v0_rms_value), metrics->v_rms_mV);
    app_fmt_value_3(view->i4_rms_value, sizeof(view->i4_rms_value), metrics->i_rms_mA);
    app_fmt_s_milli(view->p, sizeof(view->p), metrics->p_mW, "W");
    app_fmt_u_milli(view->s, sizeof(view->s), metrics->s_mVA, "VA");
    app_fmt_fixed_3(view->pf, sizeof(view->pf), metrics->pf_milli);
    app_fmt_s_milli(view->q, sizeof(view->q), metrics->q_mVAR, "VAR");

    if(metrics->freq_ok) {
        snprintf(view->freq, sizeof(view->freq), "%lu.%02lu KHZ",
                 (unsigned long)(metrics->freq_centi_khz / 100U),
                 (unsigned long)(metrics->freq_centi_khz % 100U));
        app_fmt_signed_value_2(view->freq_value, sizeof(view->freq_value), (int32_t)metrics->freq_centi_khz);
    }
    else {
        strcpy(view->freq, "--.-- KHZ");
        strcpy(view->freq_value, "--.--");
    }

    if(metrics->phi_ok) {
        app_fmt_fixed_2(view->phi, sizeof(view->phi), metrics->phi_centi_deg, "DEG");
        app_fmt_signed_value_2(view->phi_value, sizeof(view->phi_value), metrics->phi_centi_deg);
    }
    else {
        strcpy(view->phi, "---.-- DEG");
        strcpy(view->phi_value, "---.--");
    }

    if(!metrics->phi_ok) {
        strcpy(view->mode, "UNKNOWN");
    }
    else if(metrics->phi_centi_deg > 50) {
        strcpy(view->mode, "INDUCTIVE");
    }
    else if(metrics->phi_centi_deg < -50) {
        strcpy(view->mode, "CAPACITIVE");
    }
    else {
        strcpy(view->mode, "RESISTIVE");
    }
}

static int update_monitor_view_live(monitor_view_t *view, live_metrics_t *metrics)
{
    uint32_t w[TD_REG_WORDS];
    uint32_t td_seq = 0U;
    uint32_t td_bank = 0U;
    uint32_t n_samples;
    uint64_t sum_v2_0;
    uint64_t sum_i2_0;
    int64_t sum_vi_00;
    uint64_t v0_ms;
    uint64_t i4_ms;
    uint32_t v0_rms;
    uint32_t i4_rms;
    uint32_t v0_peak;
    uint32_t i4_peak;
    int64_t p_counts;
    uint64_t s_counts;
    uint32_t v0_peak_mV;
    uint32_t v0_rms_mV;
    uint32_t i4_peak_mA;
    uint32_t i4_rms_mA;
    int64_t p_uW;
    int64_t p_mW;
    uint64_t s_uVA;
    uint32_t s_mVA;
    int64_t pf_milli = 0;
    pll_phase_diff_snapshot_t pll;
    int pll_ok;
    int freq_ok;
    int phi_ok;
    uint32_t freq_centi_khz = 0;
    int32_t phi_centi_deg = 0;
    uint32_t kv_ppm = AMP_COMP_UNITY_PPM;
    uint32_t ki_ppm = AMP_COMP_UNITY_PPM;
    uint32_t kvi_ppm = AMP_COMP_UNITY_PPM;
    uint64_t abs_p_milli;
    uint64_t q2;
    int64_t q_mVAR;

    if(td_stats_read_snapshot(w, &td_seq, &td_bank) != 0) {
        power_log_once("power_model: td_stats snapshot timeout\n");
        return 0;
    }

    n_samples = w[IDX_N_SAMPLES];
    if(n_samples == 0U) {
        power_log_once("power_model: td_stats seq=%lu bank=%lu but n_samples is zero, drop invalid frame\n",
                       (unsigned long)td_seq, (unsigned long)td_bank);
        return 0;
    }

    if(g_hw.have_last_td_seq && td_seq == g_hw.last_td_seq) {
        g_hw.repeated_td_seq_count++;
        if(g_hw.repeated_td_seq_count == 5U || (g_hw.repeated_td_seq_count % 16U) == 0U) {
            fprintf(stderr,
                    "power_model: td_stats seq still %lu bank=%lu n_samples=%lu, refreshing same snapshot\n",
                    (unsigned long)td_seq,
                    (unsigned long)td_bank,
                    (unsigned long)n_samples);
        }
    } else {
        g_hw.repeated_td_seq_count = 0U;
    }

    sum_v2_0 = app_rd_u64_lohi(w, IDX_SUM_V2_BASE + 0 * 2);
    sum_i2_0 = app_rd_u64_lohi(w, IDX_SUM_I2_BASE + 0 * 2);
    sum_vi_00 = app_rd_s64_lohi(w, IDX_SUM_VI_BASE + (0 * 2 + 0) * 2);

    v0_ms = sum_v2_0 / (uint64_t)n_samples;
    i4_ms = sum_i2_0 / (uint64_t)n_samples;
    v0_rms = app_isqrt_u64(v0_ms);
    i4_rms = app_isqrt_u64(i4_ms);

    v0_peak = w[IDX_VMAX_BASE + 0];
    i4_peak = w[IDX_IMAX_BASE + 0];
    p_counts = sum_vi_00 / (int64_t)n_samples;
    s_counts = (uint64_t)v0_rms * (uint64_t)i4_rms;

    v0_peak_mV = abs_counts_to_milli(v0_peak, V_FS_MV);
    v0_rms_mV = abs_counts_to_milli(v0_rms, V_FS_MV);
    i4_peak_mA = abs_counts_to_milli(i4_peak, I_FS_MA);
    i4_rms_mA = abs_counts_to_milli(i4_rms, I_FS_MA);

    p_uW = p_counts_to_uW(p_counts);
    p_mW = app_div_round_s64(p_uW, 1000);

    s_uVA = s_counts_to_uVA(v0_rms, i4_rms);
    s_mVA = (uint32_t)((s_uVA + 500ULL) / 1000ULL);

    if(s_counts != 0U) {
        pf_milli = app_div_round_s64(p_counts * 1000LL, (int64_t)s_counts);
        if(pf_milli > 1000) {
            pf_milli = 1000;
        }
        if(pf_milli < -1000) {
            pf_milli = -1000;
        }
    }

    pll_ok = (pll_phase_diff_read_snapshot(&pll) == 0);
    freq_ok = pll_ok && pll.signal_valid && pll.pll_locked;
    phi_ok = pll_ok && pll.signal_valid && pll.dphi_valid;

    if(pll_ok) {
        freq_centi_khz = pll_phase_diff_freq_centi_khz(pll.inc);
        phi_centi_deg = pll_phase_diff_phi_centi_deg((uint32_t)(-(int32_t)pll.dphi));
    }

    if(freq_ok) {
        kv_ppm = amp_comp_lookup_voltage_ppm(freq_centi_khz);
        ki_ppm = amp_comp_lookup_current_ppm(freq_centi_khz);
        kvi_ppm = amp_comp_mul_ppm(kv_ppm, ki_ppm);

        v0_peak_mV = amp_comp_apply_u32(v0_peak_mV, kv_ppm);
        v0_rms_mV = amp_comp_apply_u32(v0_rms_mV, kv_ppm);
        i4_peak_mA = amp_comp_apply_u32(i4_peak_mA, ki_ppm);
        i4_rms_mA = amp_comp_apply_u32(i4_rms_mA, ki_ppm);
        p_mW = amp_comp_apply_s64(p_mW, kvi_ppm);
        s_mVA = amp_comp_apply_u32(s_mVA, kvi_ppm);
    }

    abs_p_milli = app_abs_s64_to_u64(p_mW);
    if(abs_p_milli > (uint64_t)s_mVA) {
        abs_p_milli = (uint64_t)s_mVA;
    }

    q2 = (uint64_t)s_mVA * (uint64_t)s_mVA - abs_p_milli * abs_p_milli;
    q_mVAR = (int64_t)app_isqrt_u64(q2);
    if(phi_centi_deg < 0) {
        q_mVAR = -q_mVAR;
    }

    metrics->v_rms_mV = v0_rms_mV;
    metrics->i_rms_mA = i4_rms_mA;
    metrics->v_peak_mV = v0_peak_mV;
    metrics->i_peak_mA = i4_peak_mA;
    metrics->p_mW = p_mW;
    metrics->s_mVA = s_mVA;
    metrics->q_mVAR = q_mVAR;
    metrics->pf_milli = pf_milli;
    metrics->freq_centi_khz = freq_centi_khz;
    metrics->phi_centi_deg = phi_centi_deg;
    metrics->freq_ok = freq_ok;
    metrics->phi_ok = phi_ok;

    g_hw.last_td_seq = td_seq;
    g_hw.have_last_td_seq = 1;

    format_view(metrics, view);
    return 1;
}

void power_model_init(void)
{
    g_tick = 0;
    g_hw.live_warned = 0;
    g_hw.backend = POWER_BACKEND_DEMO;
    g_hw.have_last_td_seq = 0;
    g_hw.last_td_seq = 0;
    g_hw.mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(g_hw.mem_fd < 0) {
        fprintf(stderr, "power_model: cannot open /dev/mem (%s), fallback to demo data\n", strerror(errno));
        return;
    }

    if(map_region(g_hw.mem_fd, &g_hw.td, (off_t)TD_STATS_BASE_ADDR, POWER_MODEL_MAP_SPAN) != 0) {
        fprintf(stderr, "power_model: map td_stats@0x%08lx failed (%s), fallback to demo data\n",
                (unsigned long)TD_STATS_BASE_ADDR, strerror(errno));
        close(g_hw.mem_fd);
        g_hw.mem_fd = -1;
        return;
    }

    if(map_region(g_hw.mem_fd, &g_hw.pll, (off_t)PLL_PHASE_BASE_ADDR, POWER_MODEL_MAP_SPAN) != 0) {
        fprintf(stderr, "power_model: map pll_phase_diff@0x%08lx failed (%s), fallback to demo data\n",
                (unsigned long)PLL_PHASE_BASE_ADDR, strerror(errno));
        unmap_region(&g_hw.td);
        close(g_hw.mem_fd);
        g_hw.mem_fd = -1;
        return;
    }

    g_hw.backend = POWER_BACKEND_LIVE;
    fprintf(stderr, "power_model: live input enabled td=0x%08lx pll=0x%08lx\n",
            (unsigned long)TD_STATS_BASE_ADDR, (unsigned long)PLL_PHASE_BASE_ADDR);
}

void monitor_view_defaults(monitor_view_t *view)
{
    strcpy(view->v0_peak, "--.--- V");
    strcpy(view->v0_rms, "--.--- V");
    strcpy(view->i4_peak, "--.--- A");
    strcpy(view->i4_rms, "--.--- A");
    strcpy(view->p, "--.--- W");
    strcpy(view->s, "--.--- VA");
    strcpy(view->pf, "-.---");
    strcpy(view->q, "--.--- VAR");
    strcpy(view->freq, "--.-- KHZ");
    strcpy(view->phi, "---.-- DEG");
    strcpy(view->v0_rms_value, "--.---");
    strcpy(view->i4_rms_value, "--.---");
    strcpy(view->freq_value, "--.--");
    strcpy(view->phi_value, "---.--");
    strcpy(view->mode, "UNKNOWN");
}

void fft_view_defaults(fft_view_t *view)
{
    static const uint16_t default_voltage_bars[7] = { 136U, 24U, 68U, 20U, 52U, 18U, 40U };
    static const uint16_t default_current_bars[7] = { 116U, 18U, 52U, 16U, 32U, 14U, 24U };
    uint32_t i;

    strcpy(view->voltage_thd, "--.--- %");
    strcpy(view->voltage_h1, "--.--- V");
    strcpy(view->voltage_h2, "--.--- V");
    strcpy(view->voltage_h3, "--.--- V");
    strcpy(view->voltage_h4, "--.--- V");
    strcpy(view->voltage_h5, "--.--- V");
    strcpy(view->voltage_h6, "--.--- V");
    strcpy(view->voltage_h7, "--.--- V");
    strcpy(view->voltage_peak, "--.--- V");
    strcpy(view->current_thd, "--.--- %");
    strcpy(view->current_h1, "--.--- A");
    strcpy(view->current_h2, "--.--- A");
    strcpy(view->current_h3, "--.--- A");
    strcpy(view->current_h4, "--.--- A");
    strcpy(view->current_h5, "--.--- A");
    strcpy(view->current_h6, "--.--- A");
    strcpy(view->current_h7, "--.--- A");
    strcpy(view->current_peak, "--.--- A");

    for(i = 0; i < 7U; i++) {
        view->voltage_bar_height[i] = default_voltage_bars[i];
        view->current_bar_height[i] = default_current_bars[i];
    }
}

void power_model_build_fft_view(fft_view_t *view, const live_metrics_t *metrics)
{
    char *voltage_harmonics[6];
    char *current_harmonics[6];
    uint32_t i;

    if(view == NULL) {
        return;
    }

    fft_view_defaults(view);
    if(metrics == NULL) {
        return;
    }

    app_fmt_u_milli(view->voltage_h1, sizeof(view->voltage_h1), metrics->v_rms_mV, "V");
    app_fmt_u_milli(view->current_h1, sizeof(view->current_h1), metrics->i_rms_mA, "A");
    app_fmt_u_milli(view->voltage_peak, sizeof(view->voltage_peak), metrics->v_peak_mV, "V");
    app_fmt_u_milli(view->current_peak, sizeof(view->current_peak), metrics->i_peak_mA, "A");

    voltage_harmonics[0] = view->voltage_h2;
    voltage_harmonics[1] = view->voltage_h3;
    voltage_harmonics[2] = view->voltage_h4;
    voltage_harmonics[3] = view->voltage_h5;
    voltage_harmonics[4] = view->voltage_h6;
    voltage_harmonics[5] = view->voltage_h7;
    current_harmonics[0] = view->current_h2;
    current_harmonics[1] = view->current_h3;
    current_harmonics[2] = view->current_h4;
    current_harmonics[3] = view->current_h5;
    current_harmonics[4] = view->current_h6;
    current_harmonics[5] = view->current_h7;

    for(i = 0; i < 6U; i++) {
        strcpy(voltage_harmonics[i], "--.--- V");
        strcpy(current_harmonics[i], "--.--- A");
    }

    view->voltage_bar_height[0] = 136U;
    view->current_bar_height[0] = 136U;
    for(i = 1; i < 7U; i++) {
        view->voltage_bar_height[i] = 0U;
        view->current_bar_height[i] = 0U;
    }
}

static uint64_t fft_mag_sq_word(uint64_t word, uint32_t channel)
{
    int32_t re;
    int32_t im;

    if(channel == 0U) {
        re = (int16_t)(word & 0xffffU);
        im = (int16_t)((word >> 16) & 0xffffU);
    }
    else {
        re = (int16_t)((word >> 32) & 0xffffU);
        im = (int16_t)((word >> 48) & 0xffffU);
    }

    return (uint64_t)((int64_t)re * (int64_t)re + (int64_t)im * (int64_t)im);
}

static uint64_t fft_mag_sq_bin(const uint64_t *frame, uint32_t frame_word_count,
                               uint32_t channel, uint32_t bin)
{
    if(frame == NULL || bin >= FFT_POINTS || bin >= frame_word_count) {
        return 0U;
    }
    return fft_mag_sq_word(frame[bin], channel);
}

static uint32_t fft_estimate_fundamental_bin(const uint64_t *frame, uint32_t frame_word_count,
                                             const live_metrics_t *metrics)
{
    uint32_t start_bin = 1U;
    uint32_t end_bin = FFT_POSITIVE_BINS - 1U;
    uint32_t best_bin = 1U;
    uint64_t best_mag = 0U;
    uint32_t bin;

    if(metrics != NULL && metrics->freq_ok) {
        uint32_t guess_bin = (uint32_t)app_div_round_u64(
            (uint64_t)metrics->freq_centi_khz * (uint64_t)FFT_POINTS, 100000U);

        if(guess_bin < 1U) {
            guess_bin = 1U;
        }
        if(guess_bin > (FFT_POSITIVE_BINS - 1U)) {
            guess_bin = FFT_POSITIVE_BINS - 1U;
        }

        start_bin = (guess_bin > FFT_SEARCH_RADIUS) ? (guess_bin - FFT_SEARCH_RADIUS) : 1U;
        end_bin = guess_bin + FFT_SEARCH_RADIUS;
        if(end_bin > (FFT_POSITIVE_BINS - 1U)) {
            end_bin = FFT_POSITIVE_BINS - 1U;
        }
    }

    for(bin = start_bin; bin <= end_bin; bin++) {
        uint64_t mag = fft_mag_sq_bin(frame, frame_word_count, 0U, bin);
        if(mag > best_mag) {
            best_mag = mag;
            best_bin = bin;
        }
    }

    return best_bin;
}

static uint64_t fft_band_energy(const uint64_t *frame, uint32_t frame_word_count,
                                uint32_t channel, uint32_t center_bin)
{
    uint32_t start_bin;
    uint32_t end_bin;
    uint32_t bin;
    uint64_t energy = 0U;

    if(center_bin == 0U || center_bin >= FFT_POSITIVE_BINS) {
        return 0U;
    }

    start_bin = (center_bin > FFT_BAND_RADIUS) ? (center_bin - FFT_BAND_RADIUS) : 1U;
    end_bin = center_bin + FFT_BAND_RADIUS;
    if(end_bin > (FFT_POSITIVE_BINS - 1U)) {
        end_bin = FFT_POSITIVE_BINS - 1U;
    }

    for(bin = start_bin; bin <= end_bin; bin++) {
        energy += fft_mag_sq_bin(frame, frame_word_count, channel, bin);
    }

    return energy;
}

static uint32_t fft_harmonic_gain_ppm(const live_metrics_t *metrics, uint32_t channel, uint32_t order)
{
    uint64_t harmonic_freq_centi_khz;

    if(metrics == NULL || !metrics->freq_ok || order == 0U) {
        return AMP_COMP_UNITY_PPM;
    }

    harmonic_freq_centi_khz = (uint64_t)metrics->freq_centi_khz * (uint64_t)order;
    if(harmonic_freq_centi_khz > 0xFFFFFFFFULL) {
        harmonic_freq_centi_khz = 0xFFFFFFFFULL;
    }

    if(channel == 0U) {
        return amp_comp_lookup_voltage_ppm((uint32_t)harmonic_freq_centi_khz);
    }

    return amp_comp_lookup_current_ppm((uint32_t)harmonic_freq_centi_khz);
}

static uint32_t fft_compensate_magnitude(uint64_t raw_energy, uint32_t gain_ppm)
{
    uint64_t raw_mag = app_isqrt_u64(raw_energy);
    return (uint32_t)app_div_round_u64(raw_mag * (uint64_t)gain_ppm, 1000000U);
}

static uint64_t fft_total_harmonic_energy_compensated(const uint64_t *frame, uint32_t frame_word_count,
                                                      uint32_t channel, uint32_t fundamental_bin,
                                                      const live_metrics_t *metrics)
{
    uint32_t order;
    uint64_t energy = 0U;

    if(fundamental_bin == 0U) {
        return 0U;
    }

    for(order = 2U; (order * fundamental_bin) < FFT_POSITIVE_BINS; order++) {
        uint64_t raw_band_energy = fft_band_energy(frame, frame_word_count, channel, order * fundamental_bin);
        uint64_t corrected_mag = fft_compensate_magnitude(raw_band_energy, fft_harmonic_gain_ppm(metrics, channel, order));
        energy += corrected_mag * corrected_mag;
    }

    return energy;
}

static uint16_t fft_make_bar_height(uint64_t ref_mag, uint64_t mag)
{
    uint64_t scaled;

    if(ref_mag == 0U || mag == 0U) {
        return 0U;
    }

    scaled = app_div_round_u64(mag * FFT_BAR_MAX_HEIGHT, ref_mag);
    if(scaled > FFT_BAR_MAX_HEIGHT) {
        scaled = FFT_BAR_MAX_HEIGHT;
    }
    return (uint16_t)scaled;
}

static uint32_t fft_estimate_h1_rms(uint32_t total_rms_milli, uint32_t thd_ratio_milli)
{
    uint64_t denom = app_isqrt_u64(1000000ULL + (uint64_t)thd_ratio_milli * (uint64_t)thd_ratio_milli);
    if(denom == 0U) {
        return total_rms_milli;
    }
    return (uint32_t)app_div_round_u64((uint64_t)total_rms_milli * 1000ULL, denom);
}

static void fill_fft_channel_view(char *thd_buf, size_t thd_len,
                                  char *h1_buf, size_t h1_len,
                                  char *h2_buf, size_t h2_len,
                                  char *h3_buf, size_t h3_len,
                                  char *h4_buf, size_t h4_len,
                                  char *h5_buf, size_t h5_len,
                                  char *h6_buf, size_t h6_len,
                                  char *h7_buf, size_t h7_len,
                                  char *peak_buf, size_t peak_len,
                                  uint16_t bar_height[7],
                                  const uint64_t *frame, uint32_t frame_word_count,
                                  uint32_t channel, uint32_t fundamental_bin,
                                  uint32_t total_rms_milli, uint32_t peak_milli,
                                  const char *unit, const live_metrics_t *metrics)
{
    static const uint32_t harmonic_order[7] = {1U, 2U, 3U, 4U, 5U, 6U, 7U};
    uint64_t harmonic_energy[7];
    uint32_t harmonic_mag[7];
    uint64_t total_harm_energy;
    uint32_t thd_ratio_milli = 0U;
    uint32_t thd_percent_milli = 0U;
    uint32_t h1_rms = 0U;
    uint32_t harmonic_milli[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint32_t i;

    for(i = 0U; i < 7U; i++) {
        uint32_t harmonic_bin = fundamental_bin * harmonic_order[i];
        harmonic_energy[i] = fft_band_energy(frame, frame_word_count, channel, harmonic_bin);
        harmonic_mag[i] = fft_compensate_magnitude(harmonic_energy[i],
                                                   fft_harmonic_gain_ppm(metrics, channel, harmonic_order[i]));
    }

    total_harm_energy = fft_total_harmonic_energy_compensated(frame, frame_word_count, channel,
                                                              fundamental_bin, metrics);
    if(harmonic_mag[0] != 0U) {
        thd_ratio_milli = (uint32_t)app_div_round_u64(
            (uint64_t)app_isqrt_u64(total_harm_energy) * 1000ULL, harmonic_mag[0]);
        thd_percent_milli = (uint32_t)app_div_round_u64(
            (uint64_t)app_isqrt_u64(total_harm_energy) * 100000ULL, harmonic_mag[0]);
        h1_rms = fft_estimate_h1_rms(total_rms_milli, thd_ratio_milli);

        for(i = 0U; i < 7U; i++) {
            harmonic_milli[i] = (uint32_t)app_div_round_u64(
                (uint64_t)h1_rms * harmonic_mag[i], harmonic_mag[0]);
            bar_height[i] = fft_make_bar_height(harmonic_mag[0], harmonic_mag[i]);
        }
        bar_height[0] = FFT_BAR_MAX_HEIGHT;
    }
    else {
        for(i = 0U; i < 7U; i++) {
            bar_height[i] = 0U;
        }
    }

    app_fmt_u_milli(thd_buf, thd_len, thd_percent_milli, "%");
    app_fmt_u_milli(h1_buf, h1_len, harmonic_milli[0], unit);
    app_fmt_u_milli(h2_buf, h2_len, harmonic_milli[1], unit);
    app_fmt_u_milli(h3_buf, h3_len, harmonic_milli[2], unit);
    app_fmt_u_milli(h4_buf, h4_len, harmonic_milli[3], unit);
    app_fmt_u_milli(h5_buf, h5_len, harmonic_milli[4], unit);
    app_fmt_u_milli(h6_buf, h6_len, harmonic_milli[5], unit);
    app_fmt_u_milli(h7_buf, h7_len, harmonic_milli[6], unit);
    app_fmt_u_milli(peak_buf, peak_len, peak_milli, unit);
}

void power_model_build_fft_view_from_frame(fft_view_t *view,
                                           const uint64_t *frame_words,
                                           uint32_t frame_word_count,
                                           const live_metrics_t *metrics)
{
    uint32_t fundamental_bin;

    if(view == NULL) {
        return;
    }

    fft_view_defaults(view);
    if(metrics != NULL) {
        app_fmt_u_milli(view->voltage_peak, sizeof(view->voltage_peak), metrics->v_peak_mV, "V");
        app_fmt_u_milli(view->current_peak, sizeof(view->current_peak), metrics->i_peak_mA, "A");
    }

    if(frame_words == NULL || frame_word_count < FFT_POINTS || metrics == NULL) {
        return;
    }

    fundamental_bin = fft_estimate_fundamental_bin(frame_words, frame_word_count, metrics);
    fill_fft_channel_view(view->voltage_thd, sizeof(view->voltage_thd),
                          view->voltage_h1, sizeof(view->voltage_h1),
                          view->voltage_h2, sizeof(view->voltage_h2),
                          view->voltage_h3, sizeof(view->voltage_h3),
                          view->voltage_h4, sizeof(view->voltage_h4),
                          view->voltage_h5, sizeof(view->voltage_h5),
                          view->voltage_h6, sizeof(view->voltage_h6),
                          view->voltage_h7, sizeof(view->voltage_h7),
                          view->voltage_peak, sizeof(view->voltage_peak),
                          view->voltage_bar_height,
                          frame_words, frame_word_count, 0U, fundamental_bin,
                          metrics->v_rms_mV, metrics->v_peak_mV, "V", metrics);

    fill_fft_channel_view(view->current_thd, sizeof(view->current_thd),
                          view->current_h1, sizeof(view->current_h1),
                          view->current_h2, sizeof(view->current_h2),
                          view->current_h3, sizeof(view->current_h3),
                          view->current_h4, sizeof(view->current_h4),
                          view->current_h5, sizeof(view->current_h5),
                          view->current_h6, sizeof(view->current_h6),
                          view->current_h7, sizeof(view->current_h7),
                          view->current_peak, sizeof(view->current_peak),
                          view->current_bar_height,
                          frame_words, frame_word_count, 1U, fundamental_bin,
                          metrics->i_rms_mA, metrics->i_peak_mA, "A", metrics);
}

void hold_state_defaults(hold_state_t *state)
{
    memset(state, 0, sizeof(*state));
}

void hold_view_defaults(hold_view_t *view)
{
    strcpy(view->vrms_max, "--.--- V");
    strcpy(view->vrms_min, "--.--- V");
    strcpy(view->irms_max, "--.--- A");
    strcpy(view->irms_min, "--.--- A");
    strcpy(view->p_max, "--.--- W");
    strcpy(view->freq_max, "--.-- KHZ");
    strcpy(view->freq_min, "--.-- KHZ");
    strcpy(view->event_count, "0");
    view->frozen = 0;
}

static void hold_seed_from_metrics(hold_state_t *state, const live_metrics_t *metrics)
{
    state->initialized = 1;
    state->vrms_max_mV = metrics->v_rms_mV;
    state->vrms_min_mV = metrics->v_rms_mV;
    state->irms_max_mA = metrics->i_rms_mA;
    state->irms_min_mA = metrics->i_rms_mA;
    state->p_max_mW = metrics->p_mW;
    state->freq_valid = metrics->freq_ok ? 1 : 0;
    if(state->freq_valid) {
        state->freq_max_centi_khz = metrics->freq_centi_khz;
        state->freq_min_centi_khz = metrics->freq_centi_khz;
    }
}

void hold_apply_actions(hold_state_t *state, const live_metrics_t *metrics, uint32_t actions)
{
    if(state == NULL || metrics == NULL || !state->initialized) {
        return;
    }

    if((actions & POWER_UI_HOLD_ACTION_CLEAR_MAX) != 0U) {
        state->vrms_max_mV = metrics->v_rms_mV;
        state->irms_max_mA = metrics->i_rms_mA;
        state->p_max_mW = metrics->p_mW;
        if(metrics->freq_ok) {
            state->freq_valid = 1;
            state->freq_max_centi_khz = metrics->freq_centi_khz;
        }
    }

    if((actions & POWER_UI_HOLD_ACTION_CLEAR_MIN) != 0U) {
        state->vrms_min_mV = metrics->v_rms_mV;
        state->irms_min_mA = metrics->i_rms_mA;
        if(metrics->freq_ok) {
            state->freq_valid = 1;
            state->freq_min_centi_khz = metrics->freq_centi_khz;
        }
    }
}

void hold_update(hold_state_t *state, const live_metrics_t *metrics)
{
    if(state == NULL || metrics == NULL) {
        return;
    }

    if(!state->initialized) {
        hold_seed_from_metrics(state, metrics);
        return;
    }

    if(state->frozen) {
        return;
    }

    if(metrics->v_rms_mV > state->vrms_max_mV) {
        state->vrms_max_mV = metrics->v_rms_mV;
        state->event_count++;
    }
    if(metrics->v_rms_mV < state->vrms_min_mV) {
        state->vrms_min_mV = metrics->v_rms_mV;
        state->event_count++;
    }
    if(metrics->i_rms_mA > state->irms_max_mA) {
        state->irms_max_mA = metrics->i_rms_mA;
        state->event_count++;
    }
    if(metrics->i_rms_mA < state->irms_min_mA) {
        state->irms_min_mA = metrics->i_rms_mA;
        state->event_count++;
    }
    if(metrics->p_mW > state->p_max_mW) {
        state->p_max_mW = metrics->p_mW;
        state->event_count++;
    }

    if(metrics->freq_ok) {
        if(!state->freq_valid) {
            state->freq_valid = 1;
            state->freq_max_centi_khz = metrics->freq_centi_khz;
            state->freq_min_centi_khz = metrics->freq_centi_khz;
        }
        else {
            if(metrics->freq_centi_khz > state->freq_max_centi_khz) {
                state->freq_max_centi_khz = metrics->freq_centi_khz;
                state->event_count++;
            }
            if(metrics->freq_centi_khz < state->freq_min_centi_khz) {
                state->freq_min_centi_khz = metrics->freq_centi_khz;
                state->event_count++;
            }
        }
    }
}

void hold_build_view(const hold_state_t *state, hold_view_t *view)
{
    if(view == NULL) {
        return;
    }

    hold_view_defaults(view);
    if(state == NULL) {
        return;
    }

    view->frozen = state->frozen;
    if(!state->initialized) {
        return;
    }

    app_fmt_u_milli(view->vrms_max, sizeof(view->vrms_max), state->vrms_max_mV, "V");
    app_fmt_u_milli(view->vrms_min, sizeof(view->vrms_min), state->vrms_min_mV, "V");
    app_fmt_u_milli(view->irms_max, sizeof(view->irms_max), state->irms_max_mA, "A");
    app_fmt_u_milli(view->irms_min, sizeof(view->irms_min), state->irms_min_mA, "A");
    app_fmt_s_milli(view->p_max, sizeof(view->p_max), state->p_max_mW, "W");
    if(state->freq_valid) {
        app_fmt_fixed_2(view->freq_max, sizeof(view->freq_max), state->freq_max_centi_khz, "KHZ");
        app_fmt_fixed_2(view->freq_min, sizeof(view->freq_min), state->freq_min_centi_khz, "KHZ");
    }
    snprintf(view->event_count, sizeof(view->event_count), "%lu", (unsigned long)state->event_count);
}

int power_model_step(monitor_view_t *view, live_metrics_t *metrics)
{
    if(view == NULL || metrics == NULL) {
        return 0;
    }

    if(g_hw.backend == POWER_BACKEND_LIVE) {
        if(update_monitor_view_live(view, metrics) != 0) {
            return 1;
        }
        return 0;
    }

    build_demo_metrics(metrics);
    format_view(metrics, view);
    g_tick++;
    return 1;
}

