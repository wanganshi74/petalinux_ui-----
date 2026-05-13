#ifndef LINUX_POWER_UI_POWER_MODEL_H
#define LINUX_POWER_UI_POWER_MODEL_H

#include <stdint.h>

typedef struct {
    char v0_peak[24];
    char v0_rms[24];
    char i4_peak[24];
    char i4_rms[24];
    char p[24];
    char s[24];
    char pf[24];
    char q[24];
    char freq[24];
    char phi[24];
    char v0_rms_value[16];
    char i4_rms_value[16];
    char freq_value[16];
    char phi_value[16];
    char mode[16];
} monitor_view_t;

typedef struct {
    uint32_t v_rms_mV;
    uint32_t i_rms_mA;
    uint32_t v_peak_mV;
    uint32_t i_peak_mA;
    int64_t p_mW;
    uint32_t s_mVA;
    int64_t q_mVAR;
    int64_t pf_milli;
    uint32_t freq_centi_khz;
    int32_t phi_centi_deg;
    int freq_ok;
    int phi_ok;
} live_metrics_t;

typedef struct {
    char voltage_thd[24];
    char voltage_h1[24];
    char voltage_h2[24];
    char voltage_h3[24];
    char voltage_h4[24];
    char voltage_h5[24];
    char voltage_h6[24];
    char voltage_h7[24];
    char voltage_peak[24];
    char current_thd[24];
    char current_h1[24];
    char current_h2[24];
    char current_h3[24];
    char current_h4[24];
    char current_h5[24];
    char current_h6[24];
    char current_h7[24];
    char current_peak[24];
    uint16_t voltage_bar_height[7];
    uint16_t current_bar_height[7];
} fft_view_t;

typedef struct {
    int initialized;
    int frozen;
    int freq_valid;
    uint32_t vrms_max_mV;
    uint32_t vrms_min_mV;
    uint32_t irms_max_mA;
    uint32_t irms_min_mA;
    int64_t p_max_mW;
    uint32_t freq_max_centi_khz;
    uint32_t freq_min_centi_khz;
    uint32_t event_count;
} hold_state_t;

typedef struct {
    char vrms_max[24];
    char vrms_min[24];
    char irms_max[24];
    char irms_min[24];
    char p_max[24];
    char freq_max[24];
    char freq_min[24];
    char event_count[16];
    int frozen;
} hold_view_t;

#define POWER_UI_HOLD_ACTION_CLEAR_MAX 0x01U
#define POWER_UI_HOLD_ACTION_CLEAR_MIN 0x02U

void power_model_init(void);
void monitor_view_defaults(monitor_view_t *view);
int power_model_step(monitor_view_t *view, live_metrics_t *metrics);
void fft_view_defaults(fft_view_t *view);
void power_model_build_fft_view(fft_view_t *view, const live_metrics_t *metrics);
void power_model_build_fft_view_from_frame(fft_view_t *view,
                                           const uint64_t *frame_words,
                                           uint32_t frame_word_count,
                                           const live_metrics_t *metrics);
void hold_state_defaults(hold_state_t *state);
void hold_view_defaults(hold_view_t *view);
void hold_apply_actions(hold_state_t *state, const live_metrics_t *metrics, uint32_t actions);
void hold_update(hold_state_t *state, const live_metrics_t *metrics);
void hold_build_view(const hold_state_t *state, hold_view_t *view);

#endif
