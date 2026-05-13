#ifndef LINUX_POWER_UI_AMP_COMP_H
#define LINUX_POWER_UI_AMP_COMP_H

#include <stdint.h>

#define AMP_COMP_UNITY_PPM 1000000U

uint32_t amp_comp_lookup_voltage_ppm(uint32_t freq_centi_khz);
uint32_t amp_comp_lookup_current_ppm(uint32_t freq_centi_khz);
uint32_t amp_comp_apply_u32(uint32_t raw_value, uint32_t gain_ppm);
int64_t amp_comp_apply_s64(int64_t raw_value, uint32_t gain_ppm);
uint32_t amp_comp_mul_ppm(uint32_t gain_a_ppm, uint32_t gain_b_ppm);

#endif
