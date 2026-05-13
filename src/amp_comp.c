#include "amp_comp.h"

typedef struct {
    uint32_t freq_centi_khz;
    uint32_t gain_ppm;
} amp_comp_point_t;

static const amp_comp_point_t voltage_comp_table[] = {
    {    0U, 1000000U },
    {  500U, 1000000U },
    { 1000U, 1001524U },
    { 1500U, 1004393U },
    { 2000U, 1009405U },
    { 2500U, 1014350U },
    { 3000U, 1019858U },
    { 3500U, 1025986U },
    { 4000U, 1034015U },
    { 4500U, 1043038U },
    { 5000U, 1052010U },
    { 5500U, 1062167U },
    { 6000U, 1073046U },
    { 6500U, 1085180U },
    { 7000U, 1098922U },
    { 7500U, 1112263U },
    { 8000U, 1127235U },
    { 8500U, 1143012U },
    { 9000U, 1159801U },
    { 9500U, 1176721U },
    {10000U, 1195282U },
    {10500U, 1208410U },
    {11000U, 1228738U },
    {11500U, 1248753U },
    {12000U, 1270290U },
};

static const amp_comp_point_t current_comp_table[] = {
    {    0U, 1020408U },
    {  500U, 1020408U },
    { 1000U, 1021963U },
    { 1500U, 1024891U },
    { 2000U, 1024777U },
    { 2500U, 1029797U },
    { 3000U, 1035389U },
    { 3500U, 1041610U },
    { 4000U, 1049761U },
    { 4500U, 1058922U },
    { 5000U, 1068030U },
    { 5500U, 1078342U },
    { 6000U, 1092715U },
    { 6500U, 1105071U },
    { 7000U, 1121349U },
    { 7500U, 1134962U },
    { 8000U, 1159707U },
    { 8500U, 1178363U },
    { 9000U, 1200622U },
    { 9500U, 1225751U },
    {10000U, 1250295U },
    {10500U, 1270673U },
    {11000U, 1293408U },
    {11500U, 1314477U },
    {12000U, 1337147U },
};

static uint32_t amp_comp_lookup_table_ppm(const amp_comp_point_t *table, uint32_t count, uint32_t freq_centi_khz)
{
    uint32_t idx;

    if(count == 0U) {
        return AMP_COMP_UNITY_PPM;
    }

    if(freq_centi_khz <= table[0].freq_centi_khz) {
        return table[0].gain_ppm;
    }

    if(freq_centi_khz >= table[count - 1U].freq_centi_khz) {
        return table[count - 1U].gain_ppm;
    }

    for(idx = 1U; idx < count; ++idx) {
        const amp_comp_point_t *lo = &table[idx - 1U];
        const amp_comp_point_t *hi = &table[idx];

        if(freq_centi_khz <= hi->freq_centi_khz) {
            uint32_t span = hi->freq_centi_khz - lo->freq_centi_khz;
            int64_t delta_gain = (int64_t)hi->gain_ppm - (int64_t)lo->gain_ppm;
            uint32_t offset = freq_centi_khz - lo->freq_centi_khz;

            if(span == 0U) {
                return hi->gain_ppm;
            }

            return (uint32_t)((int64_t)lo->gain_ppm +
                              ((delta_gain * (int64_t)offset + (int64_t)(span / 2U)) / (int64_t)span));
        }
    }

    return table[count - 1U].gain_ppm;
}

uint32_t amp_comp_apply_u32(uint32_t raw_value, uint32_t gain_ppm)
{
    return (uint32_t)(((uint64_t)raw_value * (uint64_t)gain_ppm + 500000U) / 1000000U);
}

int64_t amp_comp_apply_s64(int64_t raw_value, uint32_t gain_ppm)
{
    int64_t scaled;

    if(raw_value >= 0) {
        scaled = raw_value * (int64_t)gain_ppm + 500000LL;
        return scaled / 1000000LL;
    }

    scaled = (-raw_value) * (int64_t)gain_ppm + 500000LL;
    return -(scaled / 1000000LL);
}

uint32_t amp_comp_mul_ppm(uint32_t gain_a_ppm, uint32_t gain_b_ppm)
{
    return (uint32_t)(((uint64_t)gain_a_ppm * (uint64_t)gain_b_ppm + 500000U) / 1000000U);
}

uint32_t amp_comp_lookup_voltage_ppm(uint32_t freq_centi_khz)
{
    return amp_comp_lookup_table_ppm(
        voltage_comp_table,
        (uint32_t)(sizeof(voltage_comp_table) / sizeof(voltage_comp_table[0])),
        freq_centi_khz);
}

uint32_t amp_comp_lookup_current_ppm(uint32_t freq_centi_khz)
{
    return amp_comp_lookup_table_ppm(
        current_comp_table,
        (uint32_t)(sizeof(current_comp_table) / sizeof(current_comp_table[0])),
        freq_centi_khz);
}
