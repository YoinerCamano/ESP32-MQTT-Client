#pragma once
#include <cstdint>

extern "C" {
#include "driver/dac.h"
}


static inline uint8_t voltage12v_mv_to_dac(float v_in)
{
    if (v_in < 0.0f) v_in = 0;
    if (v_in > 12.0f) v_in = 255;

    uint8_t dac_value = (uint8_t)((85.0f/4.0f) * v_in);
    return dac_value;
}