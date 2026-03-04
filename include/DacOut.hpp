#pragma once
#include <cstdint>

extern "C" {
#include "driver/dac.h"
}

static inline uint8_t voltage12v_mv_to_dac(float v_in)
{
    if (v_in < 0.0f) v_in = 0.0f;
    if (v_in > 12.0f) v_in = 12.0f;

    if (v_in < 5.0f){
        return (uint8_t)((21.25f * v_in)- 6.9f); 
    }
    else if (v_in < 6.0f){
        return (uint8_t)((21.25f * v_in)- 6.0f); 
    }
    else if (v_in < 10.0f){
        return (uint8_t)((21.25f * v_in)- 4.0f); 

    }else
    {
        return (uint8_t)((21.25f * v_in)- 3.3f); 
    };
}

static inline void Dac_Write(dac_channel_t Channel, float v_in = 3.3f)
{
    dac_output_voltage(Channel, voltage12v_mv_to_dac(v_in));
}
static inline void dac_init(dac_channel_t Channel, uint8_t dac_value = 0)
{
    dac_output_enable(Channel);
    dac_output_voltage(Channel, dac_value);
}