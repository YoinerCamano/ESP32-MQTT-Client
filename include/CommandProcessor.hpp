#pragma once
#include <cstdint>

extern "C" {
#include "driver/dac.h"
}

class CommandProcessor {
public:
    explicit CommandProcessor(dac_channel_t dac_channel);

    // payload puede ser "12" o {"voltios_entrada":"3"} o {"voltios_entrada":3}
    bool handle_voltage_payload(const char* payload, float* out_vin = nullptr, uint8_t* out_dac = nullptr);

private:
    dac_channel_t dac_channel_;

    static bool parse_voltage(const char* payload, float* out_v);  
};
static void start_cmd_task(void* arg);