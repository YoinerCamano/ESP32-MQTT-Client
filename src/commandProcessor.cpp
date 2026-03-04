#include "CommandProcessor.hpp"
#include "DacOut.hpp"
#include <cstdlib>
#include <cstring>

CommandProcessor::CommandProcessor(dac_channel_t dac_channel)
: dac_channel_(dac_channel)
{
    dac_output_enable(dac_channel_);
}

bool CommandProcessor::parse_voltage(const char* payload, float* out_v)
{
    if (!payload || !out_v) return false;

    // 1) número directo: "6.5"
    char* end = nullptr;
    float v = strtof(payload, &end);
    if (end != payload) { *out_v = v; return true; }

    // 2) JSON simple: buscar "voltios_entrada"
    const char* key = "voltios_entrada";
    const char* p = strstr(payload, key);
    if (!p) return false;

    p = strchr(p, ':');
    if (!p) return false;
    p++;

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '"') p++;

    end = nullptr;
    v = strtof(p, &end);
    if (end == p) return false;

    *out_v = v;
    return true;
}

bool CommandProcessor::handle_voltage_payload(const char* payload, float* out_vin, uint8_t* out_dac)
{
    float vin = 0.0f;
    if (!parse_voltage(payload, &vin)) return false;

    // clamp 0..12
    if (vin < 0.0f) vin = 0.0f;
    if (vin > 12.0f) vin = 12.0f;

    // ✅ DAC queda en DacOut.hpp
    Dac_Write(dac_channel_, vin);

    if (out_vin) *out_vin = vin;
    if (out_dac) *out_dac = voltage12v_mv_to_dac(vin);

    return true;
}
