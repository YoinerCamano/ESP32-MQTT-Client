#pragma once
#include <cstdint>

extern "C" {
#include "driver/i2c.h"
#include "esp_err.h"
}

class ADS1115 {
public:
  static constexpr uint8_t REG_CONV = 0x00;
  static constexpr uint8_t REG_CFG  = 0x01;

  enum class DiffPair : uint8_t { A0_A1 = 0, A0_A3 = 1, A1_A3 = 2, A2_A3 = 3 };

  enum class PGA : uint8_t {
    FS_6_144V = 0x00,
    FS_4_096V = 0x01,
    FS_2_048V = 0x02,
    FS_1_024V = 0x03,
    FS_0_512V = 0x04,
    FS_0_256V = 0x05
  };

  enum class SPS : uint8_t {
    SPS_128 = 0x04,
    SPS_860 = 0x07
  };

  ADS1115(i2c_port_t port, uint8_t addr = 0x48);

  esp_err_t read_diff(float& volts,
                      DiffPair pair,
                      PGA pga = PGA::FS_0_256V,
                      SPS sps = SPS::SPS_128);

  esp_err_t read_diff(int16_t& raw, float& volts,
                      DiffPair pair,
                      PGA pga = PGA::FS_0_256V,
                      SPS sps = SPS::SPS_128);

  // ✅ Nuevo: hace los dos llamados (A0-A1 y A2-A3) en una sola función
  esp_err_t read_two_pairs(float& v_a0_a1,
                           float& v_a2_a3,
                           PGA pga_a0_a1 = PGA::FS_0_256V,
                           PGA pga_a2_a3 = PGA::FS_6_144V,
                           SPS sps = SPS::SPS_128);

private:
  i2c_port_t port_;
  uint8_t addr_;

  esp_err_t write_u16(uint8_t reg, uint16_t value);
  esp_err_t read_u16(uint8_t reg, uint16_t& value);

  static float fsr_from_pga(PGA pga);
  static uint32_t wait_ms_from_sps(SPS sps);
};