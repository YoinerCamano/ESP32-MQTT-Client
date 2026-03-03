#include "ADS1115.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

ADS1115::ADS1115(i2c_port_t port, uint8_t addr)
: port_(port), addr_(addr) {}

esp_err_t ADS1115::write_u16(uint8_t reg, uint16_t value)
{
  uint8_t data[3] = {
    reg,
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>(value & 0xFF)
  };
  return i2c_master_write_to_device(port_, addr_, data, sizeof(data), pdMS_TO_TICKS(100));
}

esp_err_t ADS1115::read_u16(uint8_t reg, uint16_t& value)
{
  uint8_t out = reg;
  uint8_t in[2] = {0, 0};

  esp_err_t err = i2c_master_write_read_device(
      port_, addr_, &out, 1, in, 2, pdMS_TO_TICKS(100));

  if (err != ESP_OK) return err;

  value = (static_cast<uint16_t>(in[0]) << 8) | static_cast<uint16_t>(in[1]);
  return ESP_OK;
}

float ADS1115::fsr_from_pga(PGA pga)
{
  switch (pga) {
    case PGA::FS_6_144V: return 6.144f;
    case PGA::FS_4_096V: return 4.096f;
    case PGA::FS_2_048V: return 2.048f;
    case PGA::FS_1_024V: return 1.024f;
    case PGA::FS_0_512V: return 0.512f;
    case PGA::FS_0_256V: return 0.256f;
    default: return 0.256f;
  }
}

uint32_t ADS1115::wait_ms_from_sps(SPS sps)
{
  // Conversión en single-shot tarda ~1/SPS. Aquí damos margen.
  // 128 SPS => ~7.8ms, 860 SPS => ~1.2ms
  switch (sps) {
    case SPS::SPS_860: return 2;   // margen
    case SPS::SPS_128:
    default:           return 10;  
  }
}

esp_err_t ADS1115::read_diff(float& volts, DiffPair pair, PGA pga, SPS sps)
{
  int16_t raw = 0;
  return read_diff(raw, volts, pair, pga, sps);
}

esp_err_t ADS1115::read_diff(int16_t& raw, float& volts, DiffPair pair, PGA pga, SPS sps)
{
  // MUX diferencial para ADS1115:
  // 0: A0-A1, 1: A0-A3, 2: A1-A3, 3: A2-A3
  uint8_t mux = static_cast<uint8_t>(pair);
  if (mux > 3) return ESP_ERR_INVALID_ARG;

  // Config igual a tu MicroPython:
  // OS=1 start (bit15)
  // MUX=mux (bits14..12)
  // PGA=pga (bits11..9)
  // MODE=1 single-shot (bit8)
  // DR=sps (bits7..5)
  // Comparator disabled => 0x0003 (bits1..0 = 11)
  uint16_t config =
      (1u << 15) |
      (static_cast<uint16_t>(mux) << 12) |
      (static_cast<uint16_t>(static_cast<uint8_t>(pga)) << 9) |
      (1u << 8) |
      (static_cast<uint16_t>(static_cast<uint8_t>(sps)) << 5) |
      0x0003;

  esp_err_t err = write_u16(REG_CFG, config);
  if (err != ESP_OK) return err;

  vTaskDelay(pdMS_TO_TICKS(wait_ms_from_sps(sps)));

  uint16_t u16 = 0;
  err = read_u16(REG_CONV, u16);
  if (err != ESP_OK) return err;

  // Signo 16-bit
  raw = static_cast<int16_t>(u16);

  float fsr = fsr_from_pga(pga);
  volts = static_cast<float>(raw) * (fsr / 32768.0f);
  return ESP_OK;
}

// implementación para leer ambos pares de una sola llamada
esp_err_t ADS1115::read_two_pairs(float& v_a0_a1,
                                 float& v_a2_a3,
                                 PGA pga_a0_a1,
                                 PGA pga_a2_a3,
                                 SPS sps)
{
    esp_err_t err = read_diff(v_a0_a1, DiffPair::A0_A1, pga_a0_a1, sps);
    if (err != ESP_OK) return err;
    return read_diff(v_a2_a3, DiffPair::A2_A3, pga_a2_a3, sps);
}