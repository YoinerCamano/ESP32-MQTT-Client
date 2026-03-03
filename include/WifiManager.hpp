#pragma once
#include <string>

extern "C" {
#include "esp_err.h"
#include "esp_event_base.h"
}

class WifiManager {
public:
  WifiManager(std::string ssid, std::string password);
  esp_err_t start();
  bool is_connected() const;

private:
  std::string ssid_;
  std::string password_;

  static void event_handler (void* arg,
                            esp_event_base_t event_base,
                            int32_t event_id,
                            void* event_data);

  static volatile bool connected_;
};