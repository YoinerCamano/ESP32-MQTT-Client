#include "WifiManager.hpp"
#include <cstring>

extern "C" {
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
}

static const char* TAG = "WIFI";
volatile bool WifiManager::connected_ = false;

WifiManager::WifiManager(std::string ssid, std::string password)
: ssid_(ssid), password_(password) {}

bool WifiManager::is_connected() const {
  return connected_;
}

void WifiManager::event_handler(void*,
                                esp_event_base_t event_base,
                                int32_t event_id,
                                void*)
{
  if (event_base == WIFI_EVENT &&
      event_id == WIFI_EVENT_STA_START)
    esp_wifi_connect();

  if (event_base == WIFI_EVENT &&
      event_id == WIFI_EVENT_STA_DISCONNECTED) {
    connected_ = false;
    ESP_LOGW(TAG, "Reintentando conexión...");
    esp_wifi_connect();
  }

  if (event_base == IP_EVENT &&
      event_id == IP_EVENT_STA_GOT_IP) {
    connected_ = true;
    ESP_LOGI(TAG, "WiFi conectado ✅");
  }
}

esp_err_t WifiManager::start()
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(
    esp_event_handler_register(WIFI_EVENT,
                               ESP_EVENT_ANY_ID,
                               &WifiManager::event_handler,
                               nullptr));

  ESP_ERROR_CHECK(
    esp_event_handler_register(IP_EVENT,
                               IP_EVENT_STA_GOT_IP,
                               &WifiManager::event_handler,
                               nullptr));

  wifi_config_t wifi_config{};
  std::strncpy((char*)wifi_config.sta.ssid,
               ssid_.c_str(),
               sizeof(wifi_config.sta.ssid));

  std::strncpy((char*)wifi_config.sta.password,
               password_.c_str(),
               sizeof(wifi_config.sta.password));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Conectando a %s ...", ssid_.c_str());

  return ESP_OK;
}