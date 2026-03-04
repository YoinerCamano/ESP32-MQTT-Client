#include "WifiManager.hpp"
#include "MqttClient.hpp"
#include "ADS1115.hpp"
#include "CommandProcessor.hpp"
#include "DacOut.hpp"
#include <cstdio>
#include <cstring>

extern "C" {
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
}

static const char* WIFI_SSID = "BASCOSTA_INNOVACIONES";
static const char* WIFI_PASS = "Bascosta2025";

static const char* MQTT_URI  = "mqtt://192.168.0.28:1883";
static const char* TOPIC_CMD = "prueba/esp32/cmd";
static const char* TOPIC_STATUS = "prueba/esp32/status";
static const char* TOPIC_DATA = "prueba/esp32/data";

static const char* TAG = "MAIN"; 

static ADS1115* sensor_ADS1115 = nullptr; // Sensor global para usar en callbacks
static char g_client_id[32];  // Client ID global para usar en callbacks

typedef struct {
  float voltage_input;
  float voltage_output;
} SensorData;

//Funciones auxiliares

static void i2c_init();
static void generate_client_id(char* out, size_t out_sz); 
static void on_mqtt_connected(void* user);
static void on_mqtt_message(void* user, const char* topic, const char* payload);
bool read_ads1115(SensorData& data);

//main principal
extern "C" void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());      
  i2c_init();
  sensor_ADS1115 = new ADS1115(I2C_NUM_0, 0x48);
  SensorData sensorData;
  dac_init(DAC_CHAN_1, 68);

  auto* wifi = new WifiManager(WIFI_SSID, WIFI_PASS);
  wifi->start();

  while (!wifi->is_connected()){
    vTaskDelay(pdMS_TO_TICKS(500));};
  
  generate_client_id(g_client_id, sizeof(g_client_id));
  ESP_LOGI("MAIN", "Client ID: %s", g_client_id);

  auto* mqtt = new MqttClient(MQTT_URI, g_client_id, TOPIC_CMD);

  mqtt->set_on_connected(&on_mqtt_connected, mqtt);
  mqtt->set_on_message(&on_mqtt_message, mqtt);
  ESP_ERROR_CHECK(mqtt->start());
 
  while (true) {       

    for (uint8_t i = 1; i < 13; i++) {
      Dac_Write(DAC_CHAN_1, i); // Ajusta DAC al voltaje de entrada leído
      vTaskDelay(pdMS_TO_TICKS(1500));
      bool status_sensor_ADS1115 = read_ads1115(sensorData);
      if (status_sensor_ADS1115) {
      ESP_LOGI(TAG, "OUT: %.3f mV | IN: %.4f | voltaje: %d", sensorData.voltage_output, sensorData.voltage_input, i);

      } else {
          ESP_LOGE(TAG, "ADS error: Verifique el sensor");
      }   
    mqtt->loop_once();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
}

static void i2c_init()
{
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_22;
    conf.scl_io_num = GPIO_NUM_21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

static void generate_client_id(char* out, size_t out_sz)
{
    uint8_t mac[6]={0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    std::snprintf(out, out_sz, "ESP_%02X%02X%02X%02X%02X%02X", mac[0],mac[1],mac[2], mac[3], mac[4], mac[5]);
}

static void on_mqtt_connected(void* user)
{
  auto* mqtt = static_cast<MqttClient*>(user);
 
  char msg1[96];
  std::snprintf(msg1, sizeof(msg1),
                "{\"status\":\"conectado\",\"id\":\"%s\"}", g_client_id);
  mqtt->publish(TOPIC_STATUS, msg1);
}

static void on_mqtt_message(void* user, const char* topic, const char* payload)
{    
  auto* mqtt = static_cast<MqttClient*>(user);
    ESP_LOGI("MAIN", "RX topic=%s payload=%s", topic, payload);

    char ack[128];
    snprintf(ack, sizeof(ack),
             "{\"ack\":true,\"id\":\"%s\"}",
             g_client_id);

    mqtt->publish("prueba/esp32/status", ack);
}

bool read_ads1115(SensorData& data)
{
  float output_mv,input_volt = 0.0f;
  if (sensor_ADS1115 == nullptr) return false;
  esp_err_t status_sensor_ADS11156 = sensor_ADS1115->read_two_pairs(
    output_mv,
    input_volt,
    ADS1115::PGA::FS_0_256V,   // Celda
    ADS1115::PGA::FS_6_144V,   // Entrada grande
    ADS1115::SPS::SPS_128
  );
  
if (status_sensor_ADS11156 == ESP_OK) {
    data.voltage_output = output_mv * 1000.0f;
    data.voltage_input = input_volt * 3.7f; 
    return true;
  }
else {
    ESP_LOGE(TAG, "Error leyendo ADS1115: %s", esp_err_to_name(status_sensor_ADS11156));
  return false;
};
};