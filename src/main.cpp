#include "WifiManager.hpp"
#include "MqttClient.hpp"
#include "DacOut.hpp"
#include "ADS1115.hpp"
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
float output_mv = 0.0f;   // A0-A1
float input_volt = 0.0f;   // A2-A3

//Funciones auxiliares

static void i2c_init(); 
static void generate_client_id(char* out, size_t out_sz); 
static void on_mqtt_connected(void* user);
static void on_mqtt_message(void* user, const char* topic, const char* payload);
bool read_ads1115(void);

//main principal
extern "C" void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());      
  i2c_init();
  sensor_ADS1115 = new ADS1115(I2C_NUM_0, 0x48);

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
    bool status_sensor_ADS1115 = read_ads1115();

    if (status_sensor_ADS1115 == ESP_OK) {
     ESP_LOGI(TAG, "OUT: %.3f mV | IN: %.4f", output_mv, input_volt);

    } else {
        ESP_LOGE(TAG, "ADS error: %s", esp_err_to_name(status_sensor_ADS1115));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    mqtt->loop_once();
    vTaskDelay(pdMS_TO_TICKS(10));
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
    
  bool status_sensor_ADS1115 = read_ads1115();

    if (status_sensor_ADS1115 == ESP_OK) {

        char msg2[120];
        std::snprintf(msg2, sizeof(msg2),
                      "{\"vin_v\":%.3f,\"vout_mv\":%.3f}", input_volt, output_mv);
        mqtt->publish(TOPIC_DATA, msg2);

    } else {
      char msg3[120];
        std::snprintf(msg3, sizeof(msg3),
                      "{Verifique el sensor ADS1115}");
        mqtt->publish(TOPIC_STATUS, msg3);
    }


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

bool read_ads1115(void)
{
  esp_err_t status_sensor_ADS11156 = sensor_ADS1115->read_two_pairs(
    output_mv,
    input_volt,
    ADS1115::PGA::FS_0_256V,   // Celda
    ADS1115::PGA::FS_6_144V,   // Entrada grande
    ADS1115::SPS::SPS_128
  );
  output_mv = output_mv * 1000.0f;
  input_volt = input_volt * 3.7f;   // factor de conversión para tu divisor de voltaje

  return status_sensor_ADS11156 == ESP_OK;
}