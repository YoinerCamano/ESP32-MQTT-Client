#include "WifiManager.hpp"
#include "MqttClient.hpp"
#include "ADS1115.hpp"
#include "CommandProcessor.hpp"
#include "DacOut.hpp"
#include "CmdQueue.hpp"
#include "CmdTask.hpp"

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
}

static const char* WIFI_SSID = "BASCOSTA_INNOVACIONES";
static const char* WIFI_PASS = "Bascosta2025";

static const char* MQTT_URI  = "mqtt://192.168.0.28:1883";
static const char* TOPIC_GENERAL_STATUS = "digitaltwin/esp32/general/status";

static char g_topic_cmd[96];
static char g_topic_status[96];
static char g_topic_data[96];

static const char* TAG = "MAIN"; 

//variables globales
static ADS1115* sensor_ADS1115 = nullptr; // Sensor global para usar en callbacks
static CommandProcessor* g_proc = nullptr;    // Processor global para CmdTask
static char g_client_id[32];  // Client ID global para usar en callbacks



//Funciones auxiliares

static void i2c_init();
static void generate_client_id(char* out, size_t out_sz); 
static void on_mqtt_connected(void* user);
static void on_mqtt_message(void* user, const char* topic, const char* payload);


// Contexto para CmdTask
static CmdContext g_cmd_ctx{};

//main principal
extern "C" void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());      
  i2c_init();

  //sensor ADS1115 
  sensor_ADS1115 = new ADS1115(I2C_NUM_0, 0x48);
  
  //DAC + CommandProcessor
  dac_init(DAC_CHAN_1, 0); // Inicializa DAC con valor 0
  Dac_Write(DAC_CHAN_1, 3.3f); // DAC a 3.3V 
  g_proc = new CommandProcessor(DAC_CHAN_1);

  //WiFi
  auto* wifi = new WifiManager(WIFI_SSID, WIFI_PASS);
  wifi->start();

  while (!wifi->is_connected())
  {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  
  //Generacion de client ID único basado en MAC
  generate_client_id(g_client_id, sizeof(g_client_id));
  ESP_LOGI("MAIN", "Client ID: %s", g_client_id);

  // Topics MQTT específicos para este dispositivo
  std::snprintf(g_topic_cmd, sizeof(g_topic_cmd), "digitaltwin/esp32/%s/cmd", g_client_id);
  std::snprintf(g_topic_status, sizeof(g_topic_status), "digitaltwin/esp32/%s/status", g_client_id);
  std::snprintf(g_topic_data, sizeof(g_topic_data), "digitaltwin/esp32/%s/data", g_client_id);

  // Topics MQTT específicos para este dispositivo
  auto* mqtt = new MqttClient(MQTT_URI, g_client_id, g_topic_cmd);
  mqtt->set_on_connected(&on_mqtt_connected, mqtt);
  mqtt->set_on_message(&on_mqtt_message, mqtt);
  ESP_ERROR_CHECK(mqtt->start());
 
  // Cola de comandos (FIFO) - guarda comandos mientras se esta en diagnóstico
  g_cmd_queue = xQueueCreate(20, sizeof(CmdMsg));
  if (!g_cmd_queue) {
    ESP_LOGE(TAG, "No se pudo crear g_cmd_queue");
    abort();
  }

  // Arranca CmdTask (máquina de estados del diagnóstico)
  g_cmd_ctx.proc         = g_proc;
  g_cmd_ctx.mqtt         = mqtt;
  g_cmd_ctx.ads          = sensor_ADS1115;
  g_cmd_ctx.topic_status = g_topic_status;
  g_cmd_ctx.topic_data   = g_topic_data;
  g_cmd_ctx.client_id    = g_client_id;
  g_cmd_ctx.dac_channel  = DAC_CHAN_1;
  start_cmd_task(&g_cmd_ctx);

  while (true) {
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

  // 1) general
  char gen[160];
  std::snprintf(gen, sizeof(gen),
    "{\"status\":\"online\",\"id\":\"%s\"}", g_client_id);
  mqtt->publish(TOPIC_GENERAL_STATUS, gen);

}

static void on_mqtt_message(void* user, const char* topic, const char* payload)
{
  auto* mqtt = static_cast<MqttClient*>(user);
  ESP_LOGI("MAIN", "RX topic=%s payload=%s", topic ? topic : "null", payload ? payload : "null");

  if (!payload || !g_cmd_queue) return;

  CmdMsg msg{};
  // Copia segura
  std::snprintf(msg.payload, sizeof(msg.payload), "%s", payload);

  // Encolar (no bloquear)
  if (xQueueSend(g_cmd_queue, &msg, 0) != pdTRUE) {
    char busy[180];
    std::snprintf(busy, sizeof(busy),
      "{\"ack\":false,\"err\":\"queue_full\",\"state\":\"busy\",\"id\":\"%s\"}", g_client_id);
    mqtt->publish(g_topic_status, busy);
  }
}

