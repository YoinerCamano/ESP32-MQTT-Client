/**
 * Proyecto: MODULO DE CERTIFICACION ADL
 *
 * Descripción:
 * Este firmware permite controlar y diagnosticar un sistema de pesaje
 *
 * Funcionalidades:
 * - Control de voltaje mediante DAC
 * - Lectura de señales analógicas con ADS1115
 * - Comunicación remota mediante MQTT
 * - Ejecución de diagnósticos automatizados (diag_sweep)
 *
 * Arquitectura:
 * WiFi -> MQTT -> CmdQueue -> CmdTask -> DAC + ADS1115
 */

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

static const char* MQTT_URI  = "mqtt://192.168.0.28:1883";                      // broker local en la misma red
static const char* TOPIC_GENERAL_STATUS = "digitaltwin/esp32/general/status";   // topic para publicar estados generales (ej. conexión WiFi, errores críticos, etc.)

static char g_topic_cmd[96]; // topic específico para comandos de este dispositivo, con client_id en el topic
static char g_topic_status[96]; // topic específico para estados de este dispositivo, con client_id en el topic
static char g_topic_data[96]; // topic específico para datos de este dispositivo, con client_id en el topic

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
  /*
  * @brief Función principal
  * - Inicializa NVS, I2C, ADC, DAC, WiFi, MQTT y la tarea de comandos.
  * - Genera un client ID único basado en la MAC.
  * - Configura topics MQTT específicos para este dispositivo.
  * - Entra en un loop de espera de mensajes MQTT.
  * Nota: el diagnóstico se maneja completamente dentro de CmdTask, que responde a comandos encolados desde MQTT.
  */
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
    /*
    * @brief Bucle principal
    * - Llama a mqtt->loop_once() para procesar mensajes MQTT.
    * - Espera 10 ms antes de la siguiente iteración.
    * Nota: la lógica principal de diagnóstico y respuesta a comandos se maneja dentro de CmdTask, que es más responsiva a comandos stop/status durante la ejecución del diagnóstico.
    */
    mqtt->loop_once();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void i2c_init()
/*
 * @brief Inicializa el bus I2C.
 * Configura los pines SDA y SCL, habilita las resistencias pull-up internas
 * y establece la velocidad del bus.
 * funcionalidad:
 * - Configura el puerto I2C0 en modo maestro.
 * - Habilita las resistencias pull-up internas para SDA y SCL.
 * - Establece la velocidad del bus a 400 kHz.
  * Nota: esta función debe ser llamada antes de usar cualquier dispositivo I2C (como el ADS1115).
 */
{
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.scl_io_num = GPIO_NUM_22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

static void generate_client_id(char* out, size_t out_sz)
/**
 * @brief Genera un Client ID único basado en la MAC del ESP32.
 * Este ID se usa para identificar el dispositivo en el broker MQTT.
 * @param out buffer donde se escribirá el ID
 * @param out_sz tamaño del buffer
 */
{
    uint8_t mac[6]={0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    std::snprintf(out, out_sz, "ESP_%02X%02X%02X%02X%02X%02X", mac[0],mac[1],mac[2], mac[3], mac[4], mac[5]);
}

static void on_mqtt_connected(void* user)
/**
 * @brief Callback llamado cuando el cliente MQTT se conecta al broker.
 * @param user puntero al cliente MQTT
 * Funcionalidad:
 * - Publica un mensaje de estado "online" en el topic general.
 * - El mensaje incluye el client ID para identificar el dispositivo.
 * Nota: este callback se ejecuta en el contexto del evento MQTT, no en CmdTask.
 *       Por eso no se accede a CmdContext ni se encolan comandos aquí.
 */
{
  auto* mqtt = static_cast<MqttClient*>(user);

  // 1) general
  char gen[160];
  std::snprintf(gen, sizeof(gen),
    "{\"status\":\"online\",\"id\":\"%s\"}", g_client_id); 
  mqtt->publish(TOPIC_GENERAL_STATUS, gen);

}

static void on_mqtt_message(void* user, const char* topic, const char* payload)
/*
 * @brief Callback llamado cuando se recibe un mensaje MQTT.
 * @param user puntero al cliente MQTT
 * @param topic topic del mensaje recibido
 * @param payload contenido del mensaje recibido
 * Funcionalidad:
 * - Registra el mensaje recibido en el log.
 * - Encola el mensaje en g_cmd_queue para su procesamiento por CmdTask.
 * Nota: este callback se ejecuta en el contexto del evento MQTT, no en CmdTask.
 *       Por eso no se accede directamente a CmdContext aquí.
 */
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

