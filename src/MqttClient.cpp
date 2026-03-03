#include "MqttClient.hpp"
#include <cstring>

extern "C" {
#include "esp_log.h"
}

static const char* TAG = "MQTT";

static inline void safe_copy(char* dst, size_t dst_sz, const char* src) {
  if (!dst || dst_sz == 0) return;
  if (!src) { dst[0] = '\0'; return; }
  std::strncpy(dst, src, dst_sz - 1);
  dst[dst_sz - 1] = '\0';
}

MqttClient::MqttClient(const char* uri,
                       const char* client_id,
                       const char* topic_sub)
: uri_(uri)
{
  safe_copy(client_id_, sizeof(client_id_), client_id);
  safe_copy(topic_sub_, sizeof(topic_sub_), topic_sub);
}

void MqttClient::set_on_connected(on_connected_cb_t cb, void* user) {
  on_connected_ = cb;
  on_connected_user_ = user;
}

void MqttClient::set_on_message(on_message_cb_t cb, void* user) {
  on_message_ = cb;
  on_message_user_ = user;
}

esp_err_t MqttClient::start()
{
  esp_mqtt_client_config_t cfg{};
  cfg.broker.address.uri = uri_;
  cfg.credentials.client_id = client_id_;

  client_ = esp_mqtt_client_init(&cfg);
  if (!client_) return ESP_FAIL;

  esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY, &MqttClient::mqtt_event_handler, this);

  ESP_LOGI(TAG, "Start: %s (id=%s)", uri_, client_id_);
  return esp_mqtt_client_start(client_);
}

bool MqttClient::publish(const char* topic, const char* payload, int qos, bool retain)
{
  if (!client_ || !connected_ || !topic || !payload) return false;

  int msg_id = esp_mqtt_client_publish(client_,
                                       topic,
                                       payload,
                                       0,      // auto length
                                       qos,
                                       retain);
  return msg_id >= 0;
}

void MqttClient::mqtt_event_handler(void* handler_args,
                                    esp_event_base_t,
                                    int32_t,
                                    void* event_data)
{
  auto* self = static_cast<MqttClient*>(handler_args);
  self->handle_event(reinterpret_cast<esp_mqtt_event_handle_t>(event_data));
}

void MqttClient::handle_event(esp_mqtt_event_handle_t event)
{
  switch (event->event_id) {

    case MQTT_EVENT_CONNECTED:
      connected_ = true;
      ESP_LOGI(TAG, "CONNECTED");

      esp_mqtt_client_subscribe(client_, topic_sub_, 0);

      if (on_connected_) on_connected_(on_connected_user_);
      break;

    case MQTT_EVENT_DISCONNECTED:
      connected_ = false;
      ESP_LOGW(TAG, "DISCONNECTED");
      break;

    case MQTT_EVENT_DATA:
      push_rx(event->topic, event->topic_len, event->data, event->data_len);
      break;

    default:
      break;
  }
}

void MqttClient::push_rx(const char* topic, int topic_len, const char* payload, int payload_len)
{
  if (count_ == QSIZE) return;

  RxMsg& m = q_[tail_];

  int tlen = (topic_len < (int)sizeof(m.topic) - 1) ? topic_len : (int)sizeof(m.topic) - 1;
  int plen = (payload_len < (int)sizeof(m.payload) - 1) ? payload_len : (int)sizeof(m.payload) - 1;

  std::memcpy(m.topic, topic, tlen);
  m.topic[tlen] = '\0';

  std::memcpy(m.payload, payload, plen);
  m.payload[plen] = '\0';

  tail_ = (tail_ + 1) % QSIZE;
  count_++;
}

bool MqttClient::pop_rx(RxMsg& out)
{
  if (count_ == 0) return false;

  out = q_[head_];
  head_ = (head_ + 1) % QSIZE;
  count_--;
  return true;
}

void MqttClient::loop_once()
{
  if (!on_message_) return;

  RxMsg m;
  while (pop_rx(m)) {
    on_message_(on_message_user_, m.topic, m.payload);
  }
}