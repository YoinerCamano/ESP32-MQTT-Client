#pragma once
#include <cstdint>
#include <cstddef>

extern "C" {
#include "esp_err.h"
#include "mqtt_client.h"
#include "esp_event.h"
}

class MqttClient {
public:
  // Callbacks sin std::function (menos flash)
  using on_connected_cb_t = void (*)(void* user);
  using on_message_cb_t   = void (*)(void* user, const char* topic, const char* payload);

  MqttClient(const char* uri,
             const char* client_id,
             const char* topic_sub);

  esp_err_t start();

  bool publish(const char* topic, const char* payload, int qos = 0, bool retain = false);

  void set_on_connected(on_connected_cb_t cb, void* user);
  void set_on_message(on_message_cb_t cb, void* user);

  void loop_once();

private:
  static void mqtt_event_handler(void* handler_args,
                                 esp_event_base_t base,
                                 int32_t event_id,
                                 void* event_data);

  void handle_event(esp_mqtt_event_handle_t event);

  struct RxMsg {
    char topic[96];
    char payload[192];
  };

  void push_rx(const char* topic, int topic_len, const char* payload, int payload_len);
  bool pop_rx(RxMsg& out);

private:
  const char* uri_;

  // guardamos copias internas (evita lifetimes raros)
  char client_id_[32];
  char topic_sub_[96];

  esp_mqtt_client_handle_t client_{nullptr};
  volatile bool connected_{false};

  on_connected_cb_t on_connected_{nullptr};
  void* on_connected_user_{nullptr};

  on_message_cb_t on_message_{nullptr};
  void* on_message_user_{nullptr};

  static constexpr int QSIZE = 8;
  RxMsg q_[QSIZE];
  uint8_t head_{0}, tail_{0}, count_{0};
};