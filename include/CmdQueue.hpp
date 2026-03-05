#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
}

static constexpr size_t CMD_PAYLOAD_MAX = 512;

typedef struct {
  char payload[CMD_PAYLOAD_MAX];
} CmdMsg;

extern QueueHandle_t g_cmd_queue;