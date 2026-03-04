#include "CmdQueue.hpp"
#include "CommandProcessor.hpp"

extern "C" {
#include "freertos/task.h"
}

static void cmd_task(void* arg)
{
  auto* proc = static_cast<CommandProcessor*>(arg);

  const TickType_t window = pdMS_TO_TICKS(1000);   // 1s
  TickType_t window_start = xTaskGetTickCount();
  int processed = 0;

  CmdMsg msg{};

  while (true) {
    if (xQueueReceive(g_cmd_queue, &msg, portMAX_DELAY) == pdTRUE) {

      TickType_t now = xTaskGetTickCount();
      if ((now - window_start) >= window) {
        window_start = now;
        processed = 0;
      }

      if (processed >= 3) {
        // Anti-spam: descarta
        continue;
      }

      processed++;
      proc->handle_voltage_payload(msg.payload);  // aquí haces DAC + ADS + publish
    }
  }
}

void start_cmd_task(CommandProcessor* proc)
{
  // Stack 4096 suele ir bien; si tu ADS/MQTT hace más cosas, sube a 6144
  xTaskCreatePinnedToCore(cmd_task, "cmd_task", 4096, proc, 10, nullptr, 1);
}