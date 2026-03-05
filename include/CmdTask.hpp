#pragma once

#include <cstdint>

// Forward declarations (evita includes pesados en el header)
class CommandProcessor;
class MqttClient;
class ADS1115;

extern "C" {
#include "driver/dac.h"
}

/**
 * @brief Contexto que CmdTask necesita para ejecutar comandos y publicar resultados.
 *
 * - proc: convierte/valida comandos y ajusta DAC.
 * - mqtt: publica status/data.
 * - ads: lectura del ADS1115.
 * - topic_status/topic_data: topics de salida.
 * - client_id: ID único del dispositivo (para el PC).
 * - dac_channel: canal DAC usado.
 */
struct CmdContext {
    CommandProcessor* proc = nullptr;
    MqttClient* mqtt = nullptr;
    ADS1115* ads = nullptr;

    const char* topic_status = nullptr;
    const char* topic_data = nullptr;
    const char* client_id = nullptr;

    dac_channel_t dac_channel = DAC_CHAN_1;  // por defecto, pero se puede configurar
};

/**
 * @brief Inicia la tarea de comandos (CmdTask).
 *
 * Requisitos:
 * - g_cmd_queue debe existir (xQueueCreate) antes de llamar esto.
 * - ctx debe permanecer válido durante toda la ejecución.
 */
void start_cmd_task(CmdContext* ctx);