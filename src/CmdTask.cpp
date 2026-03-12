#include "CmdTask.hpp"
#include "CmdQueue.hpp"
#include "CommandProcessor.hpp"
#include "MqttClient.hpp"
#include "ADS1115.hpp"

#include <cstdio>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cJSON.h"
}

static const char* TAG = "CmdTask";

// ================== Ajustes / límites ==================
static constexpr float AMP_DIV_FACTOR = 3.72f;   // factor del divisor (para convertir lectura "input_volt" a V real)
static constexpr float MIN_VIN_CMD = 1.0f;
static constexpr float MAX_VIN_CMD = 12.0f;

static constexpr int MAX_STEPS = 64;

// Seguridad para muestreo
static constexpr int MIN_SAMPLE_INTERVAL_MS = 5;
static constexpr int MAX_SAMPLE_INTERVAL_MS = 5000;

static constexpr int MIN_SAMPLES_PER_STEP = 1;
static constexpr int MAX_SAMPLES_PER_STEP = 500;

// Chunking (payload “manejable”)
static constexpr int SAMPLES_PER_CHUNK = 20; // 10–25 suele ir bien
static constexpr int JSON_BUF_SZ = 1400;

// ================== Estado / Config ==================
enum class DiagState : uint8_t { IDLE = 0, RUNNING };

struct DiagConfig {
    float vlist[MAX_STEPS]{0.0f};
    int steps = 0;

    int settle_ms = 800;
    int sample_interval_ms = 100;
    int samples_per_step = 20;

    bool raw_ints = true; // siempre true aquí
}; // Configuración del diagnóstico, parseada desde el comando MQTT

struct Runtime {
    DiagState state = DiagState::IDLE;
    bool stop_requested = false;

    int seq = 0;
    int current_step = -1;
    DiagConfig cfg{};
};// Estado de ejecución del diagnóstico, manejado dentro de CmdTask

// ================== Publish helpers ==================
static inline const char* cid(CmdContext* ctx) { return (ctx && ctx->client_id) ? ctx->client_id : "ESP"; }

static void pub_status(CmdContext* ctx, const char* json)
/*
 * @brief Publica un mensaje de estado en el topic de status.
 * @param ctx Contexto que contiene la información necesaria para publicar.
 * @param json Cadena JSON que representa el mensaje de estado a publicar.
 * Funcionalidad:
 * - Verifica que el contexto y el cliente MQTT estén disponibles.
 * - Publica el mensaje JSON en el topic de status específico del dispositivo.
 */
{
    if (ctx && ctx->mqtt && ctx->topic_status) ctx->mqtt->publish(ctx->topic_status, json);
}

static void pub_data(CmdContext* ctx, const char* json) {
    /*
     * @brief Publica un mensaje de datos en el topic de data.
     * @param ctx Contexto que contiene la información necesaria para publicar.
     * @param json Cadena JSON que representa el mensaje de datos a publicar.
     * Funcionalidad:
     * - Verifica que el contexto y el cliente MQTT estén disponibles.
     * - Publica el mensaje JSON en el topic de data específico del dispositivo.
     */
    if (ctx && ctx->mqtt && ctx->topic_data) ctx->mqtt->publish(ctx->topic_data, json);
}

static void pub_parse_err(CmdContext* ctx, const char* reason){
    /*
     * @brief Publica un mensaje de error de parseo en el topic de status.
     * @param ctx Contexto que contiene la información necesaria para publicar.
     * @param reason Razón del error de parseo (opcional).
     * Funcionalidad:
     * - Construye un mensaje JSON que indica un error de parseo, incluyendo la razón y el client ID.
     * - Publica el mensaje JSON en el topic de status específico del dispositivo.
     */
    char out[256];
    std::snprintf(out, sizeof(out),
        "{\"ack\":false,\"err\":\"parse_err\",\"reason\":\"%s\",\"id\":\"%s\"}",
        reason ? reason : "invalid_json", cid(ctx));
    pub_status(ctx, out);
}

static void pub_busy(CmdContext* ctx, const char* reason){
    /*
     * @brief Publica un mensaje de estado "busy" en el topic de status.
     * @param ctx Contexto que contiene la información necesaria para publicar.
     * @param reason Razón por la cual el sistema está ocupado (opcional).
     * Funcionalidad:
     * - Construye un mensaje JSON que indica que el sistema está ocupado, incluyendo la razón y el client ID.
     * - Publica el mensaje JSON en el topic de status específico del dispositivo.
     */
    char out[220];
    std::snprintf(out, sizeof(out),
        "{\"ack\":false,\"err\":\"busy\",\"reason\":\"%s\",\"state\":\"diag_running\",\"id\":\"%s\"}",
        reason ? reason : "busy", cid(ctx));
    pub_status(ctx, out);
}

static void pub_status_state(CmdContext* ctx, const Runtime& rt)
/*
 * @brief Publica el estado actual del diagnóstico en el topic de status.
 * @param ctx Contexto que contiene la información necesaria para publicar.
 * @param rt Estado de ejecución del diagnóstico.
 * Funcionalidad:
 * - Construye un mensaje JSON que indica el estado actual del diagnóstico, incluyendo el paso actual, el total de pasos y el client ID.
 * - Publica el mensaje JSON en el topic de status específico del dispositivo.
 */
{
    char out[260];
    if (rt.state == DiagState::IDLE) {
        std::snprintf(out, sizeof(out),
            "{\"status\":\"idle\",\"id\":\"%s\"}", cid(ctx));
    } else {
        std::snprintf(out, sizeof(out),
            "{\"status\":\"diag_running\",\"step\":%d,\"steps\":%d,\"seq\":%d,\"id\":\"%s\"}",
            rt.current_step + 1, rt.cfg.steps, rt.seq, cid(ctx));
    }
    pub_status(ctx, out);
}

static void pub_diag_started(CmdContext* ctx, const DiagConfig& cfg)
/*
 * @brief Publica un mensaje indicando que el diagnóstico ha comenzado, junto con la configuración utilizada.
 * @param ctx Contexto que contiene la información necesaria para publicar.
 * @param cfg Configuración del diagnóstico que se va a ejecutar.
 * Funcionalidad:
 * - Construye un mensaje JSON que indica que el diagnóstico ha comenzado, incluyendo detalles de la configuración y el client ID.
 * - Publica el mensaje JSON en el topic de status específico del dispositivo.
 */
{
    char out[360];
    float v0 = cfg.vlist[0];
    float vN = cfg.vlist[cfg.steps - 1];
    std::snprintf(out, sizeof(out),
        "{\"status\":\"diag_started\",\"type\":\"sweep\",\"steps\":%d,"
        "\"v_first\":%.3f,\"v_last\":%.3f,"
        "\"settle_ms\":%d,\"sample_interval_ms\":%d,\"samples_per_step\":%d,"
        "\"format\":\"raw_ints\",\"amp_unit\":\"mV\",\"cell_unit\":\"uV\","
        "\"id\":\"%s\"}",
        cfg.steps, v0, vN,
        cfg.settle_ms, cfg.sample_interval_ms, cfg.samples_per_step,
        cid(ctx));
    pub_status(ctx, out);
}

static void pub_step_started(CmdContext* ctx, int step_idx, float v_set, uint8_t dac)
/*
 * @brief Publica un mensaje indicando que un paso del diagnóstico ha comenzado.
 * @param ctx Contexto que contiene la información necesaria para publicar.
 * @param step_idx Índice del paso que ha comenzado.
 * @param v_set Voltaje establecido para el paso.
 * @param dac Valor del DAC aplicado.
 * Funcionalidad:
 * - Construye un mensaje JSON que indica que un paso del diagnóstico ha comenzado, incluyendo el índice del paso, el voltaje establecido, el valor del DAC y el client ID.
 * - Publica el mensaje JSON en el topic de status específico del dispositivo.
 */
{
    char out[220];
    std::snprintf(out, sizeof(out),
        "{\"status\":\"step_started\",\"step\":%d,\"v_set\":%.3f,\"dac\":%u,\"id\":\"%s\"}",
        step_idx + 1, v_set, dac, cid(ctx));
    pub_status(ctx, out);
}

static void pub_diag_finished(CmdContext* ctx, const Runtime& rt, const char* reason)
/*
 * @brief Publica un mensaje indicando que el diagnóstico ha finalizado.
 * @param ctx Contexto que contiene la información necesaria para publicar.
 * @param rt Estado de ejecución del diagnóstico.
 * @param reason Razón por la cual el diagnóstico ha finalizado (opcional).
 * Funcionalidad:
 * - Construye un mensaje JSON que indica que el diagnóstico ha finalizado, incluyendo la razón, el total de pasos y el client ID.
 * - Publica el mensaje JSON en el topic de status específico del dispositivo.
 */
{
    char out[240];
    std::snprintf(out, sizeof(out),
        "{\"status\":\"diag_finished\",\"type\":\"sweep\",\"reason\":\"%s\",\"steps\":%d,\"id\":\"%s\"}",
        reason ? reason : "done", rt.cfg.steps, cid(ctx));
    pub_status(ctx, out);
}

// ================== ADS read (una muestra) ==================
static bool read_ads_one(CmdContext* ctx, int& out_load_cell_signal, int& out_load_cell_excitation)
/*
 * @brief Lee una muestra del ADS.
 * @param ctx Contexto que contiene la información necesaria para la lectura.
 * @param out_load_cell_signal Señal de la celda de carga leída.
 * @param out_load_cell_excitation Excitación de la celda de carga leída.
 * @return true si la lectura fue exitosa, false en caso contrario.
 */
{
    if (!ctx || !ctx->ads) return false;

    float output_mv = 0.0f;  // "output_mv"
    float input_volt = 0.0f; // "input_volt"

    esp_err_t st = ctx->ads->read_two_pairs(
        output_mv,
        input_volt,
        ADS1115::PGA::FS_0_256V,   // Celda
        ADS1115::PGA::FS_6_144V,   // Alimentación (divisor)
        ADS1115::SPS::SPS_128
    );
    if (st != ESP_OK) return false;

    // output_mv viene en V? en tu uso anterior lo multiplicas por 1000 para mV,
    // como la salida esta en VOLTS.
    // Mantengo tu convención: load_cell_signal = (output_mv[V] * 1e6)
    float load_cell_signal_f = output_mv * 1000000.0f;

    // input_volt lo estabas multiplicando por 3.7 para obtener Vin real (en V)
    float amp_v_f = input_volt * AMP_DIV_FACTOR;
    float load_cell_excitation_f = amp_v_f * 1000.0f;

    out_load_cell_signal = (int)(load_cell_signal_f >= 0 ? (load_cell_signal_f + 0.5f) : (load_cell_signal_f - 0.5f));
    out_load_cell_excitation  = (int)(load_cell_excitation_f  >= 0 ? (load_cell_excitation_f  + 0.5f) : (load_cell_excitation_f  - 0.5f));

    return true;
}

// ================== DAC apply ==================
static bool apply_voltage(CmdContext* ctx, float v_set, float& out_v_applied, uint8_t& out_dac)
/*
 * @brief Aplica un voltaje utilizando el DAC.
 * @param ctx Contexto que contiene la información necesaria para la operación.
 * @param v_set Voltaje a aplicar.
 * @param out_v_applied Voltaje realmente aplicado (opcional).
 * @param out_dac Valor del DAC aplicado (opcional).
 * @return true si la operación fue exitosa, false en caso contrario.
 */
{
    if (!ctx || !ctx->proc) return false;

    // Usamos el parser existente: mandamos el número como string
    char num[32];
    std::snprintf(num, sizeof(num), "%.4f", v_set);

    float vin = 0.0f;
    uint8_t dac = 0;
    bool ok = ctx->proc->handle_voltage_payload(num, &vin, &dac);// El parser hace validación de rango, así que si ok==true, vin ya está en rango
    if (!ok) return false;

    out_v_applied = vin;
    out_dac = dac;
    return true;
}

// ================== JSON parsing ==================
static cJSON* jget(cJSON* obj, const char* key)
/*
 * @brief Obtiene un item de un objeto JSON de forma segura.
 * @param obj Objeto JSON del cual se desea obtener el item.
 * @param key Clave del item a obtener.
 * @return Puntero al item JSON correspondiente a la clave, o nullptr si no se encuentra o si los parámetros son inválidos.
 */
{
    return (obj && key) ? cJSON_GetObjectItemCaseSensitive(obj, key) : nullptr;
}

static bool get_int(cJSON* obj, const char* key, int& out)
/*
 * @brief Obtiene un valor entero de un objeto JSON de forma segura.
 * @param obj Objeto JSON del cual se desea obtener el valor.
 * @param key Clave del valor a obtener.
 * @param out Referencia donde se almacenará el valor obtenido.
 * @return true si se obtuvo un valor entero válido, false en caso contrario.
 */
{
    cJSON* it = jget(obj, key);
    if (it && cJSON_IsNumber(it)) { out = it->valueint; return true; }
    return false;
}

static bool get_float(cJSON* obj, const char* key, float& out)
/*
 * @brief Obtiene un valor flotante de un objeto JSON de forma segura.
 * @param obj Objeto JSON del cual se desea obtener el valor.
 * @param key Clave del valor a obtener.
 * @param out Referencia donde se almacenará el valor obtenido.
 * @return true si se obtuvo un valor flotante válido, false en caso contrario.
 */
{
    cJSON* it = jget(obj, key);
    if (it && cJSON_IsNumber(it)) { out = (float)it->valuedouble; return true; }
    return false;
}

static bool is_cmd(const char* payload, const char* name)
/*
 * @brief Verifica si un payload JSON contiene un comando específico.
 * @param payload Cadena JSON que contiene el comando.
 * @param name Nombre del comando a verificar.
 * @return true si el comando coincide, false en caso contrario.
 */

{
    if (!payload || !name) return false;
    cJSON* root = cJSON_Parse(payload);
    if (!root) return false;
    cJSON* cmd = jget(root, "cmd");
    bool ok = (cmd && cJSON_IsString(cmd) && cmd->valuestring && std::strcmp(cmd->valuestring, name) == 0);
    cJSON_Delete(root);
    return ok;
}

static bool parse_diag_sweep(const char* payload, DiagConfig& cfg)
/*
 * @brief Parsea un comando diag_sweep desde un payload JSON.
 * @param payload Cadena JSON que contiene el comando.
 * @param cfg Configuración de diagnóstico a llenar.
 * @return true si el parseo fue exitoso, false en caso contrario.
 */
{
    cJSON* root = cJSON_Parse(payload);
    if (!root) return false;

    cJSON* cmd = jget(root, "cmd");
    if (!cmd || !cJSON_IsString(cmd) || !cmd->valuestring || std::strcmp(cmd->valuestring, "diag_sweep") != 0) {
        cJSON_Delete(root);
        return false;
    }

    // defaults
    cfg.settle_ms = 800;            // tiempo de estabilización entre pasos
    cfg.sample_interval_ms = 100;   // intervalo entre muestras dentro de un paso
    cfg.samples_per_step = 20;      // cantidad de muestras a tomar por paso    
    cfg.raw_ints = true;            // formato de datos (siempre true en este contexto)

    get_int(root, "settle_ms", cfg.settle_ms);                          // opcional, hacer un retardo entre pasos para estabilización (ej. 800ms)
    get_int(root, "sample_interval_ms", cfg.sample_interval_ms);        // opcional, intervalo entre muestras dentro de un paso (ej. 100ms)
    get_int(root, "samples_per_step", cfg.samples_per_step);            // opcional, cantidad de muestras a tomar por paso (ej. 20)
    // El comando puede venir con una lista de voltajes específicos o con un plan (start

    // Validar límites
    if (cfg.settle_ms < 0) cfg.settle_ms = 0; 

    if (cfg.sample_interval_ms < MIN_SAMPLE_INTERVAL_MS) cfg.sample_interval_ms = MIN_SAMPLE_INTERVAL_MS;
    if (cfg.sample_interval_ms > MAX_SAMPLE_INTERVAL_MS) cfg.sample_interval_ms = MAX_SAMPLE_INTERVAL_MS;

    if (cfg.samples_per_step < MIN_SAMPLES_PER_STEP) cfg.samples_per_step = MIN_SAMPLES_PER_STEP;
    if (cfg.samples_per_step > MAX_SAMPLES_PER_STEP) cfg.samples_per_step = MAX_SAMPLES_PER_STEP;

    cfg.steps = 0;

    // Opción 1: voltages: [ ... ]
    cJSON* voltages = jget(root, "voltages");
    if (voltages && cJSON_IsArray(voltages)) {
        int n = cJSON_GetArraySize(voltages);
        if (n <= 0 || n > MAX_STEPS) { cJSON_Delete(root); return false; }

        for (int i = 0; i < n; i++) {
            cJSON* v = cJSON_GetArrayItem(voltages, i);
            if (!v || !cJSON_IsNumber(v)) { cJSON_Delete(root); return false; }
            float fv = (float)v->valuedouble;
            if (fv < MIN_VIN_CMD || fv > MAX_VIN_CMD) { cJSON_Delete(root); return false; }
            cfg.vlist[cfg.steps++] = fv;
        }

        cJSON_Delete(root);
        return true;
    }

    // Opción 2: plan: {start_v, stop_v, step_v}
    cJSON* plan = jget(root, "plan");
    if (!plan || !cJSON_IsObject(plan)) { cJSON_Delete(root); return false; }

    float start_v = 0, stop_v = 0, step_v = 0;
    if (!get_float(plan, "start_v", start_v) ||
        !get_float(plan, "stop_v",  stop_v)  ||
        !get_float(plan, "step_v",  step_v)) {
        cJSON_Delete(root);
        return false;
    }

    if (step_v <= 0.0f) { cJSON_Delete(root); return false; }
    if (start_v < MIN_VIN_CMD || start_v > MAX_VIN_CMD) { cJSON_Delete(root); return false; }
    if (stop_v  < MIN_VIN_CMD || stop_v  > MAX_VIN_CMD) { cJSON_Delete(root); return false; }

    if (stop_v >= start_v) {
        for (float v = start_v; v <= stop_v + 1e-6f; v += step_v) {
            if (cfg.steps >= MAX_STEPS) break;
            cfg.vlist[cfg.steps++] = v;
        }
    } else {
        for (float v = start_v; v >= stop_v - 1e-6f; v -= step_v) {
            if (cfg.steps >= MAX_STEPS) break;
            cfg.vlist[cfg.steps++] = v;
        }
    }

    cJSON_Delete(root);
    return (cfg.steps > 0);
}

// ================== Cola: “pumpeo” durante running ==================
static void pump_control_commands(CmdContext* ctx, Runtime& rt)
/*
 * @brief Controla los comandos durante la ejecución.
 *        Permite responder a comandos de stop y status mientras se está ejecutando un plan.
 * @param ctx Contexto de comandos.
 * @param rt Estado de ejecución.
 */
{
    
    CmdMsg msg{};
    while (g_cmd_queue && xQueueReceive(g_cmd_queue, &msg, 0) == pdTRUE) {
        if (is_cmd(msg.payload, "stop")) {
            rt.stop_requested = true;
            char out[160];
            std::snprintf(out, sizeof(out),
                "{\"status\":\"stop_requested\",\"id\":\"%s\"}", cid(ctx));
            pub_status(ctx, out);
            continue;
        }

        if (is_cmd(msg.payload, "status")) {
            pub_status_state(ctx, rt);
            continue;
        }

        // Cualquier otro comando durante running -> busy
        pub_busy(ctx, "diag_running");
    }
}

static void delay_with_pump(CmdContext* ctx, Runtime& rt, int total_ms)
/*
 * @brief Retardo con control de bombeo.
 *        Sirve para mantener la responsividad a comandos (stop/status) durante los retardos de estabilización y muestreo.
 * @param ctx Contexto de comandos.
 * @param rt Estado de ejecución.
 * @param total_ms Tiempo total de retardo en ms.
 */
{
    const int chunk = 20;
    int remaining = total_ms;
    while (remaining > 0 && !rt.stop_requested) {
        int d = (remaining > chunk) ? chunk : remaining;
        vTaskDelay(pdMS_TO_TICKS(d));
        remaining -= d;
        pump_control_commands(ctx, rt);
    }
}

// ================== Publicar RAW por chunks ==================

static void pub_step_raw_chunks(
    CmdContext* ctx,
    int seq,
    int step_idx,
    float v_set,
    uint8_t dac,
    int sample_interval_ms,
    uint32_t t0_ms,
    const int* load_cell_excitation,
    const int* load_cell_signal,
    int n_total
)/*
 * @brief Publica los datos RAW de un paso en chunks.
 * @param ctx Contexto de comandos.
 * @param seq Número de secuencia.
 * @param step_idx Índice del paso.
 * @param v_set Voltaje configurado.
 * @param dac Valor del DAC.
 * @param sample_interval_ms Intervalo de muestreo en ms.
 * @param t0_ms Tiempo inicial en ms.
 * @param load_cell_excitation Señal de excitación de la celda de carga.
 * @param load_cell_signal Señal de la celda de carga.
 * @param n_total Número total de muestras.
 */

{  const int chunks = (n_total + SAMPLES_PER_CHUNK - 1) / SAMPLES_PER_CHUNK;

    for (int c = 0; c < chunks; c++) {   // iterar por chunks
        int start = c * SAMPLES_PER_CHUNK;
        int end = start + SAMPLES_PER_CHUNK;
        if (end > n_total) end = n_total;
        int n = end - start;

        char out[JSON_BUF_SZ];
        int w = 0;

        w += std::snprintf(out + w, sizeof(out) - w,    // ojo con el tamaño del buffer y el w para no desbordar
            "{\"status\":\"step_data\",\"seq\":%d,\"step\":%d,"
            "\"v_set\":%.3f,\"dac\":%u,"
            "\"sample_interval_ms\":%d,"
            "\"t0_ms\":%u,"
            "\"chunk\":%d,\"chunks\":%d,\"n\":%d,"
            "\"cell_exc[mV]\":[",
            seq, step_idx + 1, v_set, dac,
            sample_interval_ms,
            (unsigned)t0_ms,
            (c+1), chunks, n);

        for (int i = start; i < end; i++) { // iterar por muestras dentro del chunk
            w += std::snprintf(out + w, sizeof(out) - w,
                "%d%s", load_cell_excitation[i], (i + 1 < end) ? "," : "");
        }

        w += std::snprintf(out + w, sizeof(out) - w, "],\"cell_sig[uV]\":["); // ahora la señal de la celda

        for (int i = start; i < end; i++) { // iterar por muestras dentro del chunk
            w += std::snprintf(out + w, sizeof(out) - w,
                "%d%s", load_cell_signal[i], (i + 1 < end) ? "," : "");
        }

        w += std::snprintf(out + w, sizeof(out) - w, // cerrar arrays y objeto JSON
            "],\"id\":\"%s\"}", cid(ctx));

        pub_data(ctx, out); // publicar el chunk
    }
}
// ================== Task principal ==================
static void cmd_task(void* arg)
/*
 * @brief Tarea principal de comandos.
 * @param arg Contexto de comandos.
 */
{
    auto* ctx = static_cast<CmdContext*>(arg);
    Runtime rt{};

    if (!ctx || !ctx->mqtt || !ctx->proc || !ctx->ads || !ctx->topic_status || !ctx->topic_data) { // validación básica de contexto
        ESP_LOGE(TAG, "CmdContext incompleto");
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
        /*
            * Diseño:
            - En IDLE: bloquea esperando comandos. Solo acepta "diag_sweep", "status" o "stop" (aunque stop no haría nada). Cualquier otro comando -> parse_err.
            - Al recibir "diag_sweep" con payload válido -> parsea configuración, responde con "diag_started" y arranca el diagnóstico (pasa a RUNNING).
            - En RUNNING: ejecuta el diagnóstico. Durante la ejecución, "status" respondendo el estado actual, "stop" marca una bandera para detener el diagnóstico de forma segura al finalizar el paso actual, y cualquier otro comando responde con "busy".      
            
         */
        CmdMsg msg{};

        if (rt.state == DiagState::IDLE) {
            if (xQueueReceive(g_cmd_queue, &msg, portMAX_DELAY) != pdTRUE) continue;

            // status / stop
            if (is_cmd(msg.payload, "status")) { pub_status_state(ctx, rt); continue; }
            if (is_cmd(msg.payload, "stop")) {
                char out[140];
                std::snprintf(out, sizeof(out),
                    "{\"status\":\"idle\",\"note\":\"already_stopped\",\"id\":\"%s\"}", cid(ctx));
                pub_status(ctx, out);
                continue;
            }

            // diag_sweep
            DiagConfig cfg{};
            if (!parse_diag_sweep(msg.payload, cfg)) {
                // Si era JSON pero no cmd válido -> parse_err
                cJSON* tmp = cJSON_Parse(msg.payload);
                if (tmp) { cJSON_Delete(tmp); pub_parse_err(ctx, "unknown_cmd_or_bad_fields"); }
                else { pub_parse_err(ctx, "invalid_json"); }
                continue;
            }

            rt.cfg = cfg;
            rt.stop_requested = false;
            rt.seq = 0;
            rt.current_step = -1;
            rt.state = DiagState::RUNNING;

            pub_diag_started(ctx, rt.cfg);

            // Ejecutar sweep
            for (int si = 0; si < rt.cfg.steps; si++) {
                rt.current_step = si;

                pump_control_commands(ctx, rt);
                if (rt.stop_requested) {
                    pub_diag_finished(ctx, rt, "stop_cmd");
                    break;
                }

                float v_target = rt.cfg.vlist[si];
                float v_applied = 0.0f;
                uint8_t dac = 0;

                if (!apply_voltage(ctx, v_target, v_applied, dac)) {
                    pub_diag_finished(ctx, rt, "set_v_failed");
                    break;
                }

                pub_step_started(ctx, si, v_applied, dac);

                // settle
                if (rt.cfg.settle_ms > 0) {
                    delay_with_pump(ctx, rt, rt.cfg.settle_ms);
                    if (rt.stop_requested) {
                        pub_diag_finished(ctx, rt, "stop_cmd");
                        break;
                    }
                }

                // muestreo RAW
                static int load_cell_excitation[MAX_SAMPLES_PER_STEP];
                static int load_cell_signal[MAX_SAMPLES_PER_STEP];
                int n_ok = 0;

                uint32_t t0_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                for (int k = 0; k < rt.cfg.samples_per_step; k++) {
                    pump_control_commands(ctx, rt);
                    if (rt.stop_requested) break;

                    int cell = 0, amp = 0;
                    if (read_ads_one(ctx, cell, amp)) {
                        load_cell_excitation[n_ok] = amp;
                        load_cell_signal[n_ok] = cell;
                        n_ok++;
                    }

                    if (rt.cfg.sample_interval_ms > 0) {
                        delay_with_pump(ctx, rt, rt.cfg.sample_interval_ms);
                    }
                }

                if (rt.stop_requested) {
                    pub_diag_finished(ctx, rt, "stop_cmd");
                    break;
                }

                rt.seq++;
                pub_step_raw_chunks(
                    ctx, rt.seq, si,
                    v_applied, dac,
                    rt.cfg.sample_interval_ms,
                    t0_ms,
                    load_cell_excitation, load_cell_signal, n_ok
                );
            }

            // Si terminó sin stop, cerramos
            if (!rt.stop_requested) {
                pub_diag_finished(ctx, rt, "done");
            }

            rt.state = DiagState::IDLE;
            rt.current_step = -1;
            rt.stop_requested = false;
            continue;
        }

        // Running: por diseño no debería bloquear aquí, pero mantenemos un pequeño pump
        pump_control_commands(ctx, rt);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void start_cmd_task(CmdContext* ctx)
{
    // stack suficiente para cJSON y snprintf
    xTaskCreatePinnedToCore(cmd_task, "cmd_task", 6144, ctx, 10, nullptr, 1);
}