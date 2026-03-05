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
static constexpr float AMP_DIV_FACTOR = 3.7f;   // tu factor del divisor (para convertir lectura "input_volt" a V real)
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
};

struct Runtime {
    DiagState state = DiagState::IDLE;
    bool stop_requested = false;

    int seq = 0;
    int current_step = -1;
    DiagConfig cfg{};
};

// ================== Publish helpers ==================
static inline const char* cid(CmdContext* ctx) { return (ctx && ctx->client_id) ? ctx->client_id : "ESP"; }

static void pub_status(CmdContext* ctx, const char* json)
{
    if (ctx && ctx->mqtt && ctx->topic_status) ctx->mqtt->publish(ctx->topic_status, json);
}

static void pub_data(CmdContext* ctx, const char* json) {
    if (ctx && ctx->mqtt && ctx->topic_data) ctx->mqtt->publish(ctx->topic_data, json);
}

static void pub_parse_err(CmdContext* ctx, const char* reason){
    char out[256];
    std::snprintf(out, sizeof(out),
        "{\"ack\":false,\"err\":\"parse_err\",\"reason\":\"%s\",\"id\":\"%s\"}",
        reason ? reason : "invalid_json", cid(ctx));
    pub_status(ctx, out);
}

static void pub_busy(CmdContext* ctx, const char* reason){
    char out[220];
    std::snprintf(out, sizeof(out),
        "{\"ack\":false,\"err\":\"busy\",\"reason\":\"%s\",\"state\":\"diag_running\",\"id\":\"%s\"}",
        reason ? reason : "busy", cid(ctx));
    pub_status(ctx, out);
}

static void pub_status_state(CmdContext* ctx, const Runtime& rt)
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
{
    char out[220];
    std::snprintf(out, sizeof(out),
        "{\"status\":\"step_started\",\"step\":%d,\"v_set\":%.3f,\"dac\":%u,\"id\":\"%s\"}",
        step_idx + 1, v_set, dac, cid(ctx));
    pub_status(ctx, out);
}

static void pub_diag_finished(CmdContext* ctx, const Runtime& rt, const char* reason)
{
    char out[240];
    std::snprintf(out, sizeof(out),
        "{\"status\":\"diag_finished\",\"type\":\"sweep\",\"reason\":\"%s\",\"steps\":%d,\"id\":\"%s\"}",
        reason ? reason : "done", rt.cfg.steps, cid(ctx));
    pub_status(ctx, out);
}

// ================== ADS read (una muestra) ==================
static bool read_ads_one(CmdContext* ctx, int& out_cell_uV, int& out_amp_mV)
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
    // Mantengo tu convención: cell_uv = (output_mv[V] * 1e6)
    float cell_uV_f = output_mv * 1000000.0f;

    // input_volt lo estabas multiplicando por 3.7 para obtener Vin real (en V)
    float amp_v_f = input_volt * AMP_DIV_FACTOR;
    float amp_mV_f = amp_v_f * 1000.0f;

    out_cell_uV = (int)(cell_uV_f >= 0 ? (cell_uV_f + 0.5f) : (cell_uV_f - 0.5f));
    out_amp_mV  = (int)(amp_mV_f  >= 0 ? (amp_mV_f  + 0.5f) : (amp_mV_f  - 0.5f));

    return true;
}

// ================== DAC apply ==================
static bool apply_voltage(CmdContext* ctx, float v_set, float& out_v_applied, uint8_t& out_dac)
{
    if (!ctx || !ctx->proc) return false;

    // Usamos el parser existente: mandamos el número como string
    char num[32];
    std::snprintf(num, sizeof(num), "%.4f", v_set);

    float vin = 0.0f;
    uint8_t dac = 0;
    bool ok = ctx->proc->handle_voltage_payload(num, &vin, &dac);
    if (!ok) return false;

    out_v_applied = vin;
    out_dac = dac;
    return true;
}

// ================== JSON parsing ==================
static cJSON* jget(cJSON* obj, const char* key)
{
    return (obj && key) ? cJSON_GetObjectItemCaseSensitive(obj, key) : nullptr;
}

static bool get_int(cJSON* obj, const char* key, int& out)
{
    cJSON* it = jget(obj, key);
    if (it && cJSON_IsNumber(it)) { out = it->valueint; return true; }
    return false;
}

static bool get_float(cJSON* obj, const char* key, float& out)
{
    cJSON* it = jget(obj, key);
    if (it && cJSON_IsNumber(it)) { out = (float)it->valuedouble; return true; }
    return false;
}

static bool is_cmd(const char* payload, const char* name)
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
{
    cJSON* root = cJSON_Parse(payload);
    if (!root) return false;

    cJSON* cmd = jget(root, "cmd");
    if (!cmd || !cJSON_IsString(cmd) || !cmd->valuestring || std::strcmp(cmd->valuestring, "diag_sweep") != 0) {
        cJSON_Delete(root);
        return false;
    }

    // defaults
    cfg.settle_ms = 800;
    cfg.sample_interval_ms = 100;
    cfg.samples_per_step = 20;
    cfg.raw_ints = true;

    get_int(root, "settle_ms", cfg.settle_ms);
    get_int(root, "sample_interval_ms", cfg.sample_interval_ms);
    get_int(root, "samples_per_step", cfg.samples_per_step);

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
    const int* amp_mV,
    const int* cell_uV,
    int n_total
)
{
    const int chunks = (n_total + SAMPLES_PER_CHUNK - 1) / SAMPLES_PER_CHUNK;

    for (int c = 0; c < chunks; c++) {
        int start = c * SAMPLES_PER_CHUNK;
        int end = start + SAMPLES_PER_CHUNK;
        if (end > n_total) end = n_total;
        int n = end - start;

        char out[JSON_BUF_SZ];
        int w = 0;

        w += std::snprintf(out + w, sizeof(out) - w,
            "{\"status\":\"step_data\",\"seq\":%d,\"step\":%d,"
            "\"v_set\":%.3f,\"dac\":%u,"
            "\"sample_interval_ms\":%d,"
            "\"t0_ms\":%u,"
            "\"chunk\":%d,\"chunks\":%d,\"n\":%d,"
            "\"amp_mv\":[",
            seq, step_idx + 1, v_set, dac,
            sample_interval_ms,
            (unsigned)t0_ms,
            c, chunks, n);

        for (int i = start; i < end; i++) {
            w += std::snprintf(out + w, sizeof(out) - w,
                "%d%s", amp_mV[i], (i + 1 < end) ? "," : "");
        }

        w += std::snprintf(out + w, sizeof(out) - w, "],\"cell_uv\":[");

        for (int i = start; i < end; i++) {
            w += std::snprintf(out + w, sizeof(out) - w,
                "%d%s", cell_uV[i], (i + 1 < end) ? "," : "");
        }

        w += std::snprintf(out + w, sizeof(out) - w,
            "],\"id\":\"%s\"}", cid(ctx));

        pub_data(ctx, out);
    }
}

// ================== Task principal ==================
static void cmd_task(void* arg)
{
    auto* ctx = static_cast<CmdContext*>(arg);
    Runtime rt{};

    if (!ctx || !ctx->mqtt || !ctx->proc || !ctx->ads || !ctx->topic_status || !ctx->topic_data) {
        ESP_LOGE(TAG, "CmdContext incompleto");
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
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
                static int amp_mV[MAX_SAMPLES_PER_STEP];
                static int cell_uV[MAX_SAMPLES_PER_STEP];
                int n_ok = 0;

                uint32_t t0_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                for (int k = 0; k < rt.cfg.samples_per_step; k++) {
                    pump_control_commands(ctx, rt);
                    if (rt.stop_requested) break;

                    int cell = 0, amp = 0;
                    if (read_ads_one(ctx, cell, amp)) {
                        amp_mV[n_ok] = amp;
                        cell_uV[n_ok] = cell;
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
                    amp_mV, cell_uV, n_ok
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