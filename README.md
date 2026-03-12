# Pruebas_MQTT

Pruebas de diagnóstico por MQTT sobre ESP32 (FreeRTOS), con control de voltaje y muestreo ADC (ADS1115).

## Qué hace este proyecto

La tarea `cmd_task`:

1. Escucha comandos JSON desde una cola (`g_cmd_queue`).
2. Ejecuta un barrido de voltajes (`diag_sweep`).
3. Mide señales con ADS1115 por cada paso.
4. Publica estado y datos por MQTT en:
   - `topic_status` (estados/errores)
   - `topic_data` (muestras por chunk)

## Archivo principal de lógica

- `src/CmdTask.cpp`

## Máquina de estados

- **IDLE**
  - Acepta: `diag_sweep`, `status`, `stop`.
- **RUNNING**
  - Acepta: `status`, `stop`.
  - Otros comandos responden `busy`.

## Límites y configuración

- Rango de voltaje permitido: **1.0V a 12.0V**
- Máximo de pasos: **64**
- `sample_interval_ms`: **5..5000**
- `samples_per_step`: **1..500**
- `settle_ms`: mínimo **0**
- Chunk de publicación: **20 muestras** (`SAMPLES_PER_CHUNK`)

## Comando principal: `diag_sweep`

Se puede enviar de dos formas:

1. Lista explícita:
   - `voltages: [1.0, 2.0, 3.0]`
2. Plan:
   - `plan: { start_v, stop_v, step_v }`

Parámetros opcionales (con defaults):
- `settle_ms` (default `800`)
- `sample_interval_ms` (default `100`)
- `samples_per_step` (default `20`)

## Publicaciones MQTT

### topic_status

- `diag_started`
- `step_started`
- `diag_finished` (`done`, `stop_cmd`, `set_v_failed`)
- `idle`, `diag_running`
- errores:
  - `{"ack":false,"err":"parse_err",...}`
  - `{"ack":false,"err":"busy",...}`

### topic_data

- `step_data` con:
  - `cell_exc[mV]` (excitación)
  - `cell_sig[uV]` (señal celda)
  - metadatos de chunk (`chunk`, `chunks`, `n`, `seq`, `step`)

## Conversión de medidas (actual)

- `cell_sig[uV] = output_mv * 1e6`
- `cell_exc[mV] = input_volt * 3.7 * 1000`

> `3.7` corresponde al factor del divisor (`AMP_DIV_FACTOR`).

## Ejecución (Windows, VS Code + PlatformIO)

```bash
pio run
pio run -t upload
pio device monitor
```

## Notas técnicas

- Tarea creada con `xTaskCreatePinnedToCore(..., stack=6144, prio=10, core=1)`.
- Durante esperas (`settle` y `sample_interval`) se bombean comandos para mantener respuesta a `stop/status`.
