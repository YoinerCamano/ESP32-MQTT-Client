# MQTT API (CmdTask)

## Formato de comando

Todos los comandos son JSON y usan el campo `cmd`.

---

## 1) Estado

### Request
```json
{"cmd":"status"}
```

### Response (`topic_status`)
- En reposo:
```json
{"status":"idle","id":"ESP"}
```
- En ejecución:
```json
{"status":"diag_running","step":2,"steps":5,"seq":1,"id":"ESP"}
```

---

## 2) Stop

### Request
```json
{"cmd":"stop"}
```

### Response (`topic_status`)
- Si está en IDLE:
```json
{"status":"idle","note":"already_stopped","id":"ESP"}
```
- Si está RUNNING:
```json
{"status":"stop_requested","id":"ESP"}
```
Luego termina con:
```json
{"status":"diag_finished","type":"sweep","reason":"stop_cmd","steps":5,"id":"ESP"}
```

---

## 3) Diagnóstico por barrido

### Request (modo lista)
```json
{
  "cmd":"diag_sweep",
  "voltages":[1.0, 2.0, 3.0],
  "settle_ms":800,
  "sample_interval_ms":100,
  "samples_per_step":20
}
```

### Request (modo plan)
```json
{
  "cmd":"diag_sweep",
  "plan":{"start_v":1.0,"stop_v":5.0,"step_v":0.5},
  "settle_ms":800,
  "sample_interval_ms":100,
  "samples_per_step":20
}
```

### Respuestas (`topic_status`)
Inicio:
```json
{"status":"diag_started","type":"sweep","steps":9,"v_first":1.000,"v_last":5.000,"settle_ms":800,"sample_interval_ms":100,"samples_per_step":20,"format":"raw_ints","amp_unit":"mV","cell_unit":"uV","id":"ESP"}
```

Por paso:
```json
{"status":"step_started","step":1,"v_set":1.000,"dac":42,"id":"ESP"}
```

Fin:
```json
{"status":"diag_finished","type":"sweep","reason":"done","steps":9,"id":"ESP"}
```

### Datos (`topic_data`)
```json
{
  "status":"step_data",
  "seq":1,
  "step":1,
  "v_set":1.000,
  "dac":42,
  "sample_interval_ms":100,
  "t0_ms":123456,
  "chunk":1,
  "chunks":2,
  "n":20,
  "cell_exc[mV]":[4998,5001,...],
  "cell_sig[uV]":[12,10,...],
  "id":"ESP"
}
```

---

## Errores

Parseo/comando inválido:
```json
{"ack":false,"err":"parse_err","reason":"unknown_cmd_or_bad_fields","id":"ESP"}
```

Comando no permitido durante ejecución:
```json
{"ack":false,"err":"busy","reason":"diag_running","state":"diag_running","id":"ESP"}
```