# CLAUDE.md

Firmware para un robot seguidor de linea (Arduino Mega + FreeRTOS). El codigo, los comentarios, los mensajes de commit y la documentacion se escriben en **espanol**, sin acentos en el codigo (igual que el resto del repositorio).

Detalles completos de hardware, pines, telemetria y flujo: [README.md](README.md). Reglas de la competencia: [COMPETENCIA.md](COMPETENCIA.md). Lee ambos antes de cambiar comportamiento del robot.

## Compilar

```bash
arduino-cli compile --fqbn arduino:avr:mega --library firmware .
```

- Placa: Arduino Mega (`arduino:avr:mega`). No uses otra FQBN.
- `--library firmware` compila la copia local de `RobotComponents`; no hace falta el enlace simbolico de `scripts/link-library.*`.
- Un hook (`.claude/hooks/compile.ps1`) compila automaticamente tras cada edicion de `.ino/.h/.cpp`. Si falla, corrige el error antes de seguir.
- No hay pruebas automaticas: compilar es la unica verificacion posible aqui. El resto se valida en el robot real.

## Cargar al robot

- **Nunca** ejecutes `arduino-cli upload` sin que el usuario lo pida: el robot puede moverse sobre la mesa. Pregunta tambien el puerto COM (hay varios: el USB de la Mega y el Bluetooth).

## Estructura

- `seguidor_de_linea.ino`: ensambla los componentes, define pines, `calibration_mode` y `preferred_route`.
- `bluetooth_config.h`: nombre/baudios del HC-05. `BLUETOOTH_SETUP_MODE` debe quedar en `false` para que el robot funcione.
- `firmware/src/`: biblioteca `RobotComponents` (solo cabeceras). Es la **fuente de verdad**; no edites la copia del sketchbook.
  - `components/`: un componente de hardware por archivo, todos derivan de `Component` (`component.h`).
  - `tasks/`: logica del robot. `robot.h` (controlador y velocidades), `start.h` (seguimiento PID, meta, bifurcaciones), `route_memory.h`, `collision.h`, `calibration.h`, `debug.h`.
  - Si anades un archivo, incluyelo en `RobotComponents.h`.
- `scripts/`: enlace de la biblioteca y captura de datos por serie. `logs/`: CSV capturados.

## Reglas al modificar el codigo

- **Pines**: no los cambies ni reutilices uno ocupado sin confirmar con el usuario; deben coincidir con el cableado real y con la tabla del README.
- **Memoria**: la Mega tiene 8 KB de RAM compartidos con las pilas de FreeRTOS. Evita `String`, asignacion dinamica y arreglos grandes; revisa el resumen de memoria que muestra la compilacion.
- **FreeRTOS**: `loop()` queda vacio a proposito; la logica corre en tareas creadas con `xTaskCreate`. Usa `vTaskDelay` en vez de `delay()` dentro de tareas y vigila el tamano de pila al anadir variables locales.
- **Motores**: velocidades limitadas a `-255..255`.
- **PID**: `PID_v1` divide `Kd` entre el periodo, por eso `Kd` es mucho menor que `Kp`. No "corrijas" esas ganancias sin datos de telemetria.
- **Telemetria/eventos**: si cambias el formato de las filas (`t,...`) o los nombres de eventos, actualiza el README y los scripts que los leen.
- `read_rgb_sensor()` es un stub que devuelve `{0, 0, 0}`; no asumas que el sensor de color funciona.
- Cuando cambies cableado, parametros o dependencias, actualiza el README en el mismo cambio.
