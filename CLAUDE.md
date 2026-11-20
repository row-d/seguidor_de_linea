# CLAUDE.md

Firmware para un robot seguidor de linea (Arduino Mega + FreeRTOS). El codigo, los comentarios, los mensajes de commit y la documentacion se escriben en **espanol**, sin acentos en el codigo (igual que el resto del repositorio).

Detalles completos de hardware, pines, calibraciones, comandos y flujo: [README.md](README.md). Reglas de la competencia: [COMPETENCIA.md](COMPETENCIA.md). Lee ambos antes de cambiar comportamiento del robot.

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

- `seguidor_de_linea.ino`: ensambla los componentes, define pines, umbrales y compensacion de las ruedas, y elige el modo con `CALIBRATE`, `CALIBRATE_WHEELS` y `COMMANDS`. El robot empieza el recorrido al encender. Con `COMMANDS` en 1, calibracion (`c`), registro (`r`/`s`), ganancias del PID (`p`), velocidades (`b`), detener el recorrido (`x`) y empezar uno nuevo (`g`) se piden con `scripts/console.ps1` por el puerto que elige `COMMANDS_OVER_USB`; los cambios no se guardan en EEPROM.
- `bluetooth_config.h`: nombre/baudios del HC-05 y `COMMANDS_OVER_USB` (`true`: comandos y mensajes por el USB, `Serial`; `false`: por el Bluetooth, `Serial1`). El sketch ya no usa `BLUETOOTH_SETUP_MODE` ni llama a `Hc05::configure()`.
- `firmware/src/`: biblioteca `RobotComponents` (solo cabeceras). Es la **fuente de verdad**; no edites la copia del sketchbook.
  - `components/`: un componente de hardware por archivo, todos derivan de `Component` (`component.h`).
  - `behaviors/`: comportamientos del robot, cada uno en un namespace con su estado, sus constantes y un struct `Components` (referencias a los componentes) que el sketch pasa como parametro a `xTaskCreate`. `line_follower.h` (seguimiento de linea, meta, bifurcaciones y linea perdida), `collision.h` (garra ante un objeto), `calibration.h` (sensores de linea y ruedas).
  - `commands/`: `commands.h` lee los comandos del PC por el puerto del `Logger` (`commands::poll(logger)` devuelve los `REQUEST_*`); `command_task.h` es la tarea que los aplica. Solo se crea con `#define COMMANDS 1` en el sketch, y debe tener menos prioridad que `line_follower::PRIORITY`.
  - Si anades un archivo, incluyelo en `RobotComponents.h`.
- `scripts/`: enlace de la biblioteca y consola de comandos al robot por serie (`console.ps1`). Las letras de los comandos deben coincidir con las constantes de `commands/commands.h`. `logs/`: CSV capturados.

## Reglas al modificar el codigo

- **Pines**: no los cambies ni reutilices uno ocupado sin confirmar con el usuario; deben coincidir con el cableado real y con la tabla del README.
- **Memoria**: la Mega tiene 8 KB de RAM compartidos con las pilas de FreeRTOS. Evita `String`, asignacion dinamica y arreglos grandes; revisa el resumen de memoria que muestra la compilacion.
- **FreeRTOS**: `loop()` queda vacio a proposito; la logica corre en tareas creadas con `xTaskCreate`. Usa `vTaskDelay` en vez de `delay()` dentro de tareas y vigila el tamano de pila al anadir variables locales.
- **PID**: `PID_v1` divide `Kd` entre el periodo, por eso `Kd` es mucho menor que `Kp`. No "corrijas" esas ganancias sin pruebas en el robot. Las ganancias en uso son `KP`, `KI` y `KD`, y las velocidades `speeds`, al principio de `behaviors/line_follower.h`; los comandos `p` y `b` solo las cambian hasta que se apaga el robot. El seguidor corre con periodo fijo (`PERIOD_MS`) para que el dt del PID no cambie.
- **Calibracion sin EEPROM**: no se usa la EEPROM. Las calibraciones imprimen los valores y se copian a mano en el sketch (`thresholds`, `MIN_PWM_L/R`, `WHEEL_TRIM_L/R`). Si cambias el cableado de sensores o motores, esos valores dejan de valer: hay que volver a calibrar en el robot.
- **Motores**: velocidades limitadas a `-255..255`. El seguidor mueve las ruedas con `Movement::drive_compensated()`, que aplica la compensacion de cada rueda; `drive()` es el PWM sin compensar y solo lo usan las calibraciones.
- **Mensajes por serie**: si cambias los mensajes (`meta`, `collision`, `run,...`, `pid,...`, `speeds,...`, `thresholds,...`, `calibration,...`, `recording,...`) o las letras de los comandos, actualiza el README y `scripts/console.ps1`. Hoy no se envian filas de telemetria (`t,...`) ni eventos (`Logger::telemetry()` y `Logger::event()` no se llaman).
- **Competencia**: el codigo aun no cumple todas las reglas de `COMPETENCIA.md` (beep al derribar un objeto, LEDs apagados al ir recto, frenar al retirar el objeto); ver "Ideas pendientes" en el README.
- `ColorSensor` no se usa y no hay sensor de color conectado.
- Cuando cambies cableado, parametros o dependencias, actualiza el README en el mismo cambio.
