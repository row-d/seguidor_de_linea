# Seguidor de linea

Software para un robot movil que sigue una linea, resuelve bifurcaciones, se detiene en la meta y acciona una garra al detectar un objeto.

El sketch principal ensambla los componentes del robot. La implementacion reutilizable esta en la biblioteca local `RobotComponents`, dentro de `firmware/`.

## Estado actual

- El robot empieza el recorrido al encender y se detiene sobre la linea de meta con dos beeps.
- El seguimiento de linea usa un PID (`PID_v1`) sobre 3 sensores infrarrojos. En las bifurcaciones prefiere la derecha, despues recto y por ultimo la izquierda.
- Una tarea aparte mide la distancia con el ultrasonico y, con un objeto a 7 cm o menos en 3 lecturas seguidas, abre y cierra la garra. El robot no se detiene y no emite un beep al derribar el objeto.
- Los LEDs indican hacia donde gira el robot; yendo recto se encienden los dos.
- Los umbrales de los sensores y la compensacion de las ruedas se obtienen con calibraciones que imprimen los valores, y se copian en `seguidor_de_linea.ino`. No se usa la EEPROM.
- Los comandos desde el PC (consola `scripts/console.ps1`) estan desactivados por defecto (`COMMANDS` en 0).
- `read_rgb_sensor()` no existe en el sketch actual y `ColorSensor` no se usa.
- No hay una suite automatizada de pruebas; la validacion principal se hace compilando y probando el robot en hardware.

## Estructura

```text
seguidor_de_linea.ino       Sketch principal: pines, valores de calibracion, modos y tareas
bluetooth_config.h          Nombre y velocidad del modulo HC-05, y puerto de los comandos
firmware/
  library.properties        Metadatos y dependencias de la biblioteca
  src/
    RobotComponents.h       Cabecera publica de la biblioteca
    components/             Componentes reutilizables del robot
      movement.h            Motores, direccion y compensacion de cada rueda
      line_sensors.h        Sensores de linea, umbrales y calculo del error
      color_sensor.h        Lectura y comparacion de color (sin uso)
      claw.h                Garra controlada por servo, en pasos
      buzzer.h              Zumbador
      led_array.h           LEDs de giro
      ultrasonic_sensor.h   Medicion de distancia
      wheel_sensor.h        Encoders DFRobot SEN0038 de las ruedas (flancos, rpm, distancia)
      logger.h              Mensajes y registro por Serial o Bluetooth
      hc05.h                Configuracion del HC-05 con comandos AT
      component.h           Interfaz comun de componentes
    behaviors/              Comportamientos del robot (cada uno con su tarea de FreeRTOS)
      line_follower.h       Seguimiento de linea: PID, salida, cruces, meta, bifurcaciones y linea perdida
      collision.h           Garra al detectar un objeto con el ultrasonico
      calibration.h         Calibracion de los sensores de linea y de las ruedas
    commands/
      commands.h            Lectura de los comandos del PC (letras de scripts/console.ps1)
      command_task.h        Tarea que aplica los comandos (con COMMANDS en 1)
scripts/
  console.ps1               Consola: muestra lo que envia el robot y le manda comandos
  link-library.ps1          Enlace simbolico para Windows
  link-library.sh           Enlace simbolico para Linux/macOS
logs/                       Registros CSV guardados por la consola
```

## Requisitos

- Arduino IDE o Arduino CLI.
- Arduino Mega, seleccionada en el IDE.
- Bibliotecas Arduino:
  - `Servo`
  - `Arduino_FreeRTOS`
  - `PID` (`PID_v1`)
- Git, para colaborar con el proyecto.
- En Windows, permisos para crear enlaces simbolicos si se usa el script de enlace. Si el script falla, ejecuta VS Code o PowerShell con los permisos necesarios o crea el enlace desde una terminal con permisos adecuados.

El esquema usa una Arduino Mega. No cambies los pines sin comprobar primero el cableado real.

## Preparar el entorno

Con Arduino CLI no hace falta enlazar la biblioteca: `--library firmware` compila la copia local (ver "Compilar y cargar"). Con el Arduino IDE, enlaza `firmware/` una vez desde la raiz del repositorio:

### Windows PowerShell

```powershell
.\scripts\link-library.ps1
```

Para reemplazar un enlace simbolico existente:

```powershell
.\scripts\link-library.ps1 -Force
```

### Linux/macOS

```bash
./scripts/link-library.sh
```

Para reemplazar un enlace simbolico existente:

```bash
./scripts/link-library.sh --force
```

Los scripts enlazan `firmware/` como `RobotComponents` dentro de la carpeta de bibliotecas del sketchbook de Arduino. Se puede cambiar la ubicacion usando la variable de entorno `ARDUINO_SKETCHBOOK`.

## Mapa de pines actual

Estos valores estan definidos en `seguidor_de_linea.ino`:

| Componente | Pines o configuracion |
| --- | --- |
| L298N MDV, lado izquierdo | `E1=2` (PWM), `M1=3` (sentido); conecta aqui las dos ruedas izquierdas en paralelo |
| L298N MDV, lado derecho | `E2=4` (PWM), `M2=5` (sentido); conecta aqui las dos ruedas derechas en paralelo |
| Zumbador | `10` |
| Garra / servo | `9`, abierta a `90` grados, cerrada a `0` grados, en pasos de `5` grados cada `15` ms |
| Sensores de linea | Analogicos `A0` (izquierdo), `A1` (central), `A2` (derecho) |
| LEDs de giro | `11` (LED 0, izquierdo), `12` (LED 1, derecho) |
| Ultrasonico | Trigger `7`, echo `6` |
| Bluetooth | `Serial1` de la Mega: RX1 `19`, TX1 `18`, 9600 baudios |
| Encoders de las ruedas (DFRobot SEN0038, 20 PPR) | Izquierda `20`, derecha `21` (con `attachInterrupt`, necesitan pines de interrupcion; ocupan el I2C de la Mega). Rueda de 65 mm |

Comprueba que los pines PWM y los niveles de direccion coincidan con el driver de motores usado por el hardware.

## Compilar y cargar

Con Arduino CLI, desde la raiz del repositorio:

```bash
arduino-cli compile --fqbn arduino:avr:mega --library firmware .
arduino-cli upload -p <puerto> --fqbn arduino:avr:mega .
```

Sustituye `<puerto>` por el COM del USB de la Mega (`arduino-cli board list`).

Con el Arduino IDE:

1. Abre `seguidor_de_linea.ino`.
2. Selecciona la placa Arduino Mega y el puerto correcto.
3. Instala las dependencias indicadas si el IDE las solicita.
4. Ejecuta **Verify/Compile**.
5. Conecta el robot y ejecuta **Upload**.

Al encender, el robot empieza el recorrido sin esperar al PC: colocalo antes sobre la linea de salida.

## Modos del sketch

Al principio de `seguidor_de_linea.ino` hay tres constantes que eligen que hace el robot al encender:

| Constante | En 1 | En 0 (por defecto) |
| --- | --- | --- |
| `CALIBRATE_WHEELS` | Solo calibra las ruedas (ver "Calibrar las ruedas"). Tiene prioridad sobre `CALIBRATE`. | |
| `CALIBRATE` | Solo calibra los sensores de linea (ver "Calibrar los sensores de linea"). | |
| `COMMANDS` | Ademas del recorrido, crea la tarea que atiende los comandos del PC. Sin efecto con una calibracion. | El robot no lee comandos. |

Con las dos calibraciones en 0, el robot hace el recorrido. Despues de calibrar, vuelve a poner la constante en 0 y carga de nuevo el sketch.

## Calibraciones

Las calibraciones imprimen por el puerto del `Logger` los valores que hay que copiar en `seguidor_de_linea.ino`. No se guardan en EEPROM.

### Calibrar los sensores de linea

Deja el robot sobre la linea. Con `#define CALIBRATE 1`, al encender gira sobre su eje a la derecha durante 4 segundos (velocidad `110`) midiendo la lectura minima (fondo) y maxima (linea) de cada sensor, e imprime cada 500 ms y al final:

```text
sensor 0: min=... max=... threshold=...
```

Cada umbral es el minimo mas un tercio del rango. Un sensor que casi no cambio (rango de 100 o menos) conserva su umbral. Copia los `threshold` en `thresholds[3]`, en el orden izquierdo, central y derecho. Un sensor ve la linea cuando su lectura es mayor o igual que su umbral.

Con `COMMANDS` en 1 tambien se puede calibrar con el comando `c` (ver "Comandos desde el PC"); los umbrales se aplican al momento pero se pierden al apagar.

### Calibrar las ruedas

Cada motor necesita un PWM minimo distinto para arrancar y, por encima de ese minimo, uno corre mas que el otro. La calibracion lo mide con los encoders:

1. Levanta el robot para que las ruedas giren en el aire.
2. Pon `#define CALIBRATE_WHEELS 1` y carga el sketch.
3. El robot sube el PWM de las dos ruedas desde `30` hasta `255`, de 5 en 5 cada 500 ms, e imprime `pwm`, `rpm_l` y `rpm_r`.
4. El primer PWM con el que el encoder ve giro es el minimo de esa rueda. Con las rpm por punto de PWM por encima del minimo calcula cuanto corre cada rueda, y escala la mas rapida para igualar a la lenta.
5. Copia los valores que imprime en `MIN_PWM_L`, `MIN_PWM_R`, `WHEEL_TRIM_L` y `WHEEL_TRIM_R`, y vuelve a poner `CALIBRATE_WHEELS` en 0.

Si una rueda no gira o su encoder no se lee, imprime un aviso y no calcula valores.

`Movement::drive_compensated()` aplica esos valores: cualquier velocidad distinta de 0 da al menos el PWM minimo de la rueda, y la parte por encima del minimo se escala con `WHEEL_TRIM`. El seguidor de linea siempre mueve los motores asi.

## Como funciona

`setup()` llama a `setup()` de cada componente, configura la compensacion de las ruedas y crea las tareas de `Arduino_FreeRTOS`. `loop()` queda vacio porque la logica corre en tareas:

| Tarea | Archivo | Prioridad | Que hace |
| --- | --- | --- | --- |
| `follow_line` | `behaviors/line_follower.h` | 2 | Sigue la linea cada 15 ms con periodo fijo (`xTaskDelayUntil`), para que el PID calcule siempre con el mismo intervalo. Se borra al llegar a la meta. |
| `collision` | `behaviors/collision.h` | 1 | Mide la distancia cada 65 ms y acciona la garra ante un objeto. |
| `commands` | `commands/command_task.h` | 1 | Solo con `COMMANDS` en 1: lee los comandos del PC cada 90 ms. Tiene menos prioridad que `follow_line` para poder detenerlo entre dos vueltas. |
| `calibrate` / `calibrate_wheels` | `behaviors/calibration.h` | 1 | Solo con `CALIBRATE` o `CALIBRATE_WHEELS` en 1, en lugar de las demas. |

Cada comportamiento esta en un `namespace` con sus constantes, su estado y un struct `Components` con referencias a los componentes que usa; el sketch se lo pasa a `xTaskCreate`.

### Seguimiento de linea

En cada vuelta, `line_follower::step()` lee los 3 sensores (valor y si supera su umbral) y recorre una lista de fases (`PHASES`) en orden. La primera que mueve el robot decide la vuelta:

1. `ignore_other_branch`: durante `FORK_COMMIT_MS` (400 ms) despues de tomar la rama derecha, el sensor izquierdo no cuenta.
2. `leave_start`: mientras sigue sobre la linea de salida avanza recto a `START_SPEED` (130) y no busca la meta.
3. `search_side` y `undo_search`: busqueda de la rama derecha y vuelta atras si no la encuentra (ver "Bifurcaciones").
4. `forget_search`: con el sensor central sobre la linea se puede volver a buscar la rama derecha.
5. `track_all_black`: los 3 sensores en negro durante `FINISH_MS` (600 ms) seguidos es la meta.
6. `cross_horizontal`: con los dos laterales en negro cruza recto a `ALL_BLACK_SPEED` (80), sin PID.
7. `search_before_other_side`: solo el sensor izquierdo ve la linea; antes de girar a la izquierda busca una rama a la derecha.
8. `follow_line`: sigue la linea con el PID o, si ningun sensor la ve, la recupera.

**PID.** El error es la posicion de la linea ponderada por la lectura de los sensores que la ven: `-2000` bajo el izquierdo, `0` bajo el central y `2000` bajo el derecho; una linea entre dos sensores da un valor intermedio. `PID_v1` (`KP` 0.152, `KI` 0, `KD` 0.001, en modo `REVERSE`, salida limitada a `±255`) da la correccion, que se suma a una rueda y se resta a la otra. Hasta un error de `SLOWDOWN_FROM_ERROR` (1000) la velocidad base es `speeds.base`; por encima baja hasta `speeds.curve`, asi que en las curvas cerradas gira casi sobre su eje. La rueda interior no baja de `speeds.inner_min`: en 0 se detiene en vez de ir hacia atras, lo que pide menos fuerza a los motores. Las velocidades iniciales son `{92, 0, 0}` y las ganancias estan al principio de `line_follower.h`. `PID_v1` divide `Kd` entre el periodo en segundos, asi que `Kd` debe ser mucho menor que `Kp`.

**Bifurcaciones.** La rama preferida es `FORK_SIDE` (1, derecha). Con `PRIORITIZE_FORK_SIDE`, la prioridad en cualquier desvio es derecha, recto, izquierda: mientras el central ve la linea, el sensor izquierdo no cuenta para el error. Si solo el izquierdo ve la linea, o si la linea se pierde viniendo centrada (el central cae en el hueco de una Y), el robot gira hacia la derecha durante `FORK_FIRST_MS` (350 ms), con la rueda exterior a `FORK_SEARCH_SPEED` (220) y la interior a `speeds.inner_min`. Si el central o el derecho encuentran linea, se queda con esa rama. Si no, deshace el giro con el mismo movimiento marcha atras y sigue hacia la izquierda; no vuelve a buscar hasta que el central vea la linea. Cada curva a la izquierda cuesta unos 700 ms extra.

**Linea perdida.** Si ningun sensor ve la linea:

- Justo despues de una linea horizontal (en `AFTER_BLACK_MS`, 100 ms), revisa si era la meta (ver abajo).
- Si venia centrada, busca la rama derecha y despues gira a la izquierda como mucho `FORK_SECOND_MS` (500 ms).
- Si venia hacia un lado, mantiene las ultimas velocidades durante `LOST_GRACE_MS` (250 ms): en una curva la linea suele aparecer enseguida.
- Si sigue sin aparecer, retrocede a `RECOVER_SPEED` (120) girando hacia el ultimo lado donde la vio (`RECOVER_TURN`).

**Salida, cruces y meta.** Se ven igual (los laterales en negro un momento); cambia lo que hay despues. El robot cruza toda linea horizontal recto a `ALL_BLACK_SPEED`. Si al salir del negro la linea no aparece en `FINISH_CHECK_MS` (100 ms), era la meta: retrocede hasta que al menos 2 sensores vean la linea (como mucho 1 s) y se para encima. Como respaldo, 3 sensores en negro durante `FINISH_MS` tambien es la meta. Se perdonan `ALL_BLACK_TOLERANCE_MS` (40 ms) de lecturas fallidas, por ejemplo en las uniones de la cinta. Al llegar imprime `meta`, detiene los motores, emite dos beeps de 800 ms y borra la tarea.

**LEDs.** Cada vez que el seguidor cambia la velocidad de los motores, `LedArray::show_turn()` enciende el LED del lado hacia el que gira (el de la rueda mas lenta), o los dos si la diferencia entre ruedas no pasa de `STRAIGHT_DEADBAND` (30).

### Objetos y garra

`collision::task` mide la distancia con un alcance de 14 cm cada 65 ms (60 ms o mas entre disparos, para que el eco anterior no se lea como nuevo). Con un objeto a `DISTANCE_CM` (7) o menos en `CONFIRMATIONS` (3) lecturas seguidas imprime `collision`, abre la garra y la vuelve a cerrar. Corre en paralelo con el seguimiento: el robot no se detiene. La medicion suspende el planificador y usa `pulseInLong`, porque las interrupciones del servo, del puerto serie y del tick acortaban el eco medido con `pulseIn`; sin eco dentro del alcance devuelve 999.

## Comandos desde el PC

Los comandos estan desactivados por defecto. Para usarlos, pon `#define COMMANDS 1` en `seguidor_de_linea.ino` y vuelve a cargar el sketch: se crea la tarea `commands`, que lee los comandos cada 90 ms. No tiene efecto con `CALIBRATE` o `CALIBRATE_WHEELS`.

Van por el puerto que elige `COMMANDS_OVER_USB` en `bluetooth_config.h` (ver "Puerto serie y registro"), a `BLUETOOTH_BAUD` (`9600`). La consola los envia con una tecla y muestra todo lo que responde el robot:

```powershell
.\scripts\console.ps1
```

Usa `COM3` por defecto; con `-Port COM5` eliges otro puerto. Reintenta la conexion si falla o si se corta.

| Tecla | Que hace | Respuesta del robot |
| --- | --- | --- |
| `g` | Empieza otro recorrido desde la linea de salida. | `run,ms,start`, o `run,ms,ignored` si ya hay uno en marcha |
| `x` | Detiene el recorrido en marcha y para los motores. | `run,ms,stop`, o `run,ms,ignored` si no habia ninguno |
| `c` | Calibra los sensores de linea. | `calibration,start`, las lecturas de cada sensor y `calibration,complete`, o `calibration,ignored` con un recorrido en marcha |
| `p` | Pide `Kp Ki Kd` por teclado y los cambia; con Enter vacio solo los consulta. | `pid,kp,ki,kd` con las ganancias en uso, o `pid,error` |
| `b` | Pide las velocidades `base curva interior` y las cambia; con Enter vacio solo las consulta. | `speeds,base,curva,interior` con las velocidades en uso, o `speeds,error` |
| `r` | Empieza un registro y lo guarda en `logs\run-AAAAMMDD-HHMMSS.csv`. | `recording,ms,start` y la cabecera |
| `s` | Detiene el registro y cierra el CSV. | `recording,ms,stop` |
| `v` | Muestra u oculta las filas de telemetria en pantalla (se guardan igual). | |
| `h` | Muestra la ayuda. | |
| `q` | Sale; detiene el registro si habia uno. | |

Las teclas `g`, `x`, `c`, `r` y `s` se envian tal cual al robot. `p` envia una linea: `p<kp>,<ki>,<kd>` para cambiar las ganancias o `p` sola para consultarlas; `b` funciona igual, con `b<base>,<curva>,<interior>`. Si la linea de valores no termina en 2 s, se descarta y el robot responde con el error. Las demas teclas solo afectan a la consola. Las letras estan en `firmware/src/commands/commands.h` y deben coincidir con las de `scripts/console.ps1`.

Las ganancias, las velocidades y los umbrales de una calibracion solo duran hasta que se apaga el robot: no se guardan en EEPROM. Para conservarlos, copia los valores en el codigo (`KP`, `KI`, `KD` y `speeds` en `firmware/src/behaviors/line_follower.h`, `thresholds` en `seguidor_de_linea.ino`).

### Repetir el recorrido

El primer recorrido empieza solo al encender. Cuando llega a la meta, el robot se detiene y `g` empieza otro sin reiniciarlo. Coloca antes el robot sobre la linea de salida: se mueve en cuanto recibe el comando.

Para cortar un recorrido a la mitad, pulsa `x`: el robot se detiene y queda esperando comandos. `xg` lo detiene y empieza uno nuevo desde cero.

### Ajustar las velocidades

Pulsa `b` y escribe tres numeros separados por espacios, por ejemplo `92 0 0`:

- **base (0 a 255):** velocidad con la linea bajo el sensor central.
- **curva (0 a 255):** velocidad base con la linea bajo un sensor lateral. Entre las dos se interpola segun el error.
- **interior (-255 a 255):** velocidad minima de la rueda interior al girar. Negativa, esa rueda va hacia atras.

El robot las aplica en el siguiente ciclo de control, sin detener el recorrido, y responde `speeds,base,curva,interior`. Con un valor fuera de rango, o si la linea no tiene 3 numeros, responde `speeds,error` y no cambia nada.

### Ajustar el PID

Pulsa `p` y escribe las tres ganancias separadas por espacios, por ejemplo `0.152 0 0.001`. El robot las aplica en el siguiente ciclo de control, sin detener el recorrido, y responde `pid,kp,ki,kd`. Cada ganancia debe estar entre `0` y `10`: fuera de ese rango, o si la linea no tiene 3 numeros, el robot responde `pid,error` y no cambia nada.

### Calibrar con la consola

Deten antes el recorrido con `x` y deja el robot sobre la linea. Al pulsar `c` hace la misma calibracion que `CALIBRATE` (ver "Calibrar los sensores de linea") y aplica los umbrales. Mientras calibra no atiende otros comandos.

## Puerto serie y registro

Los mensajes del robot y los comandos van por el puerto que elige `COMMANDS_OVER_USB` en `bluetooth_config.h`: `true` usa el cable USB de la Mega (`Serial`) y `false` el Bluetooth (`Serial1`), los dos a 9600 baudios. Por USB, abre la consola con el puerto COM de la Mega (`arduino-cli board list`), por ejemplo `.\scripts\console.ps1 -Port COM7`. El cable USB tambien alimenta el Arduino.

El robot envia siempre estos mensajes, haya o no un registro:

- `meta` al llegar a la meta.
- `collision` al accionar la garra.
- La salida de las calibraciones.
- Las respuestas a los comandos (`run`, `pid`, `speeds` y `calibration`).

El comando `r` empieza un registro: el robot responde `recording,ms,start`, los umbrales en uso (`thresholds,s0,s1,s2`), las ganancias (`pid,kp,ki,kd`) y las velocidades (`speeds,base,curva,interior`), y la consola guarda todo lo que llega en `logs\run-AAAAMMDD-HHMMSS.csv`, un archivo nuevo por registro, hasta que pulsas `s` o sales con `q`. El robot ya no envia filas de telemetria (`t,...`) ni eventos: `Logger::telemetry()` y `Logger::event()` existen, pero ningun comportamiento los llama.

## Configurar el Bluetooth

El nombre y la velocidad del HC-05 estan en `bluetooth_config.h` (`BLUETOOTH_NAME` y `BLUETOOTH_BAUD`). La clase `Hc05` (`components/hc05.h`) los envia con comandos AT a 38400 baudios, pero el sketch actual no la usa: `BLUETOOTH_SETUP_MODE` ya no tiene efecto. Para cambiar el nombre o la velocidad hay que volver a llamar, desde `setup()`, a `bluetooth.setup()` y `bluetooth.configure(BLUETOOTH_NAME, BLUETOOTH_BAUD)` sin crear las tareas, con el modulo en modo AT:

1. Desconecta el `VCC` del HC-05, mantén pulsado su boton, vuelve a conectar `VCC` y suelta el boton. Su LED debe parpadear lento, cada 2 segundos aproximadamente.
2. Abre el monitor del puerto USB de la Mega a `9600` baudios y pulsa RESET en la Mega. Cada comando debe responder `OK`.
3. Desconecta y vuelve a conectar el `VCC` del HC-05 para salir del modo AT, y vuelve a cargar el sketch normal.

Si Windows ya tenia vinculado el modulo, quitalo y vinculalo de nuevo para ver el nombre nuevo.

## Flujo de desarrollo

1. Crea una rama para cada cambio.
2. Conserva `firmware/` como fuente de verdad; no edites la copia instalada en el sketchbook.
3. Haz cambios pequenos en el componente o comportamiento responsable.
4. Compila el sketch antes de probar en el robot.
5. Prueba primero con las ruedas elevadas y luego en una zona despejada.
6. Documenta en el pull request los pines, parametros calibrados y pruebas realizadas.

Antes de abrir un pull request, comprueba:

- El sketch compila sin errores.
- Los cambios no reutilizan un pin ya ocupado.
- Los limites de velocidad estan entre `-255` y `255`.
- Los movimientos de la garra y del robot se han probado sin personas ni objetos en la trayectoria.
- La documentacion refleja cualquier cambio de cableado o dependencia.

## Ideas pendientes

- Emitir un beep al derribar un objeto, como piden las reglas (`COMPETENCIA.md`).
- Apagar los dos LEDs cuando el robot va recto, como piden las reglas, en vez de encender los dos.
- Detener o frenar el robot mientras la garra retira el objeto.
- Volver a enviar telemetria durante un registro, o quitar la columna de telemetria de la consola.
- Recuperar un modo para configurar el HC-05 desde el sketch.
- Implementar la lectura del sensor RGB real.
- Añadir pruebas para el calculo del error de los sensores de linea.
