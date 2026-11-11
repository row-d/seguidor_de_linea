# Seguidor de linea

Software para un robot movil que sigue una linea, detecta obstaculos y puede recoger/soltar objetos con una garra.

El sketch principal ensambla los componentes del robot. La implementacion reutilizable esta en la biblioteca local `RobotComponents`, dentro de `firmware/`.

## Estado actual

- El robot empieza el recorrido al encender. La calibracion, el registro de telemetria y un recorrido nuevo se piden desde el PC por Bluetooth (ver "Comandos desde el PC").
- El seguimiento de linea usa un controlador PID sencillo.
- La tarea de calibracion calcula umbrales, los guarda en EEPROM y envia el resultado por Bluetooth usando `Serial1`.
- La deteccion de obstaculos se hace con un sensor ultrasonico a menos de 15 cm.
- Ante un obstaculo, el robot abre la garra, avanza, cierra la garra, gira, suelta el objeto a un lado y vuelve a la linea.
- La telemetria se envia por Bluetooth o USB con la clase `Logger`.
- `read_rgb_sensor()` es un stub: actualmente siempre devuelve `{0, 0, 0}`. Hay que conectarlo al sensor RGB real antes de usar `ColorSensor`.
- No hay una suite automatizada de pruebas; la validacion principal se hace compilando y probando el robot en hardware.

## Estructura

```text
seguidor_de_linea.ino       Sketch principal y configuracion del robot
bluetooth_config.h          Nombre y velocidad del modulo HC-05
firmware/
  library.properties        Metadatos y dependencias de la biblioteca
  src/
    RobotComponents.h       Cabecera publica de la biblioteca
    components/             Componentes reutilizables del robot
      movement.h            Motores y direccion
      line_sensors.h        Sensores de linea y calculo del error
      color_sensor.h        Lectura y comparacion de color
      claw.h                Garra controlada por servo
      buzzer.h               Zumbador
      led_array.h            LEDs de giro
      ultrasonic_sensor.h    Medicion de distancia
      logger.h               Registro y telemetria por Serial o Bluetooth
      hc05.h                 Configuracion del HC-05 con comandos AT
      component.h            Interfaz comun de componentes
    tasks/                  Tareas y control del robot
      robot.h               Declaracion del controlador
      start.h               Seguimiento, meta y bifurcaciones
      route_memory.h        Ramas visitadas de cada bifurcacion
      collision.h           Retiro de objetos
      calibration.h         Calibracion de sensores
      pid_gains.h           Ganancias del PID: EEPROM y cambios desde el PC
      telemetry.h           Tareas de telemetria/comandos y de calibracion
scripts/
  console.ps1               Consola: muestra lo que envia el robot y le manda comandos
  link-library.ps1          Enlace simbolico para Windows
  link-library.sh           Enlace simbolico para Linux/macOS
```

## Requisitos

- Arduino IDE o Arduino CLI.
- Arduino Mega, seleccionada en el IDE.
- Bibliotecas Arduino:
  - `Servo`
  - `Arduino_FreeRTOS`
  - `PID` (`PID_v1`)
- Git, para colaborar con el proyecto.
- En Windows, permisos para crear enlaces simbolicos. Si el script falla, ejecuta VS Code o PowerShell con los permisos necesarios o crea el enlace desde una terminal con permisos adecuados.

El esquema usa una Arduino Mega. No cambies los pines sin comprobar primero el cableado real.

## Preparar el entorno

Desde la raiz del repositorio:

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
| Garra / servo | `8`, abierta a `20` grados, cerrada a `90` grados |
| Sensores de linea | Analogicos `A0`, `A1`, `A2` |
| LEDs de giro | `11`, `12` |
| Ultrasonico | Trigger `7`, echo `6` |
| Sensor RGB | Pendiente de conectar en `read_rgb_sensor()` |
| Bluetooth | `Serial1` de la Mega: RX1 `19`, TX1 `18`, 9600 baudios |

Comprueba que los pines PWM y los niveles de direccion coincidan con el driver de motores usado por el hardware.

## Compilar y cargar

1. Abre `seguidor_de_linea.ino` en Arduino IDE.
2. Selecciona la placa y el puerto correctos.
3. Instala las dependencias indicadas si el IDE las solicita.
4. Ejecuta **Verify/Compile**.
5. Conecta el robot y ejecuta **Upload**.

Al encender, el robot empieza el recorrido sin esperar al PC: colocalo antes en la zona de inicio.

## Comandos desde el PC

Cada comando es una letra que se envia por el puerto COM del modulo Bluetooth, a `9600` baudios. La consola los envia con una tecla y muestra todo lo que responde el robot:

```powershell
.\scripts\console.ps1
```

Usa `COM3` por defecto; con `-Port COM5` eliges otro puerto. Reintenta la conexion si falla o si se corta.

| Tecla | Que hace | Respuesta del robot |
| --- | --- | --- |
| `g` | Empieza otro recorrido cuando el anterior ya termino. | `run,ms,start`, o `run,ms,ignored` si ya hay un recorrido en marcha o esta calibrando |
| `x` | Detiene el recorrido en marcha y para los motores. | `run,ms,stop` |
| `c` | Calibra los sensores de linea. | `calibration,start`, la tabla de resultados y `calibration,complete` |
| `p` | Pide `Kp Ki Kd` por teclado y los cambia; con Enter vacio solo los consulta. | `pid,kp,ki,kd` con las ganancias en uso, o `pid,error` |
| `r` | Empieza un registro de telemetria y lo guarda en `logs\run-AAAAMMDD-HHMMSS.csv`. | `recording,ms,start` y la cabecera |
| `s` | Detiene el registro y cierra el CSV. | `recording,ms,stop` |
| `v` | Muestra u oculta las filas de telemetria en pantalla (se guardan igual). | |
| `h` | Muestra la ayuda. | |
| `q` | Sale; detiene el registro si habia uno. | |

Las teclas `g`, `x`, `c`, `r` y `s` se envian tal cual al robot. `p` envia una linea: `p<kp>,<ki>,<kd>` para cambiar las ganancias o `p` sola para consultarlas. Las demas solo afectan a la consola.

### Repetir el recorrido

El primer recorrido empieza solo al encender. Cuando termina (`finish`, `no_route`, `fork_missed` o `memory_full`), el robot se detiene y `g` empieza otro sin reiniciarlo. Coloca antes el robot en la zona de inicio: se mueve en cuanto recibe el comando. Para grabar ese recorrido desde el principio, pulsa `r` y despues `g`.

Para cortar un recorrido a la mitad, pulsa `x`: el robot se detiene de inmediato, incluso en medio de un giro, y queda esperando comandos. Despues, `g` empieza un recorrido nuevo desde cero, sin las bifurcaciones del anterior. Si se pulsa `x` durante una calibracion, la calibracion termina igual pero el robot no retoma el recorrido.

### Ajustar el PID

Pulsa `p` y escribe las tres ganancias separadas por espacios, por ejemplo `0.3 0 0.006`. El robot las aplica en el siguiente ciclo de control, sin detener el recorrido, las guarda en EEPROM y responde `pid,kp,ki,kd`. En cada arranque usa las guardadas; si nunca se guardo ninguna, usa las de `firmware/src/tasks/robot.h` (`0.25`, `0.0` y `0.005`). Cada ganancia debe estar entre `0` y `10`: fuera de ese rango, o si la linea no tiene 3 numeros, el robot responde `pid,error` y no cambia nada.

### Calibrar los sensores de linea

Pulsa `c`. Los motores se detienen durante 10 segundos desde que aparece `calibration,start`: mueve manualmente los sensores sobre el fondo y sobre la linea para que cada sensor registre ambos valores. El robot guarda en EEPROM la lectura minima (fondo) y maxima (linea) de cada sensor, fija cada umbral a un tercio de ese rango y envia un CSV con `sensor`, `min`, `max` y `threshold`. Un sensor que no llego a pasar por la linea (rango menor que 100) conserva sus valores anteriores. Los niveles guardados se cargan desde EEPROM en cada arranque. Si habia un recorrido en marcha, el robot lo retoma al terminar (`calibration,complete`).

Con Arduino CLI, el flujo equivalente es:

```bash
arduino-cli compile --fqbn <fabricante:arquitectura:placa> .
arduino-cli upload -p <puerto> --fqbn <fabricante:arquitectura:placa> .
```

Sustituye `<fabricante:arquitectura:placa>` y `<puerto>` por los valores de tu instalacion.

## Como funciona

`setup()` inicializa el puerto serie, configura todos los componentes, crea las tareas de servicio y empieza el recorrido. `loop()` queda vacio porque la logica corre en tareas de `Arduino_FreeRTOS`:

| Tarea | Prioridad | Que hace |
| --- | --- | --- |
| `Start` (`start.h`) | 2 | Control: sigue la linea. Solo existe durante un recorrido: `setup()` la crea con `Robot::start()` al encender y se borra al terminar. `Telemetry` la borra al recibir `x` y la vuelve a crear al recibir `g`. |
| `Calibration` (`telemetry.h`, `calibration.h`) | 2 | Duerme hasta que le piden calibrar. Mide 10 segundos y guarda los umbrales. |
| `Telemetry` (`telemetry.h`) | 1 | La unica que usa el puerto serie: lee los comandos del PC y envia telemetria, eventos y el resultado de la calibracion. |

Las tareas se comunican asi:

- **Telemetria:** en cada ciclo, `Start` deja su ultima muestra en una cola de 1 elemento. `Telemetry` la recoge cada 6 ticks (unos 100 ms). Como tiene menos prioridad, un puerto lento nunca retrasa el control.
- **Eventos:** `logger.event()` los guarda en una cola de 8 con su hora, y `Telemetry` los envia. Si la cola se llena, el evento se pierde.
- **Calibracion:** al recibir `c`, `Telemetry` activa una peticion. `Start` la ve al empezar su siguiente ciclo, detiene los motores, despierta a `Calibration` y espera a que termine. Si `Start` no existe o ya se detuvo, `Telemetry` despierta a `Calibration` directamente.

En cada iteracion de la tarea de control:

1. Calcula la posicion de la linea con los sensores izquierdo, central y derecho. Cada lectura se convierte a un nivel entre 0 (fondo) y 1000 (linea centrada) con los valores de la calibracion, y la posicion es el promedio ponderado por nivel: una linea que queda entre dos sensores da un error intermedio. La linea se da por perdida si ningun sensor llega al nivel 180 (`LINE_MIN_LEVEL` en `line_sensors.h`). Los umbrales solo se usan para reconocer bifurcaciones y lineas horizontales.
2. Usa `PID_v1` para convertir ese error en una correccion diferencial de los motores.
3. Recuerda el lado donde vio la linea por ultima vez. Si la pierde, gira sobre su eje hacia ese lado hasta encontrarla.
4. Si hay un registro activo, publica la muestra del ciclo para la tarea de telemetria. Despues espera 2 ticks de FreeRTOS (unos 32 ms). La deteccion de obstaculos esta desactivada por ahora.

Ademas, reacciona a estos casos:

- **Zona de inicio:** durante un maximo de `start_zone_ms`, avanza recto hasta encontrar la linea y cruza la linea horizontal de inicio sin tomarla por la meta. Debe colocarse mirando hacia esa linea y centrado con ella.
- **Linea horizontal (los 3 sensores):** la cruza en linea recta. Si despues ve linea en algun sensor lateral sin el central, era el vertice de una Y. Si detras no hay linea, es la meta y se detiene. Si la linea continua, sigue el recorrido. Una T se confunde con la meta. Si los 3 sensores siguen activos mas de 1 s, se detiene hasta que la lectura cambie.
- **Bifurcacion en Y (los dos sensores laterales ven linea con menos de `side_memory_ms` de diferencia):** la guarda en `RouteMemory`, gira hacia `preferred_route`, definida en `seguidor_de_linea.ino`, y enciende 1 s el LED de esa ruta.
- **Callejon sin salida:** si recuperar la linea exige girar `dead_end_search_ms` o mas, el robot ya dio media vuelta y sigue la linea de regreso hasta la bifurcacion. Alli marca la rama como visitada y toma la otra. Si las dos ramas no tienen salida, vuelve a la bifurcacion anterior. Se detiene con `no_route` si no quedan opciones y con `fork_missed` si tarda demasiado en volver a la Y. No admite recorridos con bucles.
- **Objeto a menos de 15 cm:** se detiene, avanza con la garra abierta, cierra, gira a la derecha, suelta el objeto, emite un beep y vuelve a la linea. Los tiempos y distancias estan en `firmware/src/tasks/collision.h` y deben ajustarse al robot.

Las ganancias del PID se cambian desde el PC y se guardan en EEPROM (ver "Ajustar el PID"); sus valores iniciales (`0.25`, `0.0` y `0.005`) estan en `firmware/src/tasks/robot.h`. `PID_v1` divide `Kd` entre el periodo en segundos, asi que `Kd` debe ser mucho menor que `Kp`. Las velocidades base (`130`) y de giro (`230`) estan en `firmware/src/tasks/robot.h`.

## Configurar el Bluetooth

El nombre y la velocidad del HC-05 estan en `bluetooth_config.h`. Para aplicarlos:

1. Edita `BLUETOOTH_NAME`, deja `BLUETOOTH_SETUP_MODE` en `true` y sube el sketch por USB.
2. Abre el monitor del puerto USB de la Mega a `9600` baudios.
3. Pon el HC-05 en modo AT: desconecta su `VCC`, mantén pulsado su boton, vuelve a conectar `VCC` y suelta el boton. Su LED debe parpadear lento, cada 2 segundos aproximadamente.
4. Pulsa RESET en la Mega. El monitor debe terminar con `bluetooth,ok`.
5. Desconecta y vuelve a conectar el `VCC` del HC-05 para salir del modo AT.
6. Cambia `BLUETOOTH_SETUP_MODE` a `false` y vuelve a subir el sketch.

Si Windows ya tenia vinculado el modulo, quitalo y vinculalo de nuevo para ver el nombre nuevo.

## Telemetria

La clase `Logger` escribe por el puerto elegido en `seguidor_de_linea.ino`: `Logger logger{Serial1, 9600}` para Bluetooth o `Logger logger{Serial, 9600}` para USB.

El robot no envia telemetria ni eventos hasta que el PC le pide un registro: el comando `r` lo empieza y `s` lo detiene. En la consola (`scripts\console.ps1`, ver "Comandos desde el PC"), `r` guarda ademas todo lo que llega en `logs\run-AAAAMMDD-HHMMSS.csv`, un archivo nuevo por registro, hasta que pulsas `s` o sales con `q`.

Cada registro empieza con `recording,ms,start`, los umbrales en uso (`thresholds,s0,s1,s2`), las ganancias del PID (`pid,kp,ki,kd`) y la cabecera de columnas. Despues, cada linea es de uno de estos tipos:

- Telemetria, unas 10 filas por segundo: `t,ms,s0,s1,s2,error,correction,left,right,line`. `ms` es la hora en que el control tomo la muestra.
- Ganancias del PID, al consultarlas o cambiarlas: `pid,kp,ki,kd` o `pid,error`.
- Inicio y fin de recorrido pedidos desde el PC: `run,ms,start`, `run,ms,ignored` o `run,ms,stop`.
- Evento: `event,ms,nombre`, donde `nombre` es `route_left`, `route_right`, `dead_end`, `back_to_stem`, `object_detected`, `object_removed`, `invalid_reading`, `no_route`, `fork_missed`, `memory_full` o `finish`.

## Flujo de desarrollo

1. Crea una rama para cada cambio.
2. Ejecuta el script de enlace una vez y conserva `firmware/` como fuente de verdad; no edites la copia instalada en el sketchbook.
3. Haz cambios pequenos en el componente responsable.
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

- Implementar la lectura del sensor RGB real.
- Añadir una tabla de placa, driver de motores y esquema de cableado.
- Separar la logica de control de hardware para poder probarla sin el robot.
- Añadir mensajes de diagnostico por puerto serie.
- Añadir pruebas para el calculo del error de los sensores de linea y la deteccion de color.