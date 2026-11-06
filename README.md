# Seguidor de linea

Software para un robot movil que sigue una linea, detecta obstaculos y puede recoger/soltar objetos con una garra.

El sketch principal ensambla los componentes del robot. La implementacion reutilizable esta en la biblioteca local `RobotComponents`, dentro de `firmware/`.

## Estado actual

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
scripts/
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
6. Abre el puerto COM del modulo Bluetooth a `9600` baudios para recibir la calibracion.

Para calibrar, deja `calibration_mode` en `true`. Los motores permanecen detenidos durante 10 segundos: mueve manualmente los sensores sobre el fondo y sobre la linea para que cada sensor registre ambos valores. La tarea calcula el umbral medio, lo guarda en EEPROM y despues envia por Bluetooth un CSV con `sensor`, `min`, `max` y `threshold`. Para volver al seguimiento normal, cambia `calibration_mode` a `false`; los umbrales guardados se cargaran automaticamente desde EEPROM.

Con Arduino CLI, el flujo equivalente es:

```bash
arduino-cli compile --fqbn <fabricante:arquitectura:placa> .
arduino-cli upload -p <puerto> --fqbn <fabricante:arquitectura:placa> .
```

Sustituye `<fabricante:arquitectura:placa>` y `<puerto>` por los valores de tu instalacion.

## Como funciona

`setup()` inicializa el puerto serie, configura todos los componentes y arranca el controlador. `loop()` queda vacio porque `Robot::start()` crea una tarea de `Arduino_FreeRTOS`.

En cada iteracion de la tarea:

1. Calcula la posicion de la linea con los sensores izquierdo, central y derecho.
2. Usa `PID_v1` para convertir ese error en una correccion diferencial de los motores.
3. Recuerda el lado donde vio la linea por ultima vez. Si la pierde, gira sobre su eje hacia ese lado hasta encontrarla.
4. Cada 3 ciclos envia una fila de telemetria si hay un registro activo. Despues espera 2 ticks de FreeRTOS (unos 32 ms). La deteccion de obstaculos esta desactivada por ahora.

Ademas, reacciona a estos casos:

- **Zona de inicio:** durante un maximo de `start_zone_ms`, avanza recto hasta encontrar la linea y cruza la linea horizontal de inicio sin tomarla por la meta. Debe colocarse mirando hacia esa linea y centrado con ella.
- **Linea horizontal (los 3 sensores):** la cruza en linea recta. Si despues ve linea en algun sensor lateral sin el central, era el vertice de una Y. Si detras no hay linea, es la meta y se detiene. Si la linea continua, sigue el recorrido. Una T se confunde con la meta. Si los 3 sensores siguen activos mas de 1 s, se detiene hasta que la lectura cambie.
- **Bifurcacion en Y (los dos sensores laterales ven linea con menos de `side_memory_ms` de diferencia):** la guarda en `RouteMemory`, gira hacia `preferred_route`, definida en `seguidor_de_linea.ino`, y enciende 1 s el LED de esa ruta.
- **Callejon sin salida:** si recuperar la linea exige girar `dead_end_search_ms` o mas, el robot ya dio media vuelta y sigue la linea de regreso hasta la bifurcacion. Alli marca la rama como visitada y toma la otra. Si las dos ramas no tienen salida, vuelve a la bifurcacion anterior. Se detiene con `no_route` si no quedan opciones y con `fork_missed` si tarda demasiado en volver a la Y. No admite recorridos con bucles.
- **Objeto a menos de 15 cm:** se detiene, avanza con la garra abierta, cierra, gira a la derecha, suelta el objeto, emite un beep y vuelve a la linea. Los tiempos y distancias estan en `firmware/src/tasks/collision.h` y deben ajustarse al robot.

Los parametros de control estan en `firmware/src/tasks/start.h`: ganancias `0.15`, `0.0` y `0.003`. `PID_v1` divide `Kd` entre el periodo en segundos, asi que `Kd` debe ser mucho menor que `Kp`. Las velocidades base (`130`) y de giro (`185`) estan en `firmware/src/tasks/robot.h`.

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

El robot no envia telemetria ni eventos hasta que el PC le pide un registro: el comando `r` lo empieza y `s` lo detiene. Para grabar un registro en un CSV:

```powershell
.\scripts\record.ps1 -Port COM5
```

Sustituye `COM5` por el puerto de tu modulo. El script envia `r`, guarda todo en `logs\run-AAAAMMDD-HHMMSS.csv` y envia `s` al terminar. Termina con Ctrl+C, cuando el robot se detiene (`finish`, `no_route`, `fork_missed` o `memory_full`) o al pasar `-DurationSeconds`. Con `-OutputFile` eliges otro archivo.

Cada registro empieza con `recording,ms,start`, los umbrales en uso (`thresholds,s0,s1,s2`) y la cabecera de columnas. Despues, cada linea es de uno de estos tipos:

- Telemetria, unas 10 filas por segundo: `t,ms,s0,s1,s2,error,correction,left,right,line`. Una fila se descarta si el puerto va saturado.
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