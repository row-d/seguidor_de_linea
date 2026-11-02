# Seguidor de linea

Software para un robot movil que sigue una linea, detecta obstaculos y puede recoger/soltar objetos con una garra.

El sketch principal ensambla los componentes del robot. La implementacion reutilizable esta en la biblioteca local `RobotComponents`, dentro de `components/`.

## Estado actual

- Seguimiento de linea negra con tres tareas FreeRTOS: adquisicion, control PID y diagnostico.
- Solo la tarea de control escribe en motores y LEDs. Las tareas intercambian copias de datos mediante colas de un elemento.
- Calibracion individual sobre fondo y negro, con polaridad automatica al usar valores calibrados.
- Busqueda sobre el lugar al perder la linea, limitada a dos segundos. Durante el seguimiento ambos motores avanzan o se detienen.
- Si ambos sensores laterales detectan negro, se detiene mientras persista esa lectura. Tres sensores por si solos no distinguen cruce y meta.
- El ultrasonico, la garra y el RGB conservan sus componentes, pero no hay tarea de obstaculos activa. `read_rgb_sensor()` todavia devuelve `{0, 0, 0}`.
- Hay pruebas de logica en el computador; la calibracion y el ajuste definitivo requieren el robot real.

## Estructura

```text
seguidor_de_linea.ino       Sketch principal y configuracion del robot
components/
  library.properties        Metadatos y dependencias de la biblioteca
  src/
    RobotComponents.h       Cabecera publica de la biblioteca
<<<<<<< Updated upstream
    robot.h                 Logica del controlador y tarea FreeRTOS
    movement.h              Motores y direccion
    line_sensors.h          Sensores de linea y calculo del error
    color_sensor.h          Lectura y comparacion de color
    claw.h                  Garra controlada por servo
    buzzer.h                Zumbador
    led_array.h             LEDs de giro
    ultrasonic_sensor.h      Medicion de distancia
    component.h             Interfaz comun de componentes
=======
    components/             Componentes reutilizables del robot
      movement.h            Motores y direccion
      line_sensors.h        Sensores de linea y calculo del error
      color_sensor.h        Lectura y comparacion de color
      claw.h                Garra controlada por servo
      buzzer.h               Zumbador
      led_array.h            LEDs de giro
      ultrasonic_sensor.h    Medicion de distancia
      component.h            Interfaz comun de componentes
    tasks/                  Tareas y control del robot
      robot.h               Componentes, colas y tipos compartidos
      line_config.h         Parametros del seguimiento
      start.h               Creacion y comprobacion de tareas
      line_acquisition.h    Muestreo de los tres sensores
      line_control.h        PID y propietario de motores/LEDs
      line_recovery.h       Busqueda alternada con limite de tiempo
      debug.h               Diagnostico CSV sin movimientos
>>>>>>> Stashed changes
scripts/
  link-library.ps1          Enlace simbolico para Windows
  link-library.sh           Enlace simbolico para Linux/macOS
```

## Requisitos

- Arduino IDE o Arduino CLI.
- La placa Arduino compatible con el montaje y seleccionada en el IDE.
- Bibliotecas Arduino:
  - `Servo`
<<<<<<< Updated upstream
  - `Arduino_FreeRTOS`
=======
  - `FreeRTOS` de Phillip Stevens (cabecera `Arduino_FreeRTOS.h`)
  - `PID` (`PID_v1`)
>>>>>>> Stashed changes
- Git, para colaborar con el proyecto.
- En Windows, permisos para crear enlaces simbolicos. Si el script falla, ejecuta VS Code o PowerShell con los permisos necesarios o crea el enlace desde una terminal con permisos adecuados.

La placa concreta y el esquema electrico todavia deben documentarse. No cambies los pines sin comprobar primero el cableado real.

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

Los scripts enlazan `components/` como `RobotComponents` dentro de la carpeta de bibliotecas del sketchbook de Arduino. Se puede cambiar la ubicacion usando la variable de entorno `ARDUINO_SKETCHBOOK`.

## Mapa de pines actual

Estos valores estan definidos en `seguidor_de_linea.ino`:

| Componente | Pines o configuracion |
| --- | --- |
| Motor izquierdo | Direccion `2`, PWM `3` |
| Motor derecho | Direccion `4`, PWM `5` |
| Zumbador | `6` |
| Garra / servo | `8`, abierta a `20` grados, cerrada a `90` grados |
| Sensores de linea | Analogicos `0`, `1`, `2` |
| LEDs de giro | `10`, `11` |
| Ultrasonico | Trigger `12`, echo `13` |
| Sensor RGB | Pendiente de conectar en `read_rgb_sensor()` |

Comprueba que los pines PWM y los niveles de direccion coincidan con el driver de motores usado por el hardware.

## Compilar y cargar

1. Abre `seguidor_de_linea.ino` en Arduino IDE.
2. Selecciona la placa y el puerto correctos.
3. Instala las dependencias indicadas si el IDE las solicita.
4. Ejecuta **Verify/Compile**.
5. Conecta el robot y ejecuta **Upload**.
6. Abre el monitor serie a `9600` baudios para observar el diagnostico CSV.

Con Arduino CLI, el flujo equivalente es:

```bash
arduino-cli compile --fqbn <fabricante:arquitectura:placa> .
arduino-cli upload -p <puerto> --fqbn <fabricante:arquitectura:placa> .
```

Sustituye `<fabricante:arquitectura:placa>` y `<puerto>` por los valores de tu instalacion.

## Como funciona

`setup()` inicializa los componentes y llama a `start(line_control)`. `loop()` queda vacio porque FreeRTOS ejecuta las tareas.

| Tarea | Prioridad | Funcion |
| --- | --- | --- |
| `LineSensors` | 3 | Lee una vez A0, A1 y A2 por ciclo, calcula posicion y publica la muestra mas reciente. |
| `LineControl` | 2 | Espera una muestra, aplica PID o recuperacion y controla motores/LEDs. |
| `LineReport` | 1 | Imprime el ultimo diagnostico cada 200 ms. Se puede desactivar. |

<<<<<<< Updated upstream
1. Lee la distancia del sensor ultrasonico.
2. Si hay un obstaculo, ejecuta la secuencia de la garra.
3. Si no hay obstaculo, calcula el error de posicion de la linea.
4. Aplica la correccion proporcional, integral y derivativa a los motores.
5. Actualiza los LEDs de giro y espera 30 ms.

Los parametros del controlador estan en `components/src/robot.h`: velocidad base `170`, ganancias `0.12`, `0.002` y `0.35`, y limite de obstaculo de `15` cm.
=======
El muestreo solicita 30 ms usando `vTaskDelayUntil`, redondeados hacia arriba al tick disponible: dos ticks, nominalmente 32 ms, con la configuracion AVR probada. El controlador espera muestras mediante `xQueueReceive`; el diagnostico no bloquea al controlador. Las colas copian estructuras completas, evitando lecturas parciales entre tareas.

La posicion va de -1000 (izquierda) a +1000 (derecha). La consigna es cero. Con correccion negativa, la rueda izquierda disminuye su velocidad; con correccion positiva, disminuye la derecha. Esto supone A0 a la izquierda, A1 al centro y A2 a la derecha, mirando en el sentido de avance.

Los valores iniciales estan en `components/src/tasks/line_config.h`; se pueden sobrescribir en `line_control` dentro del sketch antes de llamar a `start()`:

| Parametro | Inicial |
| --- | --- |
| Velocidad base | 90 PWM |
| Correccion maxima | 90 PWM |
| Kp / Ki / Kd | 0.06 / 0 / 0 |
| Periodo solicitado | 30 ms |
| Tiempo sin muestras antes de detener | 150 ms, redondeado al tick |
| Velocidad de busqueda | 70 PWM |
| Cambio de sentido de busqueda | 300 ms |
| Limite de busqueda | 2000 ms |

El arranque rechaza configuraciones que permitan velocidades fuera de 0..255 en seguimiento. Si falla la creacion de una tarea o cola, limpia los recursos creados y mantiene los motores detenidos. `start()` devuelve `false` ante esos errores.

Al perder la linea, el controlador pausa el PID y gira sobre el lugar, primero hacia el ultimo lado conocido (izquierda si todavia no hay historial), alternando cada 300 ms. A los dos segundos se detiene hasta que vuelva a ver una linea no ambigua. Al recuperarla reinicia el historial del PID para evitar un salto derivativo por la perdida. Una muestra invalida o antigua tambien detiene el robot.

Los LEDs indican el sentido de la correccion durante el seguimiento; permanecen apagados durante busqueda y detencion. No representan una decision de ruta en cruces: esa logica sigue pendiente de una regla de recorrido.

## Calibrar los sensores

1. En `seguidor_de_linea.ino`, cambia `line_control.calibration_mode` a `true` y deja `diagnostics` activado. Carga el programa: los motores permanecen detenidos y se imprimen las lecturas de los tres sensores.
2. Con los sensores a la altura de trabajo, coloca los tres sobre el fondo y anota valores estables de `raw0`, `raw1`, `raw2`. Repite sobre una superficie negra suficientemente amplia para cubrir los tres. Usa varias muestras para comprobar la variacion.
3. Copia los valores medidos a `line_background` y `line_black`, respetando izquierda, centro y derecha, y activa `use_line_calibration`. No uses los ceros iniciales como mediciones.
4. Se exige una diferencia de al menos 50 unidades ADC entre negro y fondo en cada sensor. Si falla, revisa altura, conexion e iluminacion; el robot no inicia con una calibracion rechazada.
5. La normalizacion convierte fondo a 0 y negro a 1000 para cada sensor, incluso si negro produce una lectura menor. El umbral de deteccion calibrado es 500. Las lecturas inferiores al umbral no aportan peso a la posicion.
6. Cambia `calibration_mode` a `false`, compila y carga nuevamente para probar el seguimiento.

La calibracion queda en el sketch, no en EEPROM. Mientras `use_line_calibration` sea `false`, se conservan los umbrales originales `{110, 120, 100}` y la polaridad `black_is_high`. Esos valores aun requieren confirmacion sobre tu pista.

## Ajustar el movimiento

Comienza con `Kp=0.06`, `Ki=0`, `Kd=0` y velocidad base 90, siempre que ambas ruedas se muevan de forma estable con ese PWM. Verifica primero que al poner negro bajo el sensor izquierdo disminuya la rueda izquierda, y lo mismo a la derecha.

Aumenta Kp gradualmente si la correccion llega tarde; reducelo si zigzaguea. Despues agrega Kd desde un valor pequeno para amortiguar las oscilaciones. No reutilices el anterior `Kd=0.80`: `PID_v1` escala la derivada por el periodo en segundos y la posicion esta en unidades de hasta 1000. Mantener Ki en cero inicialmente evita acumular correccion al detener o saturar las ruedas. Ajusta un parametro por prueba y sube la velocidad solamente cuando complete rectas y curvas.

Si un motor necesita mas PWM que el otro para avanzar, mide primero esa diferencia. Las ganancias finales y el PWM minimo util no se pueden obtener solo con el codigo o con las lecturas historicas sin identificar negro y fondo.

## Diagnostico y pruebas

El monitor serie genera:

```text
ms,raw0,raw1,raw2,error,found,state,correction,left_pwm,right_pwm
```

`state`: 0 detenido, 1 seguimiento, 2 busqueda, 3 lectura ambigua, 4 calibracion, 5 fallo o falta de muestras. En busqueda los PWM son firmados para indicar el giro; durante seguimiento estan en 0..255. `found` indica deteccion, no autorizacion de movimiento: el estado tambien debe permitirlo.

Para comprobar la logica sin hardware:

```bash
python3 tests/run_tests.py
```

Las pruebas cubren una lectura por sensor, posicion, calibracion normal/invertida, rechazo de poco contraste, busqueda alternada, vencimiento del tiempo, desbordamiento del reloj y comandos de motores. Requieren Python 3 y un compilador C++.

Para compilar con las bibliotecas instaladas y sin crear un enlace simbolico:

```bash
arduino-cli compile --fqbn arduino:avr:mega --library ./components .
```

La compilacion y las pruebas de escritorio no verifican el cableado, la adherencia, el PWM minimo ni la trayectoria fisica. Primero prueba con ruedas elevadas; luego valida rectas, curvas y recuperacion sobre la pista.
>>>>>>> Stashed changes

## Flujo de desarrollo

1. Crea una rama para cada cambio.
2. Ejecuta el script de enlace una vez y conserva `components/` como fuente de verdad; no edites la copia instalada en el sketchbook.
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
- Implementar reglas de cruces/meta y arbitraje de movimientos antes de activar la tarea de obstaculos.
- Añadir pruebas para la deteccion de color.

## Colaborar

Para proponer un cambio, describe el problema, la solucion, la placa utilizada y como se verifico en hardware. Evita mezclar cambios de cableado, ajustes PID y refactorizaciones en el mismo pull request salvo que sean inseparables.
