# Seguidor de linea

Software para un robot movil que sigue una linea, detecta obstaculos y puede recoger/soltar objetos con una garra.

El sketch principal ensambla los componentes del robot. La implementacion reutilizable esta en la biblioteca local `RobotComponents`, dentro de `components/`.

## Estado actual

- El seguimiento de linea usa un controlador PID sencillo.
- La deteccion de obstaculos se hace con un sensor ultrasonico a menos de 15 cm.
- Ante un obstaculo, el robot abre la garra, avanza, cierra la garra, retrocede y suelta el objeto.
- `read_rgb_sensor()` es un stub: actualmente siempre devuelve `{0, 0, 0}`. Hay que conectarlo al sensor RGB real antes de usar `ColorSensor`.
- No hay una suite automatizada de pruebas; la validacion principal se hace compilando y probando el robot en hardware.

## Estructura

```text
seguidor_de_linea.ino       Sketch principal y configuracion del robot
components/
  library.properties        Metadatos y dependencias de la biblioteca
  src/
    RobotComponents.h       Cabecera publica de la biblioteca
    robot.h                 Logica del controlador y tarea FreeRTOS
    movement.h              Motores y direccion
    line_sensors.h          Sensores de linea y calculo del error
    color_sensor.h          Lectura y comparacion de color
    claw.h                  Garra controlada por servo
    buzzer.h                Zumbador
    led_array.h             LEDs de giro
    ultrasonic_sensor.h      Medicion de distancia
    component.h             Interfaz comun de componentes
scripts/
  link-library.ps1          Enlace simbolico para Windows
  link-library.sh           Enlace simbolico para Linux/macOS
```

## Requisitos

- Arduino IDE o Arduino CLI.
- La placa Arduino compatible con el montaje y seleccionada en el IDE.
- Bibliotecas Arduino:
  - `Servo`
  - `Arduino_FreeRTOS`
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
6. Abre el monitor serie a `9600` baudios para observar mensajes de depuracion cuando se agreguen.

Con Arduino CLI, el flujo equivalente es:

```bash
arduino-cli compile --fqbn <fabricante:arquitectura:placa> .
arduino-cli upload -p <puerto> --fqbn <fabricante:arquitectura:placa> .
```

Sustituye `<fabricante:arquitectura:placa>` y `<puerto>` por los valores de tu instalacion.

## Como funciona

`setup()` inicializa el puerto serie, configura todos los componentes y arranca el controlador. `loop()` queda vacio porque `Robot::start()` crea una tarea de `Arduino_FreeRTOS`.

En cada iteracion de la tarea:

1. Lee la distancia del sensor ultrasonico.
2. Si hay un obstaculo, ejecuta la secuencia de la garra.
3. Si no hay obstaculo, calcula el error de posicion de la linea.
4. Aplica la correccion proporcional, integral y derivativa a los motores.
5. Actualiza los LEDs de giro y espera 30 ms.

Los parametros del controlador estan en `components/src/robot.h`: velocidad base `170`, ganancias `0.12`, `0.002` y `0.35`, y limite de obstaculo de `15` cm.

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
- Separar la logica de control de hardware para poder probarla sin el robot.
- Añadir mensajes de diagnostico por puerto serie.
- Añadir pruebas para el calculo del error de los sensores de linea y la deteccion de color.

## Colaborar

Para proponer un cambio, describe el problema, la solucion, la placa utilizada y como se verifico en hardware. Evita mezclar cambios de cableado, ajustes PID y refactorizaciones en el mismo pull request salvo que sean inseparables.
