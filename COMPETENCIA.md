# Competencia

## Objetivo

El robot debera completar el recorrido siguiendo una linea negra hasta llegar a la linea horizontal de meta.

## Reglas de funcionamiento

1. El robot debera iniciar el recorrido desde cualquier angulo. La linea podra encontrarse al frente, a la izquierda, a la derecha o detras del robot.
2. El robot debera buscar la linea y orientarse hacia ella antes de comenzar el seguimiento.
3. Mientras siga la linea, debera mantener los sensores sobre la superficie negra y ajustar la velocidad de los motores para permanecer centrado.
4. El robot no debera retroceder durante el seguimiento normal de la linea.
5. Al detectar una interseccion, debera elegir la ruta correspondiente y continuar el recorrido.
6. Si el robot toma la ruta de la izquierda, debera encender el LED 0.
7. Si el robot toma la ruta de la derecha, debera encender el LED 1.
8. Al detectar un objeto en la linea, debera detenerse o reducir la velocidad, accionar la garra y quitar el objeto del camino.
9. Despues de retirar el objeto, debera liberar la garra y continuar siguiendo la linea.
10. Cuando el objeto haya sido derribado o retirado, el robot debera emitir un beep.
11. El robot debera detenerse al detectar la linea horizontal de llegada.
12. Ante una lectura invalida de los sensores o una perdida de la linea, el robot debera detenerse o ejecutar una busqueda controlada sin alejarse de la zona de recorrido.

## Indicadores

- LED 0 encendido: ruta izquierda.
- LED 1 encendido: ruta derecha.
- Ambos LEDs apagados: avance recto, busqueda o robot detenido.
- Beep: objeto retirado correctamente.
