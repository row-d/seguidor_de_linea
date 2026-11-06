#!/usr/bin/env bash
# Hook PostToolUse: compila el sketch para Arduino Mega cuando Claude edita
# un archivo de codigo (.ino, .h, .hpp, .c, .cpp). Si falla, devuelve exit 2
# para que Claude reciba los errores del compilador.

input="$(cat)"

# Solo compilar si el archivo editado es codigo fuente.
if ! printf '%s' "$input" | grep -Eq '"file_path"[[:space:]]*:[[:space:]]*"[^"]*\.(ino|h|hpp|c|cpp)"'; then
    exit 0
fi

cd "${CLAUDE_PROJECT_DIR:-$(dirname "$0")/../..}" || exit 0

output="$(arduino-cli compile --no-color --fqbn arduino:avr:mega --library firmware . 2>&1)"
status=$?

if [ $status -ne 0 ]; then
    {
        echo "La compilacion para Arduino Mega fallo (arduino-cli compile --fqbn arduino:avr:mega --library firmware .):"
        printf '%s\n' "$output" | tail -n 40
    } >&2
    exit 2
fi

# Exito: mostrar el resumen de uso de memoria.
printf '%s\n' "$output" | tail -n 2
exit 0
