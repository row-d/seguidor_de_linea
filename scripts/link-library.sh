#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source="$(cd -- "$script_dir/../components" && pwd)"
sketchbook="${ARDUINO_SKETCHBOOK:-$HOME/Arduino}"
destination="$sketchbook/libraries/RobotComponents"

mkdir -p "$(dirname -- "$destination")"

if [[ -e "$destination" || -L "$destination" ]]; then
    if [[ "${1:-}" != "--force" ]]; then
        printf 'El destino ya existe: %s\nUsa --force solo para reemplazar un enlace simbolico existente.\n' "$destination" >&2
        exit 1
    fi

    if [[ ! -L "$destination" ]]; then
        printf 'El destino existe y no es un enlace simbolico; no se eliminara: %s\n' "$destination" >&2
        exit 1
    fi

    rm "$destination"
fi

ln -s "$source" "$destination"
printf 'Biblioteca enlazada: %s -> %s\n' "$destination" "$source"
