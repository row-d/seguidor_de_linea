[CmdletBinding()]
param(
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

$source = Join-Path $PSScriptRoot '..\components' | Resolve-Path
$sketchbook = if ($env:ARDUINO_SKETCHBOOK) {
    $env:ARDUINO_SKETCHBOOK
} else {
    Join-Path $env:USERPROFILE 'Documents\Arduino'
}
$destination = Join-Path $sketchbook 'libraries\RobotComponents'
$destinationParent = Split-Path $destination -Parent

New-Item -ItemType Directory -Path $destinationParent -Force | Out-Null

if (Test-Path -LiteralPath $destination) {
    $existing = Get-Item -LiteralPath $destination -Force
    $isLink = ($existing.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0

    if (-not $Force) {
        throw "El destino ya existe: $destination. Usa -Force solo si es un enlace existente que deseas reemplazar."
    }

    if (-not $isLink) {
        throw "El destino existe y no es un enlace simbolico; no se eliminara: $destination"
    }

    Remove-Item -LiteralPath $destination -Force
}

New-Item -ItemType SymbolicLink -Path $destination -Target $source | Out-Null
Write-Host "Biblioteca enlazada: $destination -> $source"
