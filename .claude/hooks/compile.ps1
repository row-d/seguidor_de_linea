# Hook PostToolUse: compila el sketch para Arduino Mega cuando Claude edita
# un archivo de codigo (.ino, .h, .hpp, .c, .cpp). Si falla, devuelve exit 2
# para que Claude reciba los errores del compilador.

$inputJson = [Console]::In.ReadToEnd()

$filePath = $null
try {
    $hookInput = $inputJson | ConvertFrom-Json
    $filePath = $hookInput.tool_input.file_path
}
catch {
    # Respaldo si el JSON no se puede analizar: buscar file_path con una expresion regular.
    if ($inputJson -match '"file_path"\s*:\s*"([^"]*)"') {
        $filePath = $Matches[1]
    }
}

# Solo compilar si el archivo editado es codigo fuente.
if (-not $filePath -or $filePath -notmatch '\.(ino|h|hpp|c|cpp)$') {
    exit 0
}

$projectDir = if ($env:CLAUDE_PROJECT_DIR) { $env:CLAUDE_PROJECT_DIR } else { Join-Path $PSScriptRoot '..\..' }
Set-Location -LiteralPath $projectDir

# ForEach-Object convierte la salida de error en texto plano (sin el formato de ErrorRecord de PowerShell 5.1).
$output = & arduino-cli compile --no-color --fqbn arduino:avr:mega --library firmware . 2>&1 | ForEach-Object { "$_" } | Out-String
$status = $LASTEXITCODE

if ($status -ne 0) {
    $lines = $output -split "`r?`n"
    [Console]::Error.WriteLine("La compilacion para Arduino Mega fallo (arduino-cli compile --fqbn arduino:avr:mega --library firmware .):")
    [Console]::Error.WriteLine((($lines | Select-Object -Last 40) -join "`n"))
    exit 2
}

# Exito: mostrar el resumen de uso de memoria.
($output.Trim() -split "`r?`n" | Select-Object -Last 2) -join "`n"
exit 0
