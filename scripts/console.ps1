# Consola del robot: muestra todo lo que envia por Bluetooth y le manda comandos con
# una tecla. Reintenta la conexion si falla o si se corta.
#
#   g  empezar otro recorrido (cuando el anterior ya termino o se detuvo con x)
#   x  detener el recorrido en marcha
#   c  calibrar los sensores de linea
#   p  ver o cambiar las ganancias del PID (Kp Ki Kd); quedan guardadas en el robot
#   r  empezar un registro y guardarlo en logs\run-AAAAMMDD-HHMMSS.csv
#   s  detener el registro y cerrar el CSV
#   v  mostrar u ocultar las filas de telemetria (se guardan igual)
#   h  ayuda
#   q  salir
param(
    [string]$Port = "COM3",
    [int]$BaudRate = 9600,
    [int]$RetryDelaySeconds = 2,
    [string]$LogDirectory = "logs"
)

# Deben coincidir con las constantes de Logger (firmware/src/components/logger.h).
$runCommand = "g"
$haltCommand = "x"
$gainsCommand = "p"
$calibrateCommand = "c"
$recordCommand = "r"
$stopCommand = "s"

$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($Port -notin $availablePorts) {
    $availablePortList = if ($availablePorts.Count -gt 0) { $availablePorts -join ", " } else { "ninguno" }
    Write-Error "El puerto '$Port' no existe. Puertos disponibles: $availablePortList."
    exit 1
}
if ([Console]::IsInputRedirected) {
    Write-Error "Esta consola necesita un teclado: ejecutala en una ventana de PowerShell."
    exit 1
}

function Show-Help {
    Write-Host ""
    Write-Host "  g  empezar otro recorrido        x  detener el recorrido" -ForegroundColor White
    Write-Host "  c  calibrar sensores             p  ver/cambiar Kp Ki Kd" -ForegroundColor White
    Write-Host "  r  grabar registro en CSV        s  detener registro" -ForegroundColor White
    Write-Host "  v  mostrar/ocultar telemetria    h  ayuda    q  salir" -ForegroundColor White
    Write-Host ""
}

# Lee una tecla pendiente sin bloquear. Devuelve $null si no hay ninguna.
function Read-PendingKey {
    if (-not [Console]::KeyAvailable) {
        return $null
    }
    return [Console]::ReadKey($true).KeyChar.ToString().ToLower()
}

# Abre el puerto, reintentando. Devuelve $null si el usuario pulsa q mientras espera.
function Connect-Robot {
    $attempt = 0
    while ($true) {
        $attempt++
        $candidate = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
        # Lo que llega mientras se escriben las ganancias espera aqui sin perderse.
        $candidate.ReadBufferSize = 65536
        try {
            Write-Host "Conectando con $Port (intento $attempt, q para salir)..." -ForegroundColor DarkGray
            $candidate.Open()
            $candidate.DiscardInBuffer()
            Write-Host "Conectado." -ForegroundColor Green
            return $candidate
        }
        catch {
            $candidate.Dispose()
            $reason = if ($_.Exception.InnerException) { $_.Exception.InnerException.Message } else { $_.Exception.Message }
            Write-Warning "No se pudo conectar: $($reason.Trim())"
            if ($attempt -eq 1) {
                Write-Host ("Revisa que el robot este encendido (LED del HC-05 parpadeando rapido) y que " +
                    "ningun otro programa este conectado al modulo.")
            }
        }

        $waitUntil = (Get-Date).AddSeconds($RetryDelaySeconds)
        while ((Get-Date) -lt $waitUntil) {
            if ((Read-PendingKey) -eq "q") {
                return $null
            }
            Start-Sleep -Milliseconds 100
        }
    }
}

function Write-RobotLine([string]$line) {
    $kind = ($line -split ",")[0]
    switch ($kind) {
        "t" { if ($script:showTelemetry) { Write-Host $line -ForegroundColor DarkGray } }
        "event" { Write-Host $line -ForegroundColor Yellow }
        "calibration" { Write-Host $line -ForegroundColor Cyan }
        "run" { Write-Host $line -ForegroundColor Green }
        "pid" { Write-Host $line -ForegroundColor Magenta }
        "recording" { Write-Host $line -ForegroundColor Green }
        default { Write-Host $line }
    }
}

function Close-Recording {
    if ($null -ne $script:writer) {
        $script:writer.Dispose()
        $script:writer = $null
        Write-Host "Registro guardado: $script:recordingFile" -ForegroundColor Green
    }
}

$script:showTelemetry = $true
$script:writer = $null
$script:recordingFile = ""
$connection = $null
$pending = ""

try {
    $connection = Connect-Robot
    if ($null -ne $connection) {
        Show-Help
    }

    while ($null -ne $connection) {
        # --- Lo que envia el robot ---
        try {
            if ($connection.BytesToRead -gt 0) {
                $pending += $connection.ReadExisting()
            }
        }
        catch {
            Write-Warning "Se perdio la conexion: $($_.Exception.Message.Trim())"
            $connection.Dispose()
            $pending = ""
            $connection = Connect-Robot
            if ($null -eq $connection) {
                break
            }
            if ($null -ne $script:writer) {
                # El robot sigue registrando, pero por si se reinicio se lo pide de nuevo.
                $script:writer.WriteLine("recording,,reconnected")
                $connection.Write($recordCommand)
            }
            continue
        }

        while (($newline = $pending.IndexOf("`n")) -ge 0) {
            $line = $pending.Substring(0, $newline).Trim()
            $pending = $pending.Substring($newline + 1)
            if (-not $line) {
                continue
            }
            if ($null -ne $script:writer) {
                $script:writer.WriteLine($line)
            }
            Write-RobotLine $line
            if ($line -like "recording,*,stop") {
                Close-Recording
            }
        }

        # --- Lo que pide el usuario ---
        $key = Read-PendingKey
        try {
            if ($null -ne $key) { switch ($key) {
                "g" {
                    Write-Host "> recorrido" -ForegroundColor White
                    $connection.Write($runCommand)
                }
                "x" {
                    Write-Host "> detener recorrido" -ForegroundColor White
                    $connection.Write($haltCommand)
                }
                "p" {
                    Write-Host "> ganancias PID. Escribe Kp Ki Kd separados por espacios (p. ej. 0.25 0 0.005)," -ForegroundColor White
                    Write-Host "  o solo Enter para ver las actuales." -ForegroundColor White
                    $typed = (Read-Host "  Kp Ki Kd").Trim()
                    if (-not $typed) {
                        $connection.Write("$gainsCommand`n")
                    }
                    else {
                        $culture = [System.Globalization.CultureInfo]::InvariantCulture
                        # Coma decimal aceptada: los valores se separan con espacios.
                        $parts = @($typed -split "\s+" | ForEach-Object { $_.Replace(",", ".") })
                        $numbers = @()
                        foreach ($part in $parts) {
                            $number = 0.0
                            if ([double]::TryParse($part, [System.Globalization.NumberStyles]::Float, $culture, [ref]$number)) {
                                $numbers += $number.ToString("0.######", $culture)
                            }
                        }
                        if ($parts.Count -eq 3 -and $numbers.Count -eq 3) {
                            $connection.Write("$gainsCommand$($numbers -join ',')`n")
                        }
                        else {
                            Write-Warning "Se esperaban 3 numeros: Kp Ki Kd. No se envio nada."
                        }
                    }
                }
                "c" {
                    Write-Host "> calibrar: cuando aparezca 'calibration,start', pasa los sensores sobre el fondo y la linea durante 10 segundos" -ForegroundColor White
                    $connection.Write($calibrateCommand)
                }
                "r" {
                    if ($null -eq $script:writer) {
                        if (-not (Test-Path $LogDirectory)) {
                            New-Item -ItemType Directory -Path $LogDirectory -Force | Out-Null
                        }
                        $script:recordingFile = Join-Path $LogDirectory ("run-{0}.csv" -f (Get-Date -Format "yyyyMMdd-HHmmss"))
                        $script:writer = New-Object System.IO.StreamWriter $script:recordingFile, $false, ([System.Text.UTF8Encoding]::new($false))
                        $script:writer.AutoFlush = $true
                        Write-Host "> registro en $script:recordingFile" -ForegroundColor White
                    }
                    $connection.Write($recordCommand)
                }
                "s" {
                    Write-Host "> detener registro" -ForegroundColor White
                    $connection.Write($stopCommand)
                }
                "v" {
                    $script:showTelemetry = -not $script:showTelemetry
                    $state = if ($script:showTelemetry) { "visible" } else { "oculta" }
                    Write-Host "> telemetria $state" -ForegroundColor White
                }
                "h" { Show-Help }
                "?" { Show-Help }
                "q" {
                    $connection.Write($stopCommand)
                    $connection.Close()
                    $connection.Dispose()
                    $connection = $null
                }
                default { Write-Host "> tecla '$key' sin comando (h para ayuda)" -ForegroundColor DarkGray }
            } }
        }
        catch {
            Write-Warning "No se pudo enviar el comando: $($_.Exception.Message.Trim())"
        }

        Start-Sleep -Milliseconds 20
    }
}
finally {
    # Tambien se ejecuta con Ctrl+C: el robot deja de enviar telemetria.
    if ($null -ne $connection) {
        if ($connection.IsOpen) {
            try { $connection.Write($stopCommand) } catch {}
            try { $connection.Close() } catch {}
        }
        $connection.Dispose()
    }
    Close-Recording
    Write-Host "Consola cerrada."
}
