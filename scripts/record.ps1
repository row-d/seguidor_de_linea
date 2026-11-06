# Pide al robot que empiece un registro, guarda lo que envia en un CSV y le pide
# que pare al terminar. Termina con Ctrl+C, al pasar -DurationSeconds o cuando el
# robot se detiene (finish, no_route, fork_missed, memory_full).
# Si no logra conectar, o si la conexion se corta a mitad del registro, reintenta.
param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$BaudRate = 9600,
    # 0: sin limite, hasta Ctrl+C o hasta que el robot se detenga.
    [int]$DurationSeconds = 0,
    # Por defecto un archivo nuevo por registro: logs\run-AAAAMMDD-HHMMSS.csv
    [string]$OutputFile = "",
    # Intentos de conexion seguidos antes de rendirse. 0: sin limite, hasta Ctrl+C.
    [int]$ConnectAttempts = 0,
    [int]$RetryDelaySeconds = 2
)

# Deben coincidir con Logger::START_COMMAND y Logger::STOP_COMMAND.
$startCommand = "r"
$stopCommand = "s"
$haltEvents = @("finish", "no_route", "fork_missed", "memory_full")
$startTimeoutSeconds = 3

if (-not $OutputFile) {
    $OutputFile = Join-Path "logs" ("run-{0}.csv" -f (Get-Date -Format "yyyyMMdd-HHmmss"))
}

$outputDirectory = Split-Path -Parent $OutputFile
if ($outputDirectory -and -not (Test-Path $outputDirectory)) {
    New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
}

$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($Port -notin $availablePorts) {
    $availablePortList = if ($availablePorts.Count -gt 0) { $availablePorts -join ", " } else { "ninguno" }
    Write-Error "El puerto '$Port' no existe. Puertos disponibles: $availablePortList."
    exit 1
}

# Abre el puerto y pide el registro. Devuelve $null si se agotan los intentos.
function Connect-Robot {
    $attempt = 0
    while ($ConnectAttempts -le 0 -or $attempt -lt $ConnectAttempts) {
        $attempt++
        $connection = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
        $connection.ReadTimeout = 500
        $connection.NewLine = "`n"
        try {
            Write-Host "Conectando con $Port (intento $attempt)..."
            $connection.Open()
            $connection.DiscardInBuffer()
            $connection.Write($startCommand)
            Write-Host "Conectado. Registro pedido."
            return $connection
        }
        catch {
            $connection.Dispose()
            $reason = if ($_.Exception.InnerException) { $_.Exception.InnerException.Message } else { $_.Exception.Message }
            Write-Warning "No se pudo conectar: $($reason.Trim())"
            if ($attempt -eq 1) {
                Write-Host ("Revisa que el robot este encendido (LED del HC-05 parpadeando rapido) y que " +
                    "ningun otro programa (monitor serie, celular) este conectado al modulo.")
            }
            Start-Sleep -Seconds $RetryDelaySeconds
        }
    }
    return $null
}

$serialPort = $null
$writer = $null

try {
    $serialPort = Connect-Robot
    if ($null -eq $serialPort) {
        Write-Error "No se pudo conectar con '$Port' despues de $ConnectAttempts intentos."
        exit 1
    }

    $writer = New-Object System.IO.StreamWriter $OutputFile, $false, ([System.Text.UTF8Encoding]::new($false))
    $writer.AutoFlush = $true
    Write-Host "Guardando en $OutputFile. Pulsa Ctrl+C para detener."

    $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    $requestedAt = $stopwatch.Elapsed.TotalSeconds
    $started = $false
    while ($DurationSeconds -le 0 -or $stopwatch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        if (-not $started -and $stopwatch.Elapsed.TotalSeconds - $requestedAt -ge $startTimeoutSeconds) {
            Write-Warning "El robot no confirmo el registro. Esta encendido y siguiendo la linea (calibration_mode = false)?"
            $started = $true
        }

        try {
            $line = $serialPort.ReadLine().Trim()
        }
        catch [System.TimeoutException] {
            continue
        }
        catch {
            # El enlace Bluetooth se corto: p. ej. el robot se alejo o se reinicio.
            Write-Warning "Se perdio la conexion: $($_.Exception.Message.Trim())"
            $serialPort.Dispose()
            $serialPort = Connect-Robot
            if ($null -eq $serialPort) {
                Write-Error "No se pudo reconectar con '$Port'."
                exit 1
            }
            $writer.WriteLine("recording,,reconnected")
            $requestedAt = $stopwatch.Elapsed.TotalSeconds
            $started = $false
            continue
        }
        if (-not $line) {
            continue
        }

        $writer.WriteLine($line)
        Write-Host $line

        if ($line -like "recording,*,start") {
            $started = $true
        }
        $fields = $line -split ","
        if ($fields[0] -eq "event" -and $fields.Count -ge 3 -and $fields[2] -in $haltEvents) {
            Write-Host "El robot se detuvo ($($fields[2]))."
            break
        }
    }
}
finally {
    # Tambien se ejecuta con Ctrl+C: el robot deja de enviar telemetria.
    if ($null -ne $serialPort) {
        if ($serialPort.IsOpen) {
            try { $serialPort.Write($stopCommand) } catch {}
            try { $serialPort.Close() } catch {}
        }
        $serialPort.Dispose()
    }
    if ($null -ne $writer) {
        $writer.Dispose()
        Write-Host "Registro terminado: $OutputFile"
    }
}
