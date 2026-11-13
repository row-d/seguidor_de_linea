# Consola del robot: muestra todo lo que envia por Bluetooth y le manda comandos con
# una tecla. Reintenta la conexion si falla o si se corta.
#
#   g  empezar otro recorrido (cuando el anterior ya termino o se detuvo con x)
#   x  detener el recorrido en marcha
#   t  activar o desactivar el giro tipo tanque (sobre su eje); queda guardado en el robot
#   c  calibrar los sensores de linea
#   p  ver o cambiar las ganancias del PID (Kp Ki Kd); quedan guardadas en el robot
#   b  ver o cambiar las velocidades (base, giro, minima); quedan guardadas en el robot
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
$tankCommand = "t"
$gainsCommand = "p"
$speedsCommand = "b"
$calibrateCommand = "c"
$recordCommand = "r"
$stopCommand = "s"

# Con el enlace Bluetooth caido, escribir en el puerto se queda esperando: pasado este
# tiempo se da la conexion por perdida.
$writeTimeoutMs = 500
# Tiempo maximo para cerrar el puerto. Con el enlace caido, cerrar tambien puede colgarse.
$closeTimeoutMs = 1500

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

# Abrir y cerrar un puerto Bluetooth puede tardar muchos segundos o no volver nunca, y
# PowerShell no puede interrumpir esa llamada: ni Ctrl+C ni las teclas responden. Por eso
# se hacen en otro hilo y la consola solo espera el resultado, sin quedarse bloqueada.
if (-not ("RobotConsole.PortCall" -as [type])) {
    Add-Type -TypeDefinition @"
using System;
using System.Reflection;
using System.Threading.Tasks;

namespace RobotConsole
{
    public static class PortCall
    {
        // Llama a un metodo sin argumentos de target en otro hilo.
        // El resultado es null si termino bien, o el mensaje del error.
        public static Task<string> Start(object target, string method)
        {
            return Task.Factory.StartNew<string>(() =>
            {
                try
                {
                    target.GetType().GetMethod(method, Type.EmptyTypes).Invoke(target, null);
                    return null;
                }
                catch (TargetInvocationException error)
                {
                    return (error.InnerException ?? error).Message;
                }
                catch (Exception error)
                {
                    return error.Message;
                }
            }, TaskCreationOptions.LongRunning);
        }

        // Para una llamada que se dejo de esperar: cuando termine, cierra el puerto
        // para que no quede abierto sin que nadie lo use.
        public static void CloseWhenDone(Task pending, object target)
        {
            pending.ContinueWith(_ =>
            {
                try
                {
                    target.GetType().GetMethod("Close", Type.EmptyTypes).Invoke(target, null);
                }
                catch (Exception)
                {
                }
            });
        }
    }
}
"@
}

function Show-Help {
    Write-Host ""
    Write-Host "  g  empezar otro recorrido        x  detener el recorrido" -ForegroundColor White
    Write-Host "  c  calibrar sensores             p  ver/cambiar Kp Ki Kd" -ForegroundColor White
    Write-Host "  b  ver/cambiar velocidades (base, giro, minima)" -ForegroundColor White
    Write-Host "  t  giro tipo tanque: activar/desactivar" -ForegroundColor White
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

# Cierra el puerto sin quedarse esperando si el enlace esta caido.
function Close-Port($serialPort) {
    if ($null -eq $serialPort) {
        return
    }
    $closing = [RobotConsole.PortCall]::Start($serialPort, "Close")
    [void]$closing.Wait($closeTimeoutMs)
}

# Abre el puerto, reintentando. Devuelve $null si el usuario pulsa q mientras espera.
function Connect-Robot {
    $attempt = 0
    while ($true) {
        $attempt++
        $candidate = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
        # Lo que llega mientras se escriben las ganancias espera aqui sin perderse.
        $candidate.ReadBufferSize = 65536
        $candidate.WriteTimeout = $writeTimeoutMs

        Write-Host "Conectando con $Port (intento $attempt, q para salir)..." -ForegroundColor DarkGray
        $opening = [RobotConsole.PortCall]::Start($candidate, "Open")
        # Variable de script: si Ctrl+C corta la espera, el bloque finally la cierra.
        $script:openingPort = $candidate
        $script:openingTask = $opening
        while (-not $opening.IsCompleted) {
            if ((Read-PendingKey) -eq "q") {
                return $null
            }
            Start-Sleep -Milliseconds 100
        }
        $script:openingPort = $null
        $script:openingTask = $null

        $reason = $opening.Result
        if ($null -eq $reason) {
            try { $candidate.DiscardInBuffer() } catch {}
            Write-Host "Conectado." -ForegroundColor Green
            return $candidate
        }

        Write-Warning "No se pudo conectar: $($reason.Trim())"
        if ($attempt -eq 1) {
            Write-Host ("Revisa que el robot este encendido (LED del HC-05 parpadeando rapido) y que " +
                "ningun otro programa este conectado al modulo.")
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

# Envia un comando al robot. Si no sale a tiempo, da la conexion por perdida.
function Send-Command([string]$text) {
    try {
        $script:connection.Write($text)
    }
    catch {
        Write-Warning "No se pudo enviar el comando: $($_.Exception.Message.Trim())"
        $script:linkLost = $true
    }
}

# Pide 3 numeros por teclado y los envia como "<letra><a>,<b>,<c>". Con Enter vacio
# envia solo la letra, que consulta los valores en uso.
function Send-Values([string]$command, [string]$names) {
    $typed = (Read-Host "  $names").Trim()
    if (-not $typed) {
        Send-Command "$command`n"
        return
    }

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
        Send-Command "$command$($numbers -join ',')`n"
    }
    else {
        Write-Warning "Se esperaban 3 numeros: $names. No se envio nada."
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
        "speeds" { Write-Host $line -ForegroundColor Magenta }
        "tank" { Write-Host $line -ForegroundColor Magenta }
        "last_stop" { Write-Host $line -ForegroundColor Yellow }
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
$script:connection = $null
$script:linkLost = $false
$script:openingPort = $null
$script:openingTask = $null
$pending = ""
$quit = $false

try {
    $script:connection = Connect-Robot
    if ($null -ne $script:connection) {
        Show-Help
    }

    while ($null -ne $script:connection -and -not $quit) {
        # --- Lo que envia el robot ---
        try {
            if ($script:connection.BytesToRead -gt 0) {
                $pending += $script:connection.ReadExisting()
            }
        }
        catch {
            Write-Warning "Se perdio la conexion: $($_.Exception.Message.Trim())"
            $script:linkLost = $true
        }

        if ($script:linkLost) {
            $script:linkLost = $false
            $pending = ""
            Close-Port $script:connection
            $script:connection = Connect-Robot
            if ($null -eq $script:connection) {
                break
            }
            if ($null -ne $script:writer) {
                # El robot sigue registrando, pero por si se reinicio se lo pide de nuevo.
                $script:writer.WriteLine("recording,,reconnected")
                Send-Command $recordCommand
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
        if ($null -ne $key) { switch ($key) {
            "g" {
                Write-Host "> recorrido" -ForegroundColor White
                Send-Command $runCommand
            }
            "x" {
                Write-Host "> detener recorrido" -ForegroundColor White
                Send-Command $haltCommand
            }
            "t" {
                Write-Host "> cambiar giro tipo tanque" -ForegroundColor White
                Send-Command $tankCommand
            }
            "p" {
                Write-Host "> ganancias PID. Escribe Kp Ki Kd separados por espacios (p. ej. 0.25 0 0.005)," -ForegroundColor White
                Write-Host "  o solo Enter para ver las actuales." -ForegroundColor White
                Send-Values $gainsCommand "Kp Ki Kd"
            }
            "b" {
                Write-Host "> velocidades (0 a 255). Escribe base, giro y minima separadas por espacios (p. ej. 150 230 110)," -ForegroundColor White
                Write-Host "  o solo Enter para ver las actuales." -ForegroundColor White
                Send-Values $speedsCommand "Base Giro Minima"
            }
            "c" {
                Write-Host "> calibrar: cuando aparezca 'calibration,start', pasa a mano los sensores sobre el fondo y la linea durante 10 segundos" -ForegroundColor White
                Send-Command $calibrateCommand
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
                Send-Command $recordCommand
            }
            "s" {
                Write-Host "> detener registro" -ForegroundColor White
                Send-Command $stopCommand
            }
            "v" {
                $script:showTelemetry = -not $script:showTelemetry
                $state = if ($script:showTelemetry) { "visible" } else { "oculta" }
                Write-Host "> telemetria $state" -ForegroundColor White
            }
            "h" { Show-Help }
            "?" { Show-Help }
            "q" { $quit = $true }
            default { Write-Host "> tecla '$key' sin comando (h para ayuda)" -ForegroundColor DarkGray }
        } }

        Start-Sleep -Milliseconds 20
    }
}
finally {
    # Tambien se ejecuta con Ctrl+C. Nada de lo que sigue espera mas de unos segundos,
    # aunque el enlace Bluetooth este caido.
    if ($null -ne $script:openingTask) {
        # Se salio a mitad de un intento de conexion: se cierra solo cuando termine.
        [RobotConsole.PortCall]::CloseWhenDone($script:openingTask, $script:openingPort)
    }
    if ($null -ne $script:connection) {
        # El robot deja de enviar telemetria.
        try { $script:connection.Write($stopCommand) } catch {}
        Close-Port $script:connection
    }
    Close-Recording
    Write-Host "Consola cerrada."
}
