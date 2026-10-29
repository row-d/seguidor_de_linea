param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$BaudRate = 9600,
    [int]$DurationSeconds = 30,
    [string]$OutputFile = "logs\line-sensors.csv"
)

$outputDirectory = Split-Path -Parent $OutputFile
if ($outputDirectory -and -not (Test-Path $outputDirectory)) {
    New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
}

$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($Port -notin $availablePorts) {
    $availablePortList = if ($availablePorts.Count -gt 0) {
        $availablePorts -join ", "
    }
    else {
        "ninguno"
    }
    Write-Error "El puerto '$Port' no existe. Puertos disponibles: $availablePortList."
    exit 1
}

$serialPort = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
$serialPort.ReadTimeout = 1000
$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()

try {
    $serialPort.Open()
    $writer = New-Object System.IO.StreamWriter $OutputFile, $false, ([System.Text.Encoding]::UTF8)
    $writer.AutoFlush = $true

    Write-Host "Guardando datos en $OutputFile. Pulsa Ctrl+C para detener."
    while ($stopwatch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        try {
            $line = $serialPort.ReadLine()
            if (-not [string]::IsNullOrWhiteSpace($line)) {
                $writer.WriteLine($line)
                Write-Host $line
            }
        }
        catch [System.TimeoutException] {
        }
    }
}
catch {
    Write-Error "No se pudo completar la captura en '$Port': $($_.Exception.Message)"
    exit 1
}
finally {
    if ($null -ne $writer) {
        $writer.Dispose()
    }
    if ($serialPort.IsOpen) {
        $serialPort.Close()
    }
    $serialPort.Dispose()
}

Write-Host "Captura terminada."
