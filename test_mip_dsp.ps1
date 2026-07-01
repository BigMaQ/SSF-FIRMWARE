# MIP DSP Stress Test – PowerShell, kein GUI
# Sendet DSP1..DSP3 an COM3, startet mit 1s Takt, wird immer schneller
param(
    [string]$ComPort = "COM3",
    [int]$Baud = 115200
)

$ErrorActionPreference = "Stop"

# COM-Port Objekt erstellen
try {
    $port = New-Object System.IO.Ports.SerialPort $ComPort, $Baud, None, 8, One
    $port.ReadTimeout = 200
    $port.WriteTimeout = 500
    $port.Open()
    Start-Sleep -Milliseconds 1500  # Arduino Reset abwarten
    Write-Host "COM3 geöffnet. STRG+C zum Abbrechen." -ForegroundColor Green
} catch {
    Write-Host "FEHLER: $ComPort nicht verfügbar: $_" -ForegroundColor Red
    exit 1
}

function Send-DSP {
    param([string]$Cmds)
    $bytes = [System.Text.Encoding]::ASCII.GetBytes($Cmds)
    $port.Write($bytes, 0, $bytes.Length)
}

function Format-Time {
    param([long]$ElapsedMs)
    $cs = [int](($ElapsedMs / 10) % 100)
    $totalSec = [int]($ElapsedMs / 1000)
    $mm = [int](($totalSec / 60) % 100)
    $ss = [int]($totalSec % 60)
    return ("{0:D2}.{1:D2}.{2:D2}" -f [int]$mm, [int]$ss, [int]$cs)
}

$v1 = 0
$v3 = 0
$interval = 1.0
$startTime = [DateTime]::UtcNow
$cycle = 0

try {
    while ($true) {
        $elapsedMs = [long]([DateTime]::UtcNow - $startTime).TotalMilliseconds
        $t2 = Format-Time $elapsedMs

        $cmd = "DSP1:$v1;DSP2:$t2;DSP3:$v3;`r`n"
        Send-DSP $cmd

        $cycle++
        $v1 = ($v1 + 1) % 10000
        $v3 = ($v3 + 1) % 10000

        if ($cycle % 10 -eq 0) {
            Write-Host "Zyklus $($cycle.ToString().PadLeft(6)) | DSP1=$($v1.ToString().PadLeft(4,'0'))  DSP2=$t2  DSP3=$($v3.ToString().PadLeft(4,'0'))  Takt=$("{0:F3}" -f $interval)s" -NoNewline
            Write-Host "`r" -NoNewline
        }

        Start-Sleep -Seconds $interval

        if ($cycle % 10 -eq 0 -and $interval -gt 0.50) {
            $interval = [Math]::Max(0.50, $interval * 0.7)
            Write-Host "`n>> Neuer Takt: $("{0:F3}" -f $interval)s (Zyklus $cycle)" -ForegroundColor Yellow
        }
    }
} catch [System.Management.Automation.BreakException] {
    Write-Host "`n`nABGEBROCHEN nach $cycle Zyklen." -ForegroundColor Cyan
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Host "$ComPort geschlossen."
    }
}
