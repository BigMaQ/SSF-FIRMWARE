# Stoppuhr + Uhrzeit + IN1-Reader auf COM3
# DSP1 = aktuelle Uhrzeit (HH.MM)
# DSP2 = Stoppuhr (MM.SS.CS)
# Seriell lesen: IN1-Bits auswerten für CHR (bit3=Start/Stop) und RST (bit4=Reset)
param(
    [string]$ComPort = "COM3",
    [int]$Baud = 115200
)

$port = New-Object System.IO.Ports.SerialPort $ComPort, $Baud, None, 8, One
$port.ReadTimeout = 50
$port.WriteTimeout = 500
$port.Open()
Start-Sleep -Milliseconds 1500

Write-Host "STOPPUHR + UHRZEIT + IN1-Reader auf $ComPort | STRG+C zum Stop" -ForegroundColor Green
Write-Host "Laufzeit`t`tDSP1(Uhr)`tDSP2(Stop)`tIN1-Event" -ForegroundColor Gray

$start   = [DateTime]::UtcNow
$lastPrint = 0
$chronoRunning = $false
$chronoHeldMs = 0
$chronoStartTicks = 0
$prevChr = $false
$prevRst = $false
$serialBuf = ""

try {
    while ($true) {
        $now = [DateTime]::UtcNow
        $ms = [long]($now - $start).TotalMilliseconds

        # --- DSP1: aktuelle Uhrzeit (HH.MM) ---
        $hh = $now.Hour
        $min = $now.Minute
        $clock = "{0:D2}.{1:D2}" -f [int]$hh, [int]$min

        # --- DSP2: Stoppuhr (MM.SS.CS) ---
        if ($chronoRunning) {
            $chronoHeldMs = [long](($now.Ticks - $chronoStartTicks) / 10000)
        }
        # Zeige immer die gehaltene Zeit (auch im Stop)
        $cCs = ($chronoHeldMs / 10) % 100
        $cSs = ($chronoHeldMs / 1000) % 60
        $cMm = ($chronoHeldMs / 60000) % 100
        $stop = "{0:D2}.{1:D2}.{2:D2}" -f [int]$cMm, [int]$cSs, [int]$cCs

        # --- Senden ---
        if (-not $port.IsOpen) { $port.Open(); Start-Sleep -Milliseconds 500 }
        $cmd = "DSP1:$clock;DSP2:$stop;`r`n"
        $port.Write([byte[]][char[]]$cmd, 0, $cmd.Length)

        # --- Seriell lesen ---
        while ($port.BytesToRead -gt 0) {
            $byte = $port.ReadByte()
            if ($byte -ge 32) { $serialBuf += [char]$byte }
        }
        # Verarbeite komplette Zeilen (enden mit ; oder LF)
        while ($serialBuf -match '^(.*?);\s*') {
            $line = $Matches[1]
            $serialBuf = $serialBuf.Substring($Matches[0].Length)
            # IN1:xxxxxxxx auswerten
            if ($line -match '^IN1:([01]{8})') {
                $bits = $Matches[1]
                $chrBtnNow = ($bits[4] -eq '1')    # Bit 3: CHR-Taster (0-indexed Pos4)
                $rstBtnNow = ($bits[3] -eq '1')    # Bit 4: RST-Taster (Pos3)

                # CHR: Toggle bei 0->1 Flanke
                if ($chrBtnNow -and -not $prevChr) {
                    if (-not $chronoRunning) {
                        # War gestoppt -> jetzt starten (ab gespeicherter Zeit)
                        $chronoRunning = $true
                        $chronoStartTicks = $now.Ticks - ($chronoHeldMs * 10000)
                        Write-Host "`n>> CHR:START bei $([int]($ms/1000))s" -ForegroundColor Yellow
                    } else {
                        # War aktiv -> jetzt stoppen, Zeit merken
                        $chronoRunning = $false
                        $chronoHeldMs = ($now.Ticks - $chronoStartTicks) / 10000
                        Write-Host "`n>> CHR:STOP bei $([int]($ms/1000))s (gehalten: $([int]($chronoHeldMs/1000))s)" -ForegroundColor Yellow
                    }
                }
                $prevChr = $chrBtnNow

                # RST: bei 0->1 Flanke -> Reset (auf 0, gestoppt)
                if ($rstBtnNow -and -not $prevRst) {
                    $chronoRunning = $false
                    $chronoHeldMs = 0
                    Write-Host "`n>> CHR:RESET bei $([int]($ms/1000))s" -ForegroundColor Red
                }
                $prevRst = $rstBtnNow
            }
        }

        # --- Konsolenausgabe ---
        if ($ms - $lastPrint -ge 500) {
            $elapsed = [int]($ms / 1000)
            $status = if ($chronoRunning) { "LAEUFT" } else { "STOP" }
            Write-Host ("{0,5}s  {1,-10}  {2}  {3,-8}  T=10ms" -f $elapsed, $clock, $stop, $status) -NoNewline
            Write-Host "`r" -NoNewline
            $lastPrint = $ms
        }

        Start-Sleep -Milliseconds 10
    }
} catch [System.Management.Automation.BreakException] {
    Write-Host "`n`nSTOP nach $(($ms/1000).ToString('F1'))s" -ForegroundColor Cyan
} finally {
    if ($port.IsOpen) { $port.Close() }
    Write-Host "$ComPort geschlossen."
}
