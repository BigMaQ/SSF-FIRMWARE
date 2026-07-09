<# 
.SYNOPSIS
    SSF Firmware Build & Version Manager
.DESCRIPTION
    Kompiliert .INO Firmware via Arduino CLI und deployed .HEX Dateien.
    Gehoert in den FIRMWARE-Ordner (nicht FDC).
.PARAMETER Build
    Build aller Panels (oder eines mit -Panel).
.PARAMETER CheckVersion
    Nur Versionen anzeigen - kein Build.
.PARAMETER Status
    Prueft ob Builds ausstehen (Exit 0=aktuell, 1=Build noetig).
.PARAMETER Panel
    Nur ein bestimmtes Panel builden/checken (z.B. -Panel RMP).
.PARAMETER Git
    Nach erfolgreichem Build Git commit + push (mit Bestaetigung).
.PARAMETER AddPanel
    Neue .INO-Dateien im Source-Ordner erkennen und zum Build-System hinzufuegen.
.PARAMETER Rebuild
    Alle Panels (oder eines mit -Panel) neu builden und bestehende HEX
    mit gleicher Versionsnummer ueberschreiben (fuer Tests/Kleinigkeiten).
.EXAMPLE
    .\fwbuild.ps1 -Build -Git         # Build + Git commit/push
    .\fwbuild.ps1 -Build -Panel ACP   # Nur ACP builden
    .\fwbuild.ps1 -Rebuild             # Alles rebuilden (ueberschreibt)
    .\fwbuild.ps1 -Rebuild -Panel RMP # Nur RMP rebuilden
    .\fwbuild.ps1 -CheckVersion        # Version-Check
    .\fwbuild.ps1 -Status              # Build-Status pruefen
    .\fwbuild.ps1                      # Diese Hilfe anzeigen
#>

param(
    [switch]$Build,
    [switch]$CheckVersion,
    [switch]$Status,
    [switch]$Git,
    [switch]$AddPanel,
    [switch]$Rebuild,
    [string]$Panel = ""
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$FirmwareDir = $ScriptDir
$SourceDir   = "$FirmwareDir\source"
$ArduinoCli  = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"

# Panel definitions
$Panels = @(
    @{Key="RMP";    InoName="RMP";    FlatFile="RMP.ino";        OutDir="rmp";       Fqbn="arduino:avr:micro"},
    @{Key="ACP";    InoName="ACP";    FlatFile="ACP.ino";        OutDir="acp";       Fqbn="arduino:avr:micro"},
    @{Key="ATC";    InoName="ATC";    FlatFile="ATC.ino";        OutDir="atc";       Fqbn="arduino:avr:micro"},
    @{Key="ECAM";   InoName="ECAM";   FlatFile="ECAM.ino";       OutDir="ecam";      Fqbn="arduino:avr:micro"},
    @{Key="MIP";    InoName="MIP";    FlatFile="MIP.ino";        OutDir="mip";       Fqbn="arduino:avr:mega:cpu=atmega2560"},
    @{Key="RUD";    InoName="RUD";    FlatFile="RUD.ino";        OutDir="rud";       Fqbn="arduino:avr:micro"},
    @{Key="TILLER"; InoName="TILLER"; FlatFile="Tiller_CPT.ino"; OutDir="tiller";    Fqbn="arduino:avr:micro"},
    @{Key="HYDPRS"; InoName="HYDPRS"; FlatFile="accu_press.ino"; OutDir="accupress"; Fqbn="arduino:avr:micro"},
    @{Key="DOOR";   InoName="DOOR";   FlatFile="CPT_DOOR.ino";    OutDir="door";      Fqbn="arduino:avr:micro"},
    @{Key="OHP_PRI"; InoName="OHP_PRI"; FlatFile="OHP_PRI.INO";   OutDir="ohp_pri";  Fqbn="arduino:avr:mega:cpu=atmega2560"},
    @{Key="OHP_SEC"; InoName="OHP_SEC"; FlatFile="OHP_SEC.INO";   OutDir="ohp_sec";  Fqbn="arduino:avr:mega:cpu=atmega2560"},
    @{Key="WXR";    InoName="WXR";    FlatFile="WXR.ino";        OutDir="wxr";       Fqbn="arduino:avr:micro"}
)

# ═══════════════════════ 1. SYNC ═══════════════════════
function Sync-SketchFolders {
    Write-Host "=== Syncing sketch folders ===" -ForegroundColor Cyan
    foreach ($p in $Panels) {
        $srcFile   = Join-Path $SourceDir $p.FlatFile
        $sketchDir = Join-Path $SourceDir $p.InoName
        $destFile  = Join-Path $sketchDir "$($p.InoName).ino"

        if (-not (Test-Path $srcFile)) {
            Write-Host "  X MISSING: $srcFile" -ForegroundColor Red
            continue
        }
        if (-not (Test-Path $sketchDir)) {
            New-Item -ItemType Directory -Path $sketchDir -Force | Out-Null
        }

        $srcHash  = (Get-FileHash $srcFile -Algorithm MD5).Hash
        $destHash = ""
        if (Test-Path $destFile) { $destHash = (Get-FileHash $destFile -Algorithm MD5).Hash }
        
        if ($srcHash -ne $destHash) {
            Copy-Item $srcFile $destFile -Force
            Write-Host "  + Synced: $($p.Key)" -ForegroundColor Green
        } else {
            Write-Host "  . $($p.Key) up-to-date" -ForegroundColor DarkGray
        }
    }
    Write-Host ""
}

# ═══════════════════════ 2. VERSION ═══════════════════════
function Get-InoVersion {
    param([string]$FilePath)
    if (-not (Test-Path $FilePath)) { return "?" }
    $content = Get-Content $FilePath -Raw -Encoding UTF8
    
    if ($content -match 'FW_VERSION[^"]*"([^"]+)"') { return $matches[1] }
    if ($content -match 'PANEL_VERSION[^"]*"([^"]+)"') { return $matches[1] }
    if ($content -match 'PANEL_IDENT[^"]*v(\d+\.\d+)') { return $matches[1] }
    if ($content -match 'IDENT_STRING[^"]*v(\d+\.\d+)') { return $matches[1] }
    return "?"
}

function Get-LatestHexVersion {
    param([string]$Folder)
    if (-not (Test-Path $Folder)) { return "" }
    $versions = @()
    Get-ChildItem $Folder -Filter "*.hex" | ForEach-Object {
        if ($_.Name -match 'v?(\d+\.\d+)\.hex') { $versions += $matches[1] }
    }
    if ($versions.Count -eq 0) { return "" }
    if ($versions.Count -eq 1) { return [string]$versions[0] }
    $sorted = $versions | Sort-Object { [version]$_ } -Descending
    return [string]$sorted[0]
}

function Compare-Versions {
    param([string]$a, [string]$b)
    if ($a -eq "?" -or $b -eq "?" -or $a -eq "" -or $b -eq "") { return 0 }
    $aParts = $a -split '\.'; $bParts = $b -split '\.'
    $aMaj = [int]$aParts[0]; $aMin = [int]$aParts[1]
    $bMaj = [int]$bParts[0]; $bMin = [int]$bParts[1]
    if ($aMaj -ne $bMaj) { return ($aMaj - $bMaj) }
    return ($aMin - $bMin)
}

function Get-AllStatus {
    $results = @()
    foreach ($p in $Panels) {
        $sketchDir = Join-Path $SourceDir $p.InoName
        $inoFile   = Join-Path $sketchDir "$($p.InoName).ino"
        if (-not (Test-Path $inoFile)) { $inoFile = Join-Path $SourceDir $p.FlatFile }
        $hexDir = Join-Path $FirmwareDir $p.OutDir
        $hv = [string](Get-LatestHexVersion $hexDir)
        
        $results += [PSCustomObject]@{
            Key = $p.Key; InoVersion = Get-InoVersion $inoFile
            HexVersion = $hv
            HexDir = $hexDir; InoFile = $inoFile; Fqbn = $p.Fqbn
        }
    }
    return $results
}

function Write-StatusTable {
    param([array]$Results)
    Write-Host ("{0,-8}  {1,-8}  {2,-8}  {3,-6}  {4}" -f "Panel", "INO Ver", "HEX Ver", "Status", "Action")
    Write-Host ("-" * 65)
    $needBuild = 0
    foreach ($r in $Results) {
        $icon = "+"; $action = ""
        if ($r.InoVersion -eq "?") { $icon = "X"; $action = "MISSING" }
        elseif ($r.HexVersion -ne "" -and (Compare-Versions $r.HexVersion $r.InoVersion) -gt 0) {
            $icon = "^"; $action = "HEX > INO"
        }
        elseif ($r.InoVersion -ne $r.HexVersion) { $icon = ">"; $action = "BUILD"; $needBuild++ }
        else { $action = "up-to-date" }
        $hv = if ($r.HexVersion) { "v{0}" -f $r.HexVersion } else { "-" }
        $iv = "v{0}" -f $r.InoVersion
        Write-Host ("{0,-8}  {1,-8}  {2,-8}  {3,-6}  {4}" -f $r.Key, $iv, $hv, $icon, $action)
    }
    Write-Host ""
    $color = if ($needBuild -gt 0) { "Yellow" } else { "Green" }
    Write-Host "BUILD NEEDED: $needBuild panel(s)" -ForegroundColor $color
    return $needBuild
}

# ═══════════════════════ ADD NEW PANEL ═══════════════════════
function Add-NewPanel {
    Write-Host "=== New Panel Detection ===" -ForegroundColor Cyan
    $knownKeys = $Panels | ForEach-Object { $_.Key }
    $knownFiles = $Panels | ForEach-Object { $_.FlatFile }
    
    $newInos = @(Get-ChildItem $SourceDir -Filter "*.ino" | Where-Object {
        $_.Name -notin $knownFiles
    })
    
    if ($newInos.Count -eq 0) {
        Write-Host "  No new .INO files found." -ForegroundColor DarkGray
        return
    }
    
    Write-Host "  Found new .INO files:" -ForegroundColor Yellow
    $newInos | ForEach-Object { Write-Host "    $($_.Name)" -ForegroundColor White }
    Write-Host ""
    
    foreach ($ino in $newInos) {
        $add = Read-Host "  Add '$($ino.Name)' to build system? (J/N)"
        if ($add -notmatch '^[JjYy]') { continue }
        
        $baseName = [System.IO.Path]::GetFileNameWithoutExtension($ino.Name)
        $key = Read-Host "    Panel key (e.g. PED, DOOR) [$baseName]"
        if (-not $key) { $key = $baseName.ToUpper() }
        $key = $key.ToUpper()
        
        $outDir = Read-Host "    Output folder name [$( $key.ToLower() )]"
        if (-not $outDir) { $outDir = $key.ToLower() }
        
        Write-Host "    Board type:"
        Write-Host "      1) Arduino Micro (ATmega32U4) - Pro Micro compatible"
        Write-Host "      2) Arduino Mega 2560"
        Write-Host "      3) Arduino Uno / Nano (ATmega328P)"
        $boardChoice = Read-Host "    Select [1]"
        if (-not $boardChoice) { $boardChoice = "1" }
        
        $fqbn = switch ($boardChoice) {
            "1" { "arduino:avr:micro" }
            "2" { "arduino:avr:mega:cpu=atmega2560" }
            "3" { "arduino:avr:uno" }
            default { "arduino:avr:micro" }
        }
        
        $newEntry = "@{Key=`"$key`"; InoName=`"$key`"; FlatFile=`"$($ino.Name)`"; OutDir=`"$outDir`"; Fqbn=`"$fqbn`"}"
        
        Write-Host "    + Adding: $newEntry" -ForegroundColor Green
        
        $scriptPath = Join-Path $ScriptDir "fwbuild.ps1"
        $scriptContent = Get-Content $scriptPath -Raw
        $insertMarker = ")`r`n"
        $insertPos = $scriptContent.IndexOf($insertMarker) + 3
        if ($insertPos -lt 3) { $insertPos = $scriptContent.IndexOf("# ═══════════════════════ 1. SYNC") - 3 }
        
        $indent = "    "
        $newLine = "$indent$newEntry,`r`n"
        $scriptContent = $scriptContent.Insert($insertPos, $newLine)
        Set-Content $scriptPath $scriptContent -NoNewline
        
        $hexDir = Join-Path $FirmwareDir $outDir
        New-Item -ItemType Directory -Path $hexDir -Force | Out-Null
        Write-Host "    + Output folder: $hexDir" -ForegroundColor Green
        
        $script:Panels += @{Key=$key; InoName=$key; FlatFile=$ino.Name; OutDir=$outDir; Fqbn=$fqbn}
    }
    
    Write-Host ""
    Write-Host "  Run .\fwbuild.ps1 to see updated panel list." -ForegroundColor Cyan
}

# No params -> show help
if (-not $Build -and -not $CheckVersion -and -not $Status -and -not $AddPanel -and -not $Rebuild) {
    Write-Host @"
SSF Firmware Build Tool
=======================

  .\fwbuild.ps1 -Build              Build aller Panels
  .\fwbuild.ps1 -Build -Git         Build + Git commit/push
  .\fwbuild.ps1 -Build -Panel ACP   Nur ACP builden
  .\fwbuild.ps1 -Rebuild             Alles rebuilden (ueberschreibt HEX)
  .\fwbuild.ps1 -Rebuild -Panel RMP Nur RMP rebuilden
  .\fwbuild.ps1 -CheckVersion        Nur Versionen anzeigen
  .\fwbuild.ps1 -Status              Pruefen ob Builds ausstehen
                                     (Exit 0 = aktuell, 1 = Build noetig)
  .\fwbuild.ps1 -AddPanel            Neue .INO im Source-Ordner suchen
                                     und zum Build-System hinzufuegen

Panels: $($Panels.Key -join ', ')
"@
    exit 0
}

# AddPanel mode (no sync needed, standalone)
if ($AddPanel) {
    Add-NewPanel
    exit 0
}

Sync-SketchFolders
$allStatus = Get-AllStatus

if ($Panel) {
    $allStatus = @($allStatus | Where-Object { $_.Key -eq $Panel.ToUpper() })
    if ($allStatus.Count -eq 0) { Write-Host "Unknown: $Panel" -ForegroundColor Red; exit 1 }
}

if ($Status) {
    $need = Write-StatusTable $allStatus
    if ($need -gt 0) { exit 1 } else { exit 0 }
}

if ($CheckVersion) {
    Write-StatusTable $allStatus | Out-Null
    exit 0
}

# BUILD (with -Build or -Rebuild flag)
if (-not $Build -and -not $Rebuild) { exit 0 }

Write-Host "=== FIRMWARE $(if ($Rebuild) {'REBUILD'} else {'BUILD'}) ===" -ForegroundColor Cyan
$need = Write-StatusTable $allStatus
Write-Host ""

if (-not (Test-Path $ArduinoCli)) {
    Write-Host "ERROR: Arduino CLI not found at $ArduinoCli" -ForegroundColor Red
    exit 1
}

$built = 0; $failed = 0; $skipped = 0

foreach ($p in $allStatus) {
    if ($p.InoVersion -eq "?") { Write-Host "  - $($p.Key): SKIP" -ForegroundColor DarkGray; $skipped++; continue }
    # Rebuild: always build | Build: only if version differs
    if (-not $Rebuild -and $p.InoVersion -eq $p.HexVersion) { Write-Host "  + $($p.Key): v$($p.InoVersion) current" -ForegroundColor DarkGray; $skipped++; continue }
    
    Write-Host "  >> $($p.Key): $(if ($Rebuild) {'REBUILD'} else {''}) v$($p.InoVersion)..." -ForegroundColor Yellow
    $buildDir = Join-Path $FirmwareDir "_build\$($p.Key)"
    $hexDir   = $p.HexDir
    $sketchFolder = Split-Path -Parent $p.InoFile
    
    New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
    New-Item -ItemType Directory -Path $hexDir -Force | Out-Null
    
    $savedEAP = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $result = & $ArduinoCli compile --fqbn $p.Fqbn --build-path $buildDir $sketchFolder 2>&1
    $ErrorActionPreference = $savedEAP
    
    $hexFiles = @(Get-ChildItem $buildDir -Filter "*.hex" -ErrorAction SilentlyContinue)
    $hexFile = $null
    foreach ($hf in $hexFiles) { if ($hf.Name -notmatch "bootloader") { $hexFile = $hf; break } }
    if (-not $hexFile -and $hexFiles.Count -gt 0) { $hexFile = $hexFiles[0] }
    
    if ($LASTEXITCODE -ne 0 -and -not $hexFile) {
        Write-Host "    X COMPILE FAILED" -ForegroundColor Red
        $result | Select-Object -Last 6 | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkGray }
        $failed++; continue
    }
    
    if ($LASTEXITCODE -ne 0 -and $hexFile) {
        Write-Host "    ! Compiled with warnings (low memory), using .hex" -ForegroundColor DarkYellow
    }
    
    if (-not $hexFile) { Write-Host "    X No .hex" -ForegroundColor Red; $failed++; continue }
    
    $destPath = Join-Path $hexDir "v$($p.InoVersion).hex"
    Copy-Item $hexFile.FullName $destPath -Force
    Write-Host "    + $destPath" -ForegroundColor Green
    $built++
}

Write-Host ""
Write-Host "=== SUMMARY: $built built, $failed failed, $skipped skipped ===" -ForegroundColor Cyan

# Git commit (mit -Git Flag immer anbieten, auch wenn kein Panel neu gebaut wurde)
if ($Git) {
    Write-Host ""
    Write-Host "Firmware Repo: $FirmwareDir" -ForegroundColor Cyan
    if ($built -eq 0) {
        Write-Host "  (alle Panels aktuell, kein neuer Build)" -ForegroundColor DarkGray
    }
    $commit = Read-Host "Jetzt auf GitHub committen? (J/N)"
    if ($commit -eq "J" -or $commit -eq "j" -or $commit -eq "Y" -or $commit -eq "y") {
        Push-Location $FirmwareDir
        try {
            $dateStr = Get-Date -Format "yyyy-MM-dd HH:mm"
            Write-Host "  git add . ..." -ForegroundColor Cyan
            git add .
            Write-Host "  git commit ..." -ForegroundColor Cyan
            git commit -m "Firmware Version Update $dateStr"
            Write-Host "  git push ..." -ForegroundColor Cyan
            git push
            Write-Host "  + Push completed" -ForegroundColor Green
        } finally {
            Pop-Location
        }
    } else {
        Write-Host "  . Commit skipped" -ForegroundColor DarkGray
    }
}

if ($failed -gt 0) { exit 1 } else { exit 0 }
