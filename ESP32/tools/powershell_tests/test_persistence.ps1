<#
  test_persistence.ps1
  Run a quick calibration, then prompt the user to reset the device and verify calibration is reloaded on boot.
#>
param(
  [string]$PortName = 'COM11',
  [int]$BaudRate = 921600,
  [int]$Samples = 2000,
  [int]$TimeoutSec = 60
)

. $(Join-Path $PSScriptRoot 'serial_helper.ps1')

$logFile = Join-Path $PSScriptRoot ("persistence_test_$(Get-Date -Format 'yyyyMMdd_HHmmss').log")
$sp = Open-SerialPort -PortName $PortName -BaudRate $BaudRate -ReadTimeoutMs 500
Write-Host "Opened $PortName @ $BaudRate, logging to $logFile"

Write-LineAndLog -sp $sp -line ("CALIB START GYRO {0}" -f $Samples) -logFile $logFile

$deadline = (Get-Date).AddSeconds($TimeoutSec)
$done = $false
while ((Get-Date) -lt $deadline -and -not $done) {
    Start-Sleep -Milliseconds 200
    $out = Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 200
    if ($out -match 'CALIB DONE' -or $out -match 'CALIB FAIL') {
        $done = $true
    }
}

if (-not $done) { Write-Host "Calibration timed out"; $sp.Close(); exit 1 }

Write-Host "Calibration done. Now please press the ESP32 EN (reset) button or power-cycle the board. Press Enter when done..."
Read-Host

# After reset, just open the port again and capture boot output
$sp.Close()
Start-Sleep -Seconds 1
$sp = Open-SerialPort -PortName $PortName -BaudRate $BaudRate -ReadTimeoutMs 2000
Write-Host "Capturing boot messages for 5 seconds..."
Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 5000
Write-Host "Now send CALIB DUMP to verify persistence"
Write-LineAndLog -sp $sp -line 'CALIB DUMP' -logFile $logFile
Start-Sleep -Milliseconds 500
Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500

$sp.Close()
Write-Host "Done. Log: $logFile"
