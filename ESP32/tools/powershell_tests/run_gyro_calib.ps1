<#
  run_gyro_calib.ps1
  Start a gyro calibration with N samples and wait for completion.
#>
param(
  [string]$PortName = 'COM11',
  [int]$BaudRate = 921600,
  [int]$Samples = 500,
  [int]$TimeoutSec = 60
)

. $(Join-Path $PSScriptRoot 'serial_helper.ps1')

$logFile = Join-Path $PSScriptRoot ("gyro_calib_$(Get-Date -Format 'yyyyMMdd_HHmmss').log")
$sp = Open-SerialPort -PortName $PortName -BaudRate $BaudRate -ReadTimeoutMs 500
Write-Host "Opened $PortName @ $BaudRate, logging to $logFile"

$cmd = "CALIB START GYRO $Samples"
Write-LineAndLog -sp $sp -line $cmd -logFile $logFile

$deadline = (Get-Date).AddSeconds($TimeoutSec)
$done = $false
while ((Get-Date) -lt $deadline -and -not $done) {
    Start-Sleep -Milliseconds 200
    $out = Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 200
    if ($out -match 'CALIB DONE' -or $out -match 'CALIB FAIL') {
        $done = $true
    }
}

if (-not $done) {
    Write-Host "Timeout waiting for calibration completion"
} else {
    Write-Host "Calibration finished. Dumping results..."
    Write-LineAndLog -sp $sp -line 'CALIB DUMP' -logFile $logFile
    Start-Sleep -Milliseconds 200
    Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500
}

$sp.Close()
Write-Host "Done. Log: $logFile"
