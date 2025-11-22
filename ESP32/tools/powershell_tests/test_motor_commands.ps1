<#
  test_motor_commands.ps1
  Sends a small sequence of MOTOR commands to the device and logs responses.
#>
param(
  [string]$PortName = 'COM10',
  [int]$BaudRate = 921600,
  [switch]$ForcePoll
)

. $(Join-Path $PSScriptRoot 'serial_helper.ps1')

$logFile = Join-Path $PSScriptRoot ("motor_test_$(Get-Date -Format 'yyyyMMdd_HHmmss').log")
$sp = $null
try {
    $sp = Open-SerialPort -PortName $PortName -BaudRate $BaudRate -ReadTimeoutMs 200 -ForcePoll:$ForcePoll
    Write-Host "Opened $PortName @ $BaudRate, logging to $logFile"
    
    Start-Sleep -Milliseconds 2000
    Write-LineAndLog -sp $sp -line 'MOTOR STATUS' -logFile $logFile
    Start-Sleep -Milliseconds 200
    Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500

    Write-LineAndLog -sp $sp -line 'MOTOR ENABLE' -logFile $logFile
    Start-Sleep -Milliseconds 200
    Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500

    Write-LineAndLog -sp $sp -line 'MOTOR SET 8 0.10' -logFile $logFile
    Start-Sleep -Milliseconds 200
    Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500

    Write-LineAndLog -sp $sp -line 'MOTOR SET 8 -0.10' -logFile $logFile
    Start-Sleep -Milliseconds 200
    Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500

    Write-LineAndLog -sp $sp -line 'MOTOR DISABLE' -logFile $logFile
    Start-Sleep -Milliseconds 200
    Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500

    Write-LineAndLog -sp $sp -line 'MOTOR DUMP' -logFile $logFile
    Start-Sleep -Milliseconds 200
    Read-AllAvailable -sp $sp -logFile $logFile -timeoutMs 500

    Write-Host "Done. Log: $logFile"
    } catch {
      Write-Host "Script interrupted or error: $($_.Exception.GetType().FullName) - $($_.Exception.Message)" -ForegroundColor Yellow
      throw
    } finally {
      if ($sp) { Close-SerialPort -sp $sp }
    }
