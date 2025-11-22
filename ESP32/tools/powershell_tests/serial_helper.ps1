# NOTE: this file provides helper functions only. Do NOT define a top-level `param()` here
# because the file is dot-sourced by caller scripts and a param block would overwrite
# caller variables (e.g. `$PortName`). All defaults are handled in `Open-SerialPort()`.

# Global stop flag set by CancelKeyPress handler
$global:stopRequested = $false
$global:__cancelSub = $null

function Open-SerialPort {
    param($PortName, $BaudRate, $ReadTimeoutMs, [switch]$ForcePoll)
    $sp = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, 'None', 8, 'One'
    $sp.ReadTimeout = $ReadTimeoutMs
    $sp.NewLine = "`n"
    $sp.Open()
    # Store ForcePoll selection globally so Read-AllAvailable can check for key events
    if ($ForcePoll) { $global:__forcePoll = $true } else { if (-not $global:__forcePoll) { $global:__forcePoll = $false } }
    # Register Ctrl+C handler to set stopRequested so loops can exit gracefully
    try {
        if (-not $global:__cancelSub) {
            $global:stopRequested = $false
            # Try to register a CancelKeyPress handler; suppress errors on hosts that don't support it
            $sub = $null
            try {
                $sub = Register-ObjectEvent -InputObject [Console] -EventName 'CancelKeyPress' -Action {
                    $global:stopRequested = $true
                    Write-Host "Ctrl+C pressed - stopping..."
                } -ErrorAction Stop
            } catch {
                # Host might not allow event registration (e.g. some restricted hosts).
                # Fall back to relying on PowerShell's interruption (Ctrl+C) and short ReadTimeouts.
                $sub = $null
                Write-Host "Note: Ctrl+C handler registration not supported in this host; the script will rely on PowerShell interrupt to stop immediately." -ForegroundColor Yellow
            }
            if ($sub) {
                $global:__cancelSub = $sub
            } else {
                $global:__cancelSub = $null
            }
        }
    } catch {
        # ignore if event registration not available
    }
    return $sp
}

function Close-SerialPort {
    param($sp)
    try {
        if ($sp -and $sp.IsOpen) { $sp.Close() }
    } catch {}
    try {
        if ($global:__cancelSub -and $global:__cancelSub.Id) {
            try { Unregister-Event -SubscriptionId $global:__cancelSub.Id -ErrorAction SilentlyContinue } catch {}
            try { Remove-Job -Id $global:__cancelSub.Id -ErrorAction SilentlyContinue } catch {}
            $global:__cancelSub = $null
        }
    } catch {}
}

function Write-LineAndLog {
    param($sp, $line, $logFile)
    $ts = (Get-Date).ToString('o')
    $entry = "$ts TX: $line"
    $entry | Out-File -FilePath $logFile -Append -Encoding utf8
    $sp.WriteLine($line)
    # also echo the transmitted line to console for visibility
    Write-Host "TX: $line"
}

function Read-AllAvailable {
    param($sp, $logFile, $timeoutMs = 1000)
    $end = (Get-Date).AddMilliseconds($timeoutMs)
    $acc = ""
    while((Get-Date) -lt $end -and -not $global:stopRequested) {
        try {
            # First, if there are bytes available in the input buffer, read them all
            if ($sp.BytesToRead -gt 0) {
                try {
                    $chunk = $sp.ReadExisting()
                    if ($chunk -and $chunk.Length -gt 0) {
                        # split into lines for logging/console output
                        $lines = $chunk -split "`r?`n"
                        foreach ($l in $lines) {
                            if ($l -and $l.Length -gt 0) {
                                $ts = (Get-Date).ToString('o')
                                $entry = $ts + ' RX: ' + $l
                                $entry | Out-File -FilePath $logFile -Append -Encoding utf8
                                Write-Host $l
                                $acc += "$l`n"
                            }
                        }
                    }
                } catch {
                    # Ignore transient read errors here and proceed to ReadLine
                }
            }
            # If ForcePoll is enabled, check console key state to detect Ctrl+C in hosts
            if ($global:__forcePoll) {
                try {
                    if ([Console]::KeyAvailable) {
                        $k = [Console]::ReadKey($true)
                        if ($k.Key -eq 'C' -and ($k.Modifiers -band [ConsoleModifiers]::Control)) {
                            $global:stopRequested = $true
                            Write-Host "Ctrl+C pressed (detected by ForcePoll) - stopping..."
                            break
                        }
                    }
                } catch {
                    # Some hosts may not allow Console access; ignore and continue
                }
            }
            # Attempt to read one line (blocks until ReadTimeout)
            $line = $sp.ReadLine()
            if ($line -ne $null -and $line.Length -gt 0) {
                $ts = (Get-Date).ToString('o')
                $entry = $ts + ' RX: ' + $line
                $entry | Out-File -FilePath $logFile -Append -Encoding utf8
                Write-Host $line
                $acc += "$line`n"
            }
        } catch [System.TimeoutException] {
            # ReadLine timed out -- continue until overall deadline or stop
        } catch {
            # If the exception appears to be a pipeline/host interruption (Ctrl+C), rethrow
            $ex = $_.Exception
            $typeName = $ex.GetType().FullName
            if ($typeName -match 'PipelineStoppedException|StopUpstream|OperationCanceledException|ThreadAbortException') {
                throw
            }
            # Other serial errors or host-related errors: break out
            break
        }
    }
    return $acc
}
