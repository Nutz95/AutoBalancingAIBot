param(
    [string]$PortName = 'COM11',
    [int]$BaudRate = 921600,
    [int]$ReadTimeoutMs = 200
)

# Global stop flag set by CancelKeyPress handler
$global:stopRequested = $false
$global:__cancelSub = $null

function Open-SerialPort {
    param($PortName, $BaudRate, $ReadTimeoutMs)
    $sp = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, 'None', 8, 'One'
    $sp.ReadTimeout = $ReadTimeoutMs
    $sp.NewLine = "`n"
    $sp.Open()
    # Register Ctrl+C handler to set stopRequested so loops can exit gracefully
    try {
        if (-not $global:__cancelSub) {
            $global:stopRequested = $false
            $global:__cancelSub = Register-ObjectEvent -InputObject [Console] -EventName 'CancelKeyPress' -Action {
                $global:stopRequested = $true
                Write-Host "Ctrl+C pressed — stopping..."
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
        if ($global:__cancelSub) {
            Unregister-Event -SubscriptionId $global:__cancelSub.Id -ErrorAction SilentlyContinue
            Remove-Job -Id $global:__cancelSub.Id -ErrorAction SilentlyContinue
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
            # Attempt to read one line (blocks until ReadTimeout)
            $line = $sp.ReadLine()
            if ($line -ne $null -and $line.Length -gt 0) {
                $ts = (Get-Date).ToString('o')
                $entry = $ts + ' RX: ' + $line
                $entry | Out-File -FilePath $logFile -Append -Encoding utf8
                Write-Host $line
                $acc += $line + "`n"
            }
        } catch [System.TimeoutException] {
            # ReadLine timed out -- continue until overall deadline or stop
        } catch {
            # Other serial errors: break out
            break
        }
    }
    return $acc
}
