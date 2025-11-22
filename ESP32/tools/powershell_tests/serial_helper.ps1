param(
    [string]$PortName = 'COM11',
    [int]$BaudRate = 921600,
    [int]$ReadTimeoutMs = 2000
)

function Open-SerialPort {
    param($PortName, $BaudRate, $ReadTimeoutMs)
    $sp = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, 'None', 8, 'One'
    $sp.ReadTimeout = $ReadTimeoutMs
    $sp.NewLine = "`n"
    $sp.Open()
    return $sp
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
    while((Get-Date) -lt $end) {
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
            # ReadLine timed out -- continue until overall deadline
        } catch {
            # Other serial errors: break out
            break
        }
    }
    return $acc
}
