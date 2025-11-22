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
}

function Read-AllAvailable {
    param($sp, $logFile, $timeoutMs = 1000)
    $end = (Get-Date).AddMilliseconds($timeoutMs)
    $acc = ""
    while((Get-Date) -lt $end) {
        Start-Sleep -Milliseconds 50
        try {
            $s = $sp.ReadExisting()
            if ($s -and $s.Length -gt 0) {
                $acc += $s
                $ts = (Get-Date).ToString('o')
                ($ts + ' RX: ' + $s) | Out-File -FilePath $logFile -Append -Encoding utf8
            }
        } catch {
            # ignore read timeout
        }
    }
    return $acc
}
