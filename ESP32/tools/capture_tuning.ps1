# capture_tuning.ps1
# PowerShell helper to capture serial output from COM10 at 921600 and save to a timestamped file.
# Usage: .\capture_tuning.ps1 [--python] [--outfile path]
#  --python : use bundled Python capture script (requires pyserial)
#  If not provided, falls back to pio device monitor piping.

param(
    [switch]$UsePython,
    [string]$outfile,
    [string]$command,
    [string]$stopString = "TUNING: capture stopped (auto)"
)

if (-not $outfile) {
    $ts = (Get-Date).ToString('yyyyMMdd_HHmmss')
    $outfile = "tuning_capture_$ts.txt"
}

# If no command provided, default to a sensible auto-start command
if (-not $command -or $command.Trim().Length -eq 0) {
    Write-Host "No -command provided. Using default: 'TUNING START 2000'"
    $command = 'TUNING START 10000'
}

$port = 'COM10'
$baud = 921600

Write-Host "Capturing serial on $port at $baud to $outfile"

# Locate the bundled Python script next to this ps1
$pyPath = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) 'capture_tuning.py'

# Helper: find a python executable
function Find-PythonExecutable {
    $candidates = @('python', 'python3', 'py')
    foreach ($c in $candidates) {
        try {
            $cmd = Get-Command $c -ErrorAction SilentlyContinue
            if ($cmd) { return $cmd.Path }
        } catch { }
    }
    return $null
}

$pythonExe = Find-PythonExecutable

if ($UsePython -or $pythonExe) {
    if (-not (Test-Path $pyPath)) {
        Write-Warning "Python script not found: $pyPath. Falling back to PlatformIO monitor."
    } elseif (-not $pythonExe) {
        Write-Warning "No Python executable found in PATH. Falling back to PlatformIO monitor."
    } else {
        # Build python args and run the capture script
        $args = @($pyPath,'-p',$port,'-b',$baud,'-o',$outfile)
        if ($command) { $args += @('-c',$command) }
        if ($stopString) { $args += @('-s',$stopString) }

        Write-Host "Running: $pythonExe $($args -join ' ')"
        & $pythonExe @args
        $captureExit = $LASTEXITCODE

        # If available, run the analysis script on the saved file
        $analyzePath = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) 'analyze_tuning_capture.py'
        if ((Test-Path $analyzePath) -and $pythonExe) {
            Write-Host "Running analysis: $pythonExe $analyzePath $outfile"
            & $pythonExe $analyzePath $outfile
            $analysisExit = $LASTEXITCODE
            if ($analysisExit -ne 0) {
                Write-Warning "Analysis script exited with code $analysisExit"
            }
        } else {
            Write-Host "Analysis script not found or no Python executable; skipping analysis."
        }

        exit $captureExit
    }
}

# Fallback to PlatformIO device monitor piping
Write-Host "Using PlatformIO monitor. Press Ctrl+C to stop capture."
Push-Location "..\ESP32"
# pio device monitor will run until Ctrl+C; redirect output to file
pio device monitor --port $port --baud $baud 2>&1 | Tee-Object -FilePath (Join-Path (Get-Location) $outfile)
Pop-Location

