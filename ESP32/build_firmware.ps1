<#
build_firmware.ps1

PowerShell helper to build the ESP32 firmware using PlatformIO from the ESP32 project root.
#>

param(
    [Parameter(ValueFromRemainingArguments=$true)]
    [string[]] $RemainingArgs = @()
)

Set-StrictMode -Version Latest

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
Push-Location $scriptDir

try {
    Write-Host "[build_firmware] Running PlatformIO build in: $PWD"
    $pio = Get-Command pio -ErrorAction SilentlyContinue
    if (-not $pio) {
        Write-Error "[build_firmware] ERROR: 'pio' (PlatformIO CLI) not found in PATH."
        Write-Host "Install PlatformIO Core (https://docs.platformio.org/) or add it to PATH." -ForegroundColor Yellow
        exit 2
    }

    if (-not $RemainingArgs) { $RemainingArgs = @() }
    $args = @('run') + $RemainingArgs
    Write-Host "[build_firmware] Command: pio $($args -join ' ')"
    & pio @args
    $code = $LASTEXITCODE
    if ($code -ne 0) { throw "PlatformIO build failed (exit $code)" }
    Write-Host "[build_firmware] Build finished." -ForegroundColor Green
} catch {
    Write-Error "[build_firmware] $_"
    exit 1
} finally {
    Pop-Location
}
