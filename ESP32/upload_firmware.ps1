<#
upload_firmware.ps1

PowerShell helper to upload previously built firmware to the ESP32 using PlatformIO.
#>

param(
    [Parameter(ValueFromRemainingArguments=$true)]
    [string[]] $RemainingArgs = @()
)

Set-StrictMode -Version Latest

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
Push-Location $scriptDir

try {
    Write-Host "[upload_firmware] Running PlatformIO upload in: $PWD"
    $pio = Get-Command pio -ErrorAction SilentlyContinue
    if (-not $pio) {
        Write-Error "[upload_firmware] ERROR: 'pio' (PlatformIO CLI) not found in PATH."
        Write-Host "Install PlatformIO Core (https://docs.platformio.org/) or add it to PATH." -ForegroundColor Yellow
        exit 2
    }

    if (-not $RemainingArgs) { $RemainingArgs = @() }
    $uploadArgs = @('run','--target','upload') + $RemainingArgs
    Write-Host "[upload_firmware] Command: pio $($uploadArgs -join ' ')"
    & pio @uploadArgs
    $code = $LASTEXITCODE
    if ($code -ne 0) { throw "PlatformIO upload failed (exit $code)" }
    Write-Host "[upload_firmware] Upload finished." -ForegroundColor Green
} catch {
    Write-Error "[upload_firmware] $_"
    exit 1
} finally {
    Pop-Location
}
