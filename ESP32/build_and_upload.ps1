<#
build_and_upload.ps1

PowerShell helper to build and then upload the ESP32 firmware using PlatformIO.
#>

param(
    [Parameter(ValueFromRemainingArguments=$true)]
    [string[]] $RemainingArgs = @()
)

Set-StrictMode -Version Latest

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
Push-Location $scriptDir

try {
    Write-Host "[build_and_upload] Running PlatformIO build+upload in: $PWD"
    $pio = Get-Command pio -ErrorAction SilentlyContinue
    if (-not $pio) {
        Write-Error "[build_and_upload] ERROR: 'pio' (PlatformIO CLI) not found in PATH."
        Write-Host "Install PlatformIO Core (https://docs.platformio.org/) or add it to PATH." -ForegroundColor Yellow
        exit 2
    }

    if (-not $RemainingArgs) { $RemainingArgs = @() }
    $buildArgs = @('run') + $RemainingArgs
    Write-Host "[build_and_upload] Build command: pio $($buildArgs -join ' ')"
    & pio @buildArgs
    $code = $LASTEXITCODE
    if ($code -ne 0) { throw "PlatformIO build failed (exit $code)" }

    Write-Host "[build_and_upload] Build finished. Starting upload..."
    $uploadArgs = @('run','--target','upload')
    Write-Host "[build_and_upload] Upload command: pio $($uploadArgs -join ' ')"
    & pio @uploadArgs
    $code2 = $LASTEXITCODE
    if ($code2 -ne 0) { throw "PlatformIO upload failed (exit $code2)" }

    Write-Host "[build_and_upload] Upload finished." -ForegroundColor Green
} catch {
    Write-Error "[build_and_upload] $_"
    exit 1
} finally {
    Pop-Location
}
