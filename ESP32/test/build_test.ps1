<#
build_test.ps1

Simple PowerShell helper to compile and run the comprehensive host-native unit test
`test/imu_fusion_tests.cpp` using a MinGW-w64 `g++` toolchain.

Behaviour:
- Detect `g++` in PATH
- If not found, try default MSYS2 mingw64 paths (C:\msys64 or D:\msys64)
- Compile `test/imu_fusion_tests.cpp` and `src/imu_fusion.cpp` with -I include
- Run the produced `build\imu_fusion_tests.exe`

Note: imu_fusion_tests.cpp includes comprehensive tests (quaternion norm, axis stationary,
high rotation, zero accel, noise robustness, linear accel, variable dt/spike, fuzz) which
fully cover and exceed the basic tests in imu_fusion_unit.cpp.

Usage (PowerShell):
  cd 'I:\GIT\AutoBalancingAIBot\ESP32\test'
  .\build_test.ps1

#>
Set-StrictMode -Version Latest

function Find-Gpp {
    $g = Get-Command g++ -ErrorAction SilentlyContinue
    if ($g) { return $g.Path }

    $candidates = @(
        'C:\msys64\mingw64\bin\g++.exe',
        'D:\msys64\mingw64\bin\g++.exe',
        'C:\msys64\usr\bin\g++.exe',
        'D:\msys64\usr\bin\g++.exe'
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) { return $p }
    }
    return $null
}

function Find-Bash {
    $candidates = @(
        'C:\msys64\usr\bin\bash.exe',
        'D:\msys64\usr\bin\bash.exe',
        'C:\msys64\bin\bash.exe',
        'D:\msys64\bin\bash.exe'
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) { return $p }
    }
    return $null
}

try {
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
    $projDir = Resolve-Path (Join-Path $scriptDir '..')
    $buildDir = Join-Path $projDir 'build'
    if (-not (Test-Path $buildDir)) { New-Item -ItemType Directory -Path $buildDir | Out-Null }

    $gpp = Find-Gpp
    $bash = Find-Bash
    if (-not $gpp) {
        Write-Host 'g++ not found. Please install MSYS2 (mingw-w64) and/or add g++ to PATH.' -ForegroundColor Yellow
        Write-Host 'See https://www.msys2.org for installation instructions.'
        exit 2
    }

    $gppPath = $gpp
    Write-Host "Using g++: $gppPath"

    # Compile and run the comprehensive test suite
    $srcTest = Join-Path $projDir 'test\imu_fusion_tests.cpp'
    $srcImpl = Join-Path $projDir 'src\imu_fusion.cpp'
    $include = Join-Path $projDir 'include'
    $outExe = Join-Path $buildDir 'imu_fusion_tests.exe'

    if (-not (Test-Path $srcTest)) { Write-Error "Missing file: $srcTest"; exit 3 }
    if (-not (Test-Path $srcImpl)) { Write-Error "Missing file: $srcImpl"; exit 3 }

    $args = @(
        '-std=c++17',
        '-O2',
        '-I', $include,
        $srcTest,
        $srcImpl,
        '-o', $outExe,
        '-DM_PI=(3.14159265358979323846)'
    )

    Write-Host "Compiling comprehensive test suite..." -NoNewline; Write-Host " (output: $outExe)"
    $cmdDisplay = "`"$gppPath`" " + ($args -join ' ')
    Write-Host "Command: $cmdDisplay"
    
    # Invoke g++ and capture output
    $compileOutput = & $gppPath @args 2>&1
    $code = $LASTEXITCODE
    
    # Force display of output
    if ($compileOutput) {
        Write-Host "=== Compiler Output ===" -ForegroundColor Cyan
        $compileOutput | ForEach-Object { 
            if ($_ -is [System.Management.Automation.ErrorRecord]) {
                Write-Host $_.Exception.Message -ForegroundColor Red
            } else {
                Write-Host $_
            }
        }
        Write-Host "=== End Compiler Output ===" -ForegroundColor Cyan
    } else {
        Write-Host "No compiler output captured (compileOutput is empty/null)." -ForegroundColor Yellow
    }
    
    # Check if executable was created (more reliable than exit code)
    if (Test-Path $outExe) {
        Write-Host "Compilation succeeded (executable created)." -ForegroundColor Green
    } elseif ($code -ne 0) {
        Write-Error "Compilation failed (exit code $code, no executable created)."
        exit $code
    } else {
        Write-Host 'Compilation succeeded.' -ForegroundColor Green
    }

    Write-Host 'Running comprehensive test suite...'
    # Add MinGW64 bin to PATH temporarily so runtime DLLs are found
    $mingwBin = Split-Path -Parent $gppPath
    $oldPath = $env:Path
    $env:Path = "$mingwBin;$env:Path"

    # Run test executable without forwarding script args (test binary runs both groups internally)
    $testOutput = & $outExe 2>&1
    $runCode = $LASTEXITCODE
    
    # Restore original PATH
    $env:Path = $oldPath
    
    if ($testOutput) {
        $testOutput | ForEach-Object { Write-Host $_ }
    } else {
        Write-Host "No test output captured." -ForegroundColor Yellow
    }
    Write-Host "Test process exited with code $runCode"
    exit $runCode

} catch {
    Write-Error "Unexpected error: $_"
    exit 4
}
