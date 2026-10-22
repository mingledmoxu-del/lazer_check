param([ValidateSet('Debug','Release')][string]$Configuration = 'Debug')
$ErrorActionPreference = 'Stop'
$bundles = Join-Path $env:LOCALAPPDATA 'stm32cube/bundles'
foreach ($tool in @(@{Command='arm-none-eabi-gcc'; Bundle='gnu-tools-for-stm32'},
                    @{Command='ninja'; Bundle='ninja'}, @{Command='cmake'; Bundle='cmake'})) {
    if (-not (Get-Command $tool.Command -ErrorAction SilentlyContinue)) {
        $package = Join-Path $bundles $tool.Bundle
        $candidate = Get-ChildItem -LiteralPath $package -Directory | Sort-Object Name -Descending |
            Where-Object { Test-Path (Join-Path $_.FullName ('bin/' + $tool.Command + '.exe')) } |
            Select-Object -First 1
        if (-not $candidate) { throw "Install $($tool.Command) or add it to PATH." }
        $env:PATH = (Join-Path $candidate.FullName 'bin') + ';' + $env:PATH
    }
}
Push-Location $PSScriptRoot
try {
    cmake --preset $Configuration
    if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed' }
    cmake --build --preset $Configuration
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed' }
} finally { Pop-Location }
