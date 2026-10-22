$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$outDir = Join-Path $firmwareRoot 'build/host_tests'
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
$compiler = (Get-Command gcc -ErrorAction Stop).Source
$common = @('Communication/Src/command_manager.c', 'Communication/Src/axis_motion.c',
    'Communication/Src/limit_switch.c', 'Communication/Src/axis_guard.c',
    'Communication/Src/axis_guard_controller.c', 'Communication/Src/trajectory.c')
$cases = @(
    @{Name='stop'; File='tests/test_stop.c'; Sources=$common; Defines=@()},
    @{Name='no_limits'; File='tests/test_limits.c'; Sources=$common; Defines=@()},
    @{Name='legacy_limits'; File='tests/test_limits.c'; Sources=$common; Defines=@('-DMOTOR_LIMITS_ENABLED=1')},
    @{Name='guard'; File='tests/test_guard.c'; Sources=$common; Defines=@()},
    @{Name='trajectory'; File='tests/test_trajectory.c'; Sources=$common; Defines=@()},
    @{Name='trajectory_protocol'; File='tests/test_trajectory_protocol.c'; Sources=@('Communication/Src/pc_protocol.c','Communication/Src/crc16.c'); Defines=@()},
    @{Name='axis_motion'; File='tests/test_axis_motion.c'; Sources=@('Communication/Src/axis_motion.c'); Defines=@()},
    @{Name='limit_debug'; File='tests/test_limit_debug.c'; Sources=@('Communication/Src/limit_debug.c','Communication/Src/limit_switch.c'); Defines=@()}
)
foreach ($case in $cases) {
    $target = Join-Path $outDir ($case.Name + '.exe')
    $arguments = @('-std=c11','-Wall','-Wextra','-Werror',
        ('-I' + (Join-Path $firmwareRoot 'tests/stubs')),
        ('-I' + (Join-Path $firmwareRoot 'Communication/Inc')),
        ('-I' + (Join-Path $firmwareRoot 'Motor_driver/Inc'))) + $case.Defines
    $arguments += Join-Path $firmwareRoot $case.File
    $arguments += @($case.Sources | ForEach-Object { Join-Path $firmwareRoot $_ })
    $arguments += @('-o', $target)
    & $compiler @arguments
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($case.Name)" }
    & $target
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $($case.Name)" }
}
