$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$outDir = Join-Path $firmwareRoot 'build/host_tests'
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
$compiler = (Get-Command gcc -ErrorAction Stop).Source
$common = @('Communication/Src/command_manager.c', 'Communication/Src/axis_motion.c',
    'Communication/Src/limit_switch.c', 'Communication/Src/axis_guard.c',
    'Communication/Src/axis_guard_controller.c', 'Communication/Src/trajectory.c')
$lwip = @(Get-ChildItem -Path (Join-Path $firmwareRoot 'Middlewares/Third_Party/LwIP/src/core/*.c'),
    (Join-Path $firmwareRoot 'Middlewares/Third_Party/LwIP/src/core/ipv4/*.c') |
    ForEach-Object { $_.FullName.Substring($firmwareRoot.Length + 1) })
$networkSources = $lwip + @('Middlewares/Third_Party/LwIP/src/netif/ethernet.c',
    'Communication/Src/pc_protocol.c','Communication/Src/crc16.c')
$cases = @(
    @{Name='p100s'; File='tests/test_p100s.c'; Sources=@('Motor_driver/Src/p100s.c','Communication/Src/modbus_rtu.c','Communication/Src/crc16.c'); Defines=@()},
    @{Name='stop'; File='tests/test_stop.c'; Sources=$common; Defines=@()},
    @{Name='position_execution'; File='tests/test_position_execution.c'; Sources=$common; Defines=@()},
    @{Name='point_tracking'; File='tests/test_point_tracking.c'; Sources=$common; Defines=@()},
    @{Name='point_latency'; File='tests/test_point_latency.c'; Sources=$common; Defines=@()},
    @{Name='point_bank'; File='tests/test_point_bank.c'; Sources=$common; Defines=@()},
    @{Name='no_limits'; File='tests/test_limits.c'; Sources=$common; Defines=@()},
    @{Name='legacy_limits'; File='tests/test_limits.c'; Sources=$common; Defines=@('-DMOTOR_LIMITS_ENABLED=1')},
    @{Name='guard'; File='tests/test_guard.c'; Sources=$common; Defines=@()},
    @{Name='dual_limits'; File='tests/test_dual_limits.c'; Sources=$common; Defines=@()},
    @{Name='network'; File='tests/test_network.c'; Sources=$networkSources; Defines=@()},
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
        ('-I' + (Join-Path $firmwareRoot 'Network/Inc')),
        ('-I' + (Join-Path $firmwareRoot 'Middlewares/Third_Party/LwIP/src/include')),
        ('-I' + (Join-Path $firmwareRoot 'Motor_driver/Inc'))) + $case.Defines
    $arguments += Join-Path $firmwareRoot $case.File
    $arguments += @($case.Sources | ForEach-Object { Join-Path $firmwareRoot $_ })
    $arguments += @('-o', $target)
    & $compiler @arguments
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($case.Name)" }
    & $target
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $($case.Name)" }
}
