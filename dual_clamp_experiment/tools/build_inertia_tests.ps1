param([string]$VisualStudio = 'D:\Work_software\VS2022')
$ErrorActionPreference = 'Stop'
chcp 65001 | Out-Null
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$root = Split-Path $PSScriptRoot -Parent
$msvc = Get-ChildItem "$VisualStudio\VC\Tools\MSVC" -Directory | Sort-Object Name -Descending | Select-Object -First 1
$sdkRoot = "${env:ProgramFiles(x86)}\Windows Kits\10"
$sdk = Get-ChildItem "$sdkRoot\Lib" -Directory | Sort-Object Name -Descending | Select-Object -First 1
$env:INCLUDE = "$($msvc.FullName)\include;$sdkRoot\Include\$($sdk.Name)\ucrt;$sdkRoot\Include\$($sdk.Name)\shared;$sdkRoot\Include\$($sdk.Name)\um"
$env:LIB = "$($msvc.FullName)\lib\x64;$($sdk.FullName)\ucrt\x64;$($sdk.FullName)\um\x64"
$compiler = "$($msvc.FullName)\bin\Hostx64\x64\cl.exe"
$output = Join-Path $root 'x64\Debug_inertia25g'
$objects = Join-Path $root 'obj\InertiaTests'
New-Item -ItemType Directory -Force -Path $output, $objects | Out-Null
$flags = @('/nologo', '/std:c++17', '/EHsc', '/utf-8', '/O2', '/MD', '/DWIN32_LEAN_AND_MEAN',
    '/DNOMINMAX', "/I$(Join-Path (Split-Path $root -Parent) '64位ADS - 相对路径 - 传数组 - 加上手柄\ADS\Include')", "/Fo$objects\")
# 测试只链接计算和文件记录实现，不链接ADS通信实现或生产入口。
& $compiler @flags "$PSScriptRoot\test_force_pulse.cpp" "/Fe$output\test_force_pulse.exe"
if ($LASTEXITCODE -ne 0) { throw 'Pulse test build failed' }
$debugFlags = @($flags | Where-Object { $_ -ne '/O2' }) + '/Od'
& $compiler @debugFlags "$PSScriptRoot\test_force_pulse.cpp" "/Fe$output\test_force_pulse_debug.exe"
if ($LASTEXITCODE -ne 0) { throw 'Debug pulse test build failed' }
& $compiler @flags "$PSScriptRoot\test_clamp_dynamics.cpp" "/Fe$output\test_clamp_dynamics.exe"
if ($LASTEXITCODE -ne 0) { throw '动力学测试构建失败' }
& $compiler @flags "$PSScriptRoot\test_clamp_recording.cpp" "$root\ExperimentStreamRecorder.cpp" `
    "$root\ForceCalibration.cpp" "$root\ProgrammedDeliveryTypes.cpp" "$root\DualClampTypes.cpp" `
    "/Fe$output\test_clamp_recording.exe"
if ($LASTEXITCODE -ne 0) { throw '记录测试构建失败' }
& $compiler @flags "$PSScriptRoot\test_clamp_illustration.cpp" "/Fe$output\test_clamp_illustration.exe"
if ($LASTEXITCODE -ne 0) { throw '模型2测试构建失败' }
