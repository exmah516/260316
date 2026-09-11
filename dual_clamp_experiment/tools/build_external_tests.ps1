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
$output = Join-Path $root 'x64\Debug_external'
$objects = Join-Path $root 'obj\ExternalTests'
$verification = Join-Path $output 'verification'
New-Item -ItemType Directory -Force -Path $output, $objects, $verification | Out-Null
$flags = @('/nologo', '/std:c++17', '/EHsc', '/utf-8', '/O2', '/MD', '/DWIN32_LEAN_AND_MEAN',
    '/DNOMINMAX', "/I$(Join-Path (Split-Path $root -Parent) '64位ADS - 相对路径 - 传数组 - 加上手柄\ADS\Include')", "/Fo$objects\")
# 验证入口不链接ADS实现或生产main，不启动设备。
& python "$PSScriptRoot\test_external_plc.py" --output $verification
if ($LASTEXITCODE -ne 0) { throw 'PLC离线状态机验证失败' }
foreach ($name in @('test_external_validation', 'test_clamp_recording')) {
    & $compiler @flags "$PSScriptRoot\$name.cpp" "$root\ExperimentStreamRecorder.cpp" `
        "$root\ForceCalibration.cpp" "$root\ProgrammedDeliveryTypes.cpp" "$root\DualClampTypes.cpp" "/Fe$output\$name.exe"
    if ($LASTEXITCODE -ne 0) { throw "$name 构建失败" }
    if ($name -eq 'test_external_validation') { & "$output\$name.exe" "$verification\plc_trace.csv" $verification }
    else { & "$output\$name.exe" }
    if ($LASTEXITCODE -ne 0) { throw "$name 验证失败" }
}
foreach ($name in @('test_clamp_dynamics', 'test_clamp_illustration', 'test_force_pulse')) {
    & $compiler @flags "$PSScriptRoot\$name.cpp" "/Fe$output\$name.exe"
    if ($LASTEXITCODE -ne 0) { throw "$name 构建失败" }
    & "$output\$name.exe"
    if ($LASTEXITCODE -ne 0) { throw "$name 验证失败" }
}
& python "$PSScriptRoot\verify_external_results.py" $verification
if ($LASTEXITCODE -ne 0) { throw '外源归档结果核对失败' }
