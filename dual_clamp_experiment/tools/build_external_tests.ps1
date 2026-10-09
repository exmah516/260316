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
$adsInclude = Join-Path (Split-Path $root -Parent) 'Vessel intervention Robot\260316\64位ADS - 相对路径 - 传数组 - 加上手柄\ADS\Include'
if (-not (Test-Path -LiteralPath (Join-Path $adsInclude 'ADSComm1.h'))) { throw '找不到生产工程使用的 ADSComm1.h' }
$adsIncludeForBuild = Join-Path $env:TEMP ("dual_clamp_ads_include_" + [Guid]::NewGuid().ToString("N"))
New-Item -ItemType Junction -Path $adsIncludeForBuild -Target $adsInclude | Out-Null
$flags = @('/nologo', '/std:c++17', '/EHsc', '/utf-8', '/O2', '/MD', '/DWIN32_LEAN_AND_MEAN', '/DNOMINMAX',
    "/I$adsIncludeForBuild")

function Invoke-ExternalCompiler([string[]]$sources, [string]$outputExe) {
    $response = Join-Path $env:TEMP ("dual_clamp_external_" + [Guid]::NewGuid().ToString("N") + ".rsp")
    $previousDirectory = Get-Location
    try {
        $arguments = @($flags) + ($sources | ForEach-Object { "`"$_`"" }) + "/Fe`"$outputExe`""
        [System.IO.File]::WriteAllLines($response, $arguments, [System.Text.UTF8Encoding]::new($false))
        Set-Location $objects
        & $compiler "@$response"
        if ($LASTEXITCODE -ne 0) { throw "编译失败：$outputExe" }
    }
    finally {
        Set-Location $previousDirectory
        Remove-Item -LiteralPath $response -Force -ErrorAction SilentlyContinue
    }
}

# 验证入口不链接ADS实现或生产main，不启动设备。
& python "$PSScriptRoot\test_ads_contract.py" "$verification\plc_symbols.txt"
if ($LASTEXITCODE -ne 0) { throw 'PLC ADS接口核对失败' }
Invoke-ExternalCompiler @("$PSScriptRoot\test_selfcheck_status.cpp", "$root\ProgrammedDeliveryAds.cpp") "$output\test_selfcheck_status.exe"
& "$output\test_selfcheck_status.exe" "$verification\plc_symbols.txt"
if ($LASTEXITCODE -ne 0) { throw '自检状态读取验证失败' }
Invoke-ExternalCompiler @("$PSScriptRoot\test_host_selfcheck.cpp", "$root\DualClampAds.cpp") "$output\test_host_selfcheck.exe"
& "$output\test_host_selfcheck.exe"
if ($LASTEXITCODE -ne 0) { throw '主机会话与自检请求验证失败' }
& python "$PSScriptRoot\test_handle_delivery.py" --output $verification
if ($LASTEXITCODE -ne 0) { throw 'PLC离线状态机验证失败' }
foreach ($name in @('test_external_validation', 'test_clamp_recording')) {
    Invoke-ExternalCompiler @("$PSScriptRoot\$name.cpp", "$root\ExperimentStreamRecorder.cpp",
        "$root\ForceCalibration.cpp", "$root\ProgrammedDeliveryTypes.cpp", "$root\DualClampTypes.cpp") "$output\$name.exe"
    if ($name -eq 'test_external_validation') { & "$output\$name.exe" "$verification\plc_trace.csv" $verification }
    else { & "$output\$name.exe" }
    if ($LASTEXITCODE -ne 0) { throw "$name 验证失败" }
}
foreach ($name in @('test_clamp_dynamics', 'test_clamp_illustration', 'test_force_pulse')) {
    Invoke-ExternalCompiler @("$PSScriptRoot\$name.cpp") "$output\$name.exe"
    & "$output\$name.exe"
    if ($LASTEXITCODE -ne 0) { throw "$name 验证失败" }
}
& python "$PSScriptRoot\verify_external_results.py" $verification
if ($LASTEXITCODE -ne 0) { throw '外源归档结果核对失败' }
