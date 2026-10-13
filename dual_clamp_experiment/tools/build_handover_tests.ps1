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
$output = Join-Path $root 'x64\Debug_handover\verification'
$objects = Join-Path $root 'obj\HandoverTests'
New-Item -ItemType Directory -Force -Path $output, $objects | Out-Null
$adsInclude = Join-Path (Split-Path $root -Parent) '64位ADS - 相对路径 - 传数组 - 加上手柄\ADS\Include'
function Build-Test([string]$name, [string[]]$sources) {
    Push-Location $objects
    try {
        & $compiler /nologo /std:c++17 /EHsc /utf-8 /O2 /MD /DWIN32_LEAN_AND_MEAN /DNOMINMAX "/I$adsInclude" "/I$root" @sources "/Fe$output\$name.exe"
        if ($LASTEXITCODE -ne 0) { throw "Build failed: $name" }
    } finally { Pop-Location }
}
Build-Test 'test_handover_hold' @("$PSScriptRoot\test_handover_hold.cpp")
& "$output\test_handover_hold.exe"
if ($LASTEXITCODE -ne 0) { throw 'Handover state tests failed' }
Build-Test 'test_clamp_recording' @("$PSScriptRoot\test_clamp_recording.cpp", "$root\ExperimentStreamRecorder.cpp",
    "$root\ForceCalibration.cpp", "$root\ProgrammedDeliveryTypes.cpp", "$root\DualClampTypes.cpp")
& "$output\test_clamp_recording.exe"
if ($LASTEXITCODE -ne 0) { throw 'Recording regression tests failed' }
foreach ($name in @('test_clamp_dynamics', 'test_force_pulse')) {
    Build-Test $name @("$PSScriptRoot\$name.cpp")
    & "$output\$name.exe"
    if ($LASTEXITCODE -ne 0) { throw "Regression tests failed: $name" }
}
