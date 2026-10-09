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
$output = Join-Path $root 'x64\Debug_pause\verification'
$objects = Join-Path $root 'obj\PauseTests'
New-Item -ItemType Directory -Force -Path $output, $objects | Out-Null
$adsInclude = Join-Path (Split-Path $root -Parent) '64位ADS - 相对路径 - 传数组 - 加上手柄\ADS\Include'

function Build-Test([string]$name, [string[]]$sources) {
    $response = Join-Path $objects "$name.rsp"
    $arguments = @('/nologo', '/std:c++17', '/EHsc', '/utf-8', '/O2', '/MD', '/DWIN32_LEAN_AND_MEAN', '/DNOMINMAX', "/I`"$adsInclude`"", "/I`"$root`"", "/I`"$objects`"")
    $arguments += $sources | ForEach-Object { "`"$_`"" }
    $arguments += @("/Fe`"$output\$name.exe`"", '/link', '/OPT:REF', '/OPT:ICF')
    [System.IO.File]::WriteAllText($response, ($arguments -join ' '), [System.Text.UTF8Encoding]::new($true))
    Push-Location $objects
    try {
        & $compiler "@$response"
        if ($LASTEXITCODE -ne 0) { throw "编译失败：$name" }
    } finally { Pop-Location }
}

& python -B "$PSScriptRoot\test_forward_pause.py" --output $output
if ($LASTEXITCODE -ne 0) { throw '暂停PLC状态机测试失败' }
# 仅抽取生产函数体以避开设备线程和ADS入口；不复制维护另一份业务逻辑。
$pipe = [System.IO.File]::ReadAllText((Join-Path $root 'DualClampPipe.cpp'))
$controller = [System.IO.File]::ReadAllText((Join-Path $root 'ProgrammedDeliveryController.cpp'))
$parserEnd = $pipe.IndexOf('DualClampPipeServer::DualClampPipeServer()')
$validatorStart = $controller.IndexOf('bool ProgrammedDeliveryController::validate_config(')
$validatorEnd = $controller.IndexOf('bool ProgrammedDeliveryController::prepare(', $validatorStart)
if ($parserEnd -lt 0 -or $validatorStart -lt 0 -or $validatorEnd -lt 0) { throw '生产函数边界发生变化，请更新测试提取入口' }
$validator = $controller.Substring($validatorStart, $validatorEnd - $validatorStart).Replace('ProgrammedDeliveryController::validate_config', 'validate_program_config').Replace(') const', ')')
[System.IO.File]::WriteAllText((Join-Path $objects 'pause_host_functions.inc'), ($pipe.Substring(0, $parserEnd) + $validator), [System.Text.UTF8Encoding]::new($true))
Build-Test 'test_forward_pause_host' @("$PSScriptRoot\test_forward_pause_host.cpp", "$root\ProgrammedDeliveryAds.cpp")
& "$output\test_forward_pause_host.exe" "$output\pause_symbols.txt"
if ($LASTEXITCODE -ne 0) { throw '暂停管道与ADS测试失败' }
Build-Test 'test_forward_pause_recording' @("$PSScriptRoot\test_forward_pause_recording.cpp", "$root\ExperimentStreamRecorder.cpp",
    "$root\ForceCalibration.cpp", "$root\ProgrammedDeliveryTypes.cpp", "$root\DualClampTypes.cpp")
& "$output\test_forward_pause_recording.exe" "$output\pause_trace.csv"
if ($LASTEXITCODE -ne 0) { throw '长时间暂停归档测试失败' }
