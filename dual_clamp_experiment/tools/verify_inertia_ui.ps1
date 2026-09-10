param([string]$UiExecutable = '')
$ErrorActionPreference = 'Stop'
chcp 65001 | Out-Null
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$root = Split-Path $PSScriptRoot -Parent
$base = Join-Path $root 'x64\Debug_inertia25g'
$verification = Get-Content -LiteralPath (Join-Path $base 'verification.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$exe = Join-Path $root 'AdsControlUI\bin\Inertia25g\DualClampExperimentUI.exe'
if ($UiExecutable) { $exe = (Resolve-Path -LiteralPath $UiExecutable).Path }
$before = @(Get-Process DualClampExperiment -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)
$cases = @(
    @{directory=$verification.recording_tests[0]; name='catheter_1280'; width=1280; height=850; extra=''},
    @{directory=$verification.recording_tests[7]; name='guidewire_1080'; width=1080; height=700;
      extra='--validation-replay --negative-sign-replay --model2-replay'}
)
foreach ($case in $cases) {
    $fixture = Join-Path $case.directory 'ui_fixture.csv'
    $png = Join-Path $base ($case.name + '.png')
    $arguments = '--curve-replay "' + $fixture + '" --dynamics-replay --curve-snapshot "' + $png +
        '" --replay-width ' + $case.width + ' --replay-height ' + $case.height + ' ' + $case.extra
    # 只允许带离线回放参数的入口；超时仅结束本脚本创建的进程。
    $process = Start-Process -FilePath $exe -ArgumentList $arguments -WindowStyle Hidden -PassThru
    if (!$process.WaitForExit(30000)) {
        Stop-Process -Id $process.Id
        throw '离线界面回放超时'
    }
    if ($process.ExitCode -ne 0 -or !(Test-Path -LiteralPath $png)) { throw '离线界面回放失败' }
    $snapshot = Get-Content -LiteralPath ($png + '.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if (!$snapshot.dynamics_ui_tests_passed -or $snapshot.hardware_connected -or $snapshot.points -ne 600) {
        throw '离线界面断言失败'
    }
    $snapshot | ConvertTo-Json -Compress
}
$after = @(Get-Process DualClampExperiment -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)
if ((@($before | Sort-Object) -join ',') -cne (@($after | Sort-Object) -join ',')) {
    throw '后端进程状态发生变化，请检查是否有其他程序启动后端'
}
