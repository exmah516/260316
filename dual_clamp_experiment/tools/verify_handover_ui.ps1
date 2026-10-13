param([string]$Artifacts = '', [string]$Executable = '')
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (!$Artifacts) { $Artifacts = Join-Path $root 'analysis\handover_hold_20261013' }
if (!$Executable) { $Executable = Join-Path $root 'AdsControlUI\bin\Debug\net472\DualClampExperimentUI.exe' }
$before = @(Get-Process DualClampExperiment -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)
$cases = @()
foreach ($directory in Get-ChildItem $Artifacts -Directory) {
    if (Test-Path (Join-Path $directory.FullName 'ui_fixture.csv')) {
        $cases += @{input=(Join-Path $directory.FullName 'ui_fixture.csv');image=(Join-Path $directory.FullName 'ui_hold_1280.png');width=1280;height=850;hold=$true}
    }
}
$first = $cases[0]
$cases += @{input=$first.input;image=(Join-Path $Artifacts 'ui_hold_1080.png');width=1080;height=700;hold=$true}
foreach ($width in @(13,17,23)) {
    $cases += @{input=(Join-Path (Split-Path $first.input -Parent) "ui_legacy_$width.csv");image=(Join-Path $Artifacts "ui_legacy_$width.png");width=1280;height=850;hold=$false}
}
foreach ($case in $cases) {
    $arguments = '--curve-replay "' + $case.input + '" --dynamics-replay --curve-snapshot "' +
        $case.image + '" --replay-width ' + $case.width + ' --replay-height ' + $case.height
    if ($case.hold) { $arguments += ' --handover-replay' }
    $process = Start-Process -FilePath $Executable -ArgumentList $arguments -WindowStyle Hidden -PassThru
    if (!$process.WaitForExit(30000)) {
        Stop-Process -Id $process.Id
        throw 'Offline handover UI replay timed out'
    }
    if ($process.ExitCode -ne 0 -or !(Test-Path -LiteralPath $case.image)) { throw 'Offline UI replay failed' }
    $result = Get-Content -Encoding UTF8 -LiteralPath ($case.image + '.json') | ConvertFrom-Json
    if ($result.handover_visible -ne $case.hold -or $result.points -eq 0) { throw 'UI hold visibility mismatch' }
    Write-Output ((Split-Path $case.image -Leaf) + ': PASS')
}
$after = @(Get-Process DualClampExperiment -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)
if ((@($before | Sort-Object) -join ',') -cne (@($after | Sort-Object) -join ',')) { throw 'Backend process inventory changed' }
