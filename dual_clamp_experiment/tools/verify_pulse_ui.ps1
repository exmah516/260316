param([Parameter(Mandatory=$true)][string]$Artifacts)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$exe = Join-Path $root 'AdsControlUI\bin\x64\Debug\net472\DualClampExperimentUI.exe'
$before = @(Get-Process DualClampExperiment -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)
foreach ($case in @(
    @{fixture='075539_ui.csv'; image='catheter_pulse_1280.png';width=1280;height=850},
    @{fixture='synthetic_guidewire_ui.csv';image='guidewire_pulse_1080.png';width=1080;height=700}
)) {
    $inputFile = Join-Path $Artifacts $case.fixture
    $image = Join-Path $Artifacts $case.image
    $args = '--curve-replay "' + $inputFile + '" --dynamics-replay --curve-snapshot "' +
        $image + '" --replay-width ' + $case.width + ' --replay-height ' + $case.height
    $p = Start-Process -FilePath $exe -ArgumentList $args -WindowStyle Hidden -PassThru
    if (!$p.WaitForExit(30000)) {
        Stop-Process -Id $p.Id
        throw 'Offline UI replay timeout'
    }
    if ($p.ExitCode -ne 0 -or !(Test-Path -LiteralPath $image)) {
        throw 'Offline UI replay failed'
    }
    Get-Content -LiteralPath ($image + '.json')
}
$after = @(Get-Process DualClampExperiment -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)
if ((@($before | Sort-Object) -join ',') -cne (@($after | Sort-Object) -join ',')) {
    throw 'Backend process inventory changed'
}
