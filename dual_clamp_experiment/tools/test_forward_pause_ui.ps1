param([string]$AssemblyPath, [string]$PreviewPath)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName PresentationFramework, PresentationCore, WindowsBase
$assembly = [Reflection.Assembly]::LoadFrom((Resolve-Path -LiteralPath $AssemblyPath))
$app = [Activator]::CreateInstance($assembly.GetType('DualClampExperimentUI.App'))
$app.InitializeComponent()
$window = [Activator]::CreateInstance($assembly.GetType('DualClampExperimentUI.MainWindow'))
$flags = [Reflection.BindingFlags]'Instance,NonPublic'
$type = $window.GetType()
function Invoke-Private([string]$name, [object[]]$values) { $type.GetMethod($name, $flags).Invoke($window, $values) | Out-Null }
function Check([bool]$ok, [string]$message) { if (-not $ok) { throw $message } }

# 不Show、不Run、不触发Loaded，因此不会启动后端或访问设备。
$mode = $window.FindName('ExperimentModeBox')
$mode.SelectedItem = $mode.Items[1]
Invoke-Private 'UpdateModeView' @()
$panel = $window.FindName('ProgramForwardPausePanel')
$enabled = $window.FindName('ProgramForwardPauseEnabled')
$distance = $window.FindName('ProgramForwardPauseDistance')
$seconds = $window.FindName('ProgramForwardPauseSeconds')
Check ($panel.Visibility -eq 'Visible' -and -not $enabled.IsChecked -and $distance.Text -eq '10' -and $seconds.Text -eq '3') "Default pause controls mismatch: $($panel.Visibility), $($enabled.IsChecked), $($distance.Text), $($seconds.Text)"
foreach ($index in @(2, 3)) {
    $mode.SelectedIndex = $index
    Invoke-Private 'UpdateModeView' @()
    Check ($panel.Visibility -eq 'Collapsed') 'Pause controls visible outside catheter mode'
}
$mode.SelectedIndex = 1
Invoke-Private 'UpdateModeView' @()
$type.GetField('_motionSessionReady', $flags).SetValue($window, $true)
$state = @('0') * 59
$state[0] = 'PROGRAM_STATE'; $state[1] = '1'; $state[40] = '1'; $state[41] = ''; $state[51] = ''
foreach ($phase in @(0, 1, 2, 4, 9, 10, 11, 12)) {
    $state[2] = [string]$phase
    $state[5] = if ($phase -eq 1) { '1' } else { '0' }
    $state[53] = if ($phase -in @(4, 9)) { '3' } else { '0' }
    Invoke-Private 'ParseProgramState' @(($state -join '|'))
    Check ($panel.IsEnabled -eq ($phase -eq 0 -or $phase -ge 10)) 'Pause settings lock mismatch'
    if ($phase -in @(4, 9)) {
        Check ($window.FindName('PhaseText').Text -eq '前进定点停留') 'Pause status text mismatch'
    }
}
$enabled.IsChecked = $true
$panel.Background = [Windows.Media.Brushes]::White
$panel.Measure([Windows.Size]::new(330, 120))
$panel.Arrange([Windows.Rect]::new(0, 0, 330, 120))
$panel.UpdateLayout()
if ($PreviewPath) {
    $bitmap = [Windows.Media.Imaging.RenderTargetBitmap]::new(330, 120, 96, 96, [Windows.Media.PixelFormats]::Pbgra32)
    $bitmap.Render($panel)
    $encoder = [Windows.Media.Imaging.PngBitmapEncoder]::new()
    $encoder.Frames.Add([Windows.Media.Imaging.BitmapFrame]::Create($bitmap))
    $stream = [IO.File]::Create($PreviewPath)
    try { $encoder.Save($stream) } finally { $stream.Dispose() }
}
Check (-not $type.GetField('_loaded', $flags).GetValue($window)) 'UI test unexpectedly loaded live window'
Write-Output 'PASS WPF XAML, defaults, mode visibility, settings lock and pause status; no backend started'
