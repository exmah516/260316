param(
    [string]$MasterAddress = '192.168.50.1',
    [ValidateRange(1024,65535)][int]$Port = 32110,
    [switch]$CheckInvalidFrames
)
$ErrorActionPreference = 'Stop'
$client = New-Object System.Net.Sockets.TcpClient
$writer = $null
$result = 1
try {
    $connecting = $client.ConnectAsync($MasterAddress, $Port)
    if (!$connecting.Wait(5000)) { throw 'Connection timed out; start receiver and check firewall.' }
    $client.NoDelay = $true
    $stream = $client.GetStream()
    $stream.ReadTimeout = 5000
    $stream.WriteTimeout = 5000
    $writer = New-Object System.IO.BinaryWriter($stream)
    function Send-Pose([double[]]$Values, [int]$Expected, [int]$Magic = 0x314B5456) {
        if ($Values.Length -ne 11) { throw 'Expected 11 display values.' }
        $writer.Write($Magic)
        foreach ($value in $Values) { $writer.Write([double]$value) }
        $writer.Flush()
        $reply = $stream.ReadByte()
        if ($reply -ne $Expected) { throw "Unexpected display acknowledgement: $reply; expected $Expected" }
    }
    if ($CheckInvalidFrames) {
        Send-Pose (New-Object double[] 11) 0 0
        $bad = New-Object double[] 11
        $bad[0] = [double]::NaN
        Send-Pose $bad 0
        $bad[0] = 1
        Send-Pose $bad 0
        Write-Output 'PASS: wrong magic / NaN / excessive display value rejected'
    }
    Send-Pose (New-Object double[] 11) 1
    for ($joint = 0; $joint -lt 9; $joint++) {
        $name = if ($joint -lt 7) { 'sj' + ($joint + 1) } else { 'xj' + ($joint - 6) }
        $amplitude = if ($joint -in @(0,1,5)) { 0.08 } else { 0.35 }
        for ($step = 1; $step -le 32; $step++) {
            $values = New-Object double[] 11
            if ($step -ne 32) { $values[$joint] = $amplitude * [Math]::Sin($step * 2 * [Math]::PI / 32) }
            $expected = if ($joint -eq 8 -and $step -eq 32) { 2 } else { 1 }
            Send-Pose $values $expected
            Start-Sleep -Milliseconds 25
        }
        Write-Output "PASS: $name sent and display application acknowledged"
    }
    Write-Output 'PASS: receiver verified 9/9 model joints and return to zero. No robot commands sent.'
    $result = 0
}
catch { Write-Output "FAIL: $($_.Exception.Message)" }
finally {
    if ($writer) { $writer.Dispose() }
    $client.Close()
}
exit $result
