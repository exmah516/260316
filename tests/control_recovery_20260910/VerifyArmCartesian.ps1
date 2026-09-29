param(
    [string]$Source = (Join-Path $PSScriptRoot '../../下位机工程程序/250902/Untitled2/POUs/ArmCartesian.TcPOU')
)
$ErrorActionPreference = 'Stop'
chcp 65001 | Out-Null
$OutputEncoding = [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$document = [xml](Get-Content -LiteralPath $Source -Encoding UTF8 -Raw)
$declaration = $document.TcPlcObject.POU.Declaration.InnerText
$implementation = $document.TcPlcObject.POU.Implementation.ST.InnerText

# 只检查本次回归：TwinCAT不区分大小写，DT、S不能声明为变量。
$conflicts = [regex]::Matches($declaration, '(?im)^\s*(dt|s)\s*:') |
    ForEach-Object { $_.Groups[1].Value }
if ($conflicts) {
    throw "TwinCAT keyword used as variable: $($conflicts -join ', ')"
}
$code = [regex]::Replace($implementation, '//[^\r\n]*|\(\*[\s\S]*?\*\)', '')
if ($code -match '(?i)\b(dt|s)\b') { throw 'Old variable reference remains in ST implementation.' }
foreach ($name in @('cycle_seconds', 'path_progress')) {
    if ($declaration -notmatch "(?im)^\s*$name\s*:\s*LREAL\s*;" -or
        $code -notmatch "(?im)^\s*$name\s*:=") {
        throw "Missing LREAL declaration or assignment: $name"
    }
}

$service = Get-Content -LiteralPath (Join-Path $PSScriptRoot '../../64位ADS - 相对路径 - 传数组 - 加上手柄/arm_manual_ads_service.cpp') -Encoding UTF8 -Raw
if ($service -notmatch 'snapshot_\.motion_busy\[i\]\s*\|\|\s*std::abs\(snapshot_\.act_vel\[i\]\)\s*>\s*0\.1\)\s*return' -or
    $code -notmatch 'ABS\(G\.arm_act_vel\[i\]\)\s*>\s*0\.1' -or
    $code -notmatch 'ABS\(G\.arm_act_vel\[i\]\)\s*<=\s*0\.5') {
    throw '协调运动起动阈值应为0.1，到位阈值应为0.5。'
}
Write-Output 'PASS: ArmCartesian XML and DT/S naming regression; not a TwinCAT compiler check.'
