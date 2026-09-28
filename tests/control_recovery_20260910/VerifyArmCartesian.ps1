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
Write-Output 'PASS: ArmCartesian XML and DT/S naming regression; not a TwinCAT compiler check.'
