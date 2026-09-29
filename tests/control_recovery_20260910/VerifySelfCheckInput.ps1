$ErrorActionPreference = 'Stop'
chcp 65001 | Out-Null
$OutputEncoding = [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$pou = Join-Path $PSScriptRoot '../../下位机工程程序/250902/Untitled2/POUs'
$main = [xml](Get-Content -LiteralPath (Join-Path $pou 'MAIN.TcPOU') -Raw -Encoding UTF8)
$selfCheck = [xml](Get-Content -LiteralPath (Join-Path $pou 'SelfCheck.TcPOU') -Raw -Encoding UTF8)
$declaration = $selfCheck.TcPlcObject.POU.Declaration.'#cdata-section'
$implementation = $main.TcPlcObject.POU.Implementation.ST.'#cdata-section'
if ($declaration -notmatch '(?s)VAR_INPUT\s+init_target_from_left\s*:\s*ARRAY\[1\.\.7\] OF LREAL') {
    throw '自检目标必须是 SelfCheck 的 VAR_INPUT。'
}
if ($implementation -notmatch 'selfcheck_target_latched\s*:=\s*G\.selfcheck_target_from_left' -or
    $implementation -notmatch 'self_check_\(init_target_from_left\s*:=\s*selfcheck_target_latched\)' -or
    $implementation -match 'self_check_\.init_target_from_left\s*:=') {
    throw 'MAIN 必须在自检开始时锁存目标，再通过功能块输入传入。'
}
Write-Output 'PASS: SelfCheck 目标输入接口和 MAIN 调用方式一致。'
