param(
    [string]$Configuration = 'Release'
)
$ErrorActionPreference = 'Stop'
chcp 65001 | Out-Null
$OutputEncoding = [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$root = Join-Path $PSScriptRoot '../../64位ADS - 相对路径 - 传数组 - 加上手柄'
$header = Get-Content -LiteralPath (Join-Path $root 'vis_server.h') -Encoding UTF8 -Raw
$assembly = [Reflection.Assembly]::LoadFrom(
    (Join-Path $root "AdsControlUI/bin/x64/$Configuration/net472/AdsControlUI.exe"))
$state = $assembly.GetType('AdsControlUI.VisState', $true)
$commands = $assembly.GetType('AdsControlUI.VisCommandType', $true)
if ($state.StructLayoutAttribute.Pack -ne 1) { throw 'Pack 必须为 1' }
$sizes = @{ double = 8; 'unsigned short' = 2; int = 4; bool = 1; DWORD = 4;
    'std::uint64_t' = 8; 'std::uint32_t' = 4; 'std::int8_t' = 1; 'std::uint8_t' = 1 }
$types = @{ double = 'System.Double'; 'unsigned short' = 'System.UInt16'; int = 'System.Int32';
    bool = 'System.Boolean'; DWORD = 'System.UInt32'; 'std::uint64_t' = 'System.UInt64';
    'std::uint32_t' = 'System.UInt32'; 'std::int8_t' = 'System.SByte'; 'std::uint8_t' = 'System.Byte' }
$body = [regex]::Match($header, '(?s)struct VisState\s*\{(.*?)\};').Groups[1].Value
$body = [regex]::Replace($body, '//[^\r\n]*', '')
$fields = $state.GetFields()
$offset = 0
$index = 0
foreach ($declaration in ($body -split ';')) {
    if ([string]::IsNullOrWhiteSpace($declaration)) { continue }
    $match = [regex]::Match($declaration.Trim(), '^([\w: ]+)\s+(\w+)(?:\[(\d+)\])?$')
    if (!$match.Success) { throw "未识别的 C++ 字段: $declaration" }
    $type = $match.Groups[1].Value
    $name = $match.Groups[2].Value
    $count = if ($match.Groups[3].Success) { [int]$match.Groups[3].Value } else { 1 }
    $field = $fields[$index]
    if ($field.Name -cne $name) { throw "字段顺序不同: $name" }
    $actualOffset = [Runtime.InteropServices.Marshal]::OffsetOf($state, $name).ToInt32()
    if ($actualOffset -ne $offset) { throw "字段偏移不同: $name $actualOffset != $offset" }
    $actualType = $field.FieldType
    $marshal = $field.GetCustomAttributes([Runtime.InteropServices.MarshalAsAttribute], $false)
    if ($count -gt 1) {
        if (!$actualType.IsArray -or $marshal.Count -ne 1 -or $marshal[0].SizeConst -ne $count -or
            $marshal[0].Value -ne [Runtime.InteropServices.UnmanagedType]::ByValArray) {
            throw "数组布局不同: $name"
        }
        $actualType = $actualType.GetElementType()
    }
    if ($actualType.FullName -cne $types[$type]) { throw "字段类型不同: $name" }
    if ($type -eq 'bool') {
        $boolMarshal = if ($count -gt 1) { $marshal[0].ArraySubType } else { $marshal[0].Value }
        if ($boolMarshal -ne [Runtime.InteropServices.UnmanagedType]::I1) { throw "布尔封送不同: $name" }
    }
    $offset += $sizes[$type] * $count
    $index++
}
if ($index -ne $fields.Count -or $offset -ne 841 -or
    [Runtime.InteropServices.Marshal]::SizeOf([Activator]::CreateInstance($state)) -ne 841) { throw '字段数量或总大小不同' }
$enumBody = [regex]::Match($header, '(?s)enum class VisCommandType : int\s*\{(.*?)\};').Groups[1].Value
$enumBody = [regex]::Replace($enumBody, '//[^\r\n]*', '')
$entries = [regex]::Matches($enumBody, '(\w+)\s*=\s*(\d+)')
foreach ($entry in $entries) {
    $name = $entry.Groups[1].Value
    $value = [int]$entry.Groups[2].Value
    if ([int][Enum]::Parse($commands, $name) -ne $value) { throw "命令编号不同: $name" }
}
if ([Enum]::GetNames($commands).Count -ne $entries.Count) { throw '命令数量不同' }
foreach ($hole in @(14, 15, 16, 17, 19, 23)) {
    if ([Enum]::IsDefined($commands, $hole)) { throw "历史命令编号被复用: $hole" }
}
Write-Output "PASS: $index 个字段的类型、顺序、数组、布尔封送及偏移一致，841 字节；$($entries.Count) 个命令编号一致。"
