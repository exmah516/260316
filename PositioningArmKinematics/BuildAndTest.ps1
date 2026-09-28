param(
    [string]$EigenRoot = (Join-Path $PSScriptRoot '.deps/Library'),
    [string]$BuildRoot = (Join-Path $PSScriptRoot 'build-v142'),
    [ValidateSet('Release', 'RelWithDebInfo')][string]$Configuration = 'Release',
    [string]$Toolset = 'v142',
    [string]$CMake = ''
)

$ErrorActionPreference = 'Stop'
chcp 65001 | Out-Null
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$OutputEncoding = [Console]::OutputEncoding

if (-not $CMake) {
    $command = Get-Command cmake -ErrorAction SilentlyContinue
    if ($command) { $CMake = $command.Source }
    else {
        $vswhere = "${env:ProgramFiles(x86)}/Microsoft Visual Studio/Installer/vswhere.exe"
        if (Test-Path -LiteralPath $vswhere) {
            $vs = & $vswhere -latest -products '*' -property installationPath
            $CMake = Join-Path $vs 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
        }
    }
}
if (-not $CMake -or -not (Test-Path -LiteralPath $CMake)) {
    throw 'CMake was not found. Pass -CMake with its executable path.'
}
if (-not (Test-Path -LiteralPath (Join-Path $EigenRoot 'include/eigen3/Eigen'))) {
    throw 'Eigen include directory is missing; see README.md.'
}

& $CMake -S $PSScriptRoot -B $BuildRoot -G 'Visual Studio 17 2022' -A x64 -T $Toolset `
    "-DCMAKE_PREFIX_PATH=$EigenRoot" -DBUILD_TESTING=ON
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed.' }
& $CMake --build $BuildRoot --config $Configuration --parallel 2
if ($LASTEXITCODE -ne 0) { throw 'C++ build failed.' }

$ctest = Join-Path (Split-Path $CMake) 'ctest.exe'
& $ctest --test-dir $BuildRoot -C $Configuration --output-on-failure
if ($LASTEXITCODE -ne 0) {
    throw 'Offline kinematics tests failed.'
}
