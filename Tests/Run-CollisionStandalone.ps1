param([string]$OutputDirectory = "$env:TEMP\TVFCollisionStandalone")
$ErrorActionPreference = 'Stop'
$PluginRoot = Split-Path $PSScriptRoot -Parent
$VsWhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$VsRoot = & $VsWhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VsRoot) { throw 'MSVC C++ tools are required.' }
$VcVars = Join-Path $VsRoot 'VC\Auxiliary\Build\vcvars64.bat'
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$Core = Join-Path $PluginRoot 'Source\TAVisualFractureCore'
$Command = 'call "{0}" >nul && cl /nologo /std:c++20 /EHsc /O2 /I"{1}\Public" "{1}\Private\TVFMotion.cpp" "{1}\Private\TVFCollision.cpp" "{2}\CollisionStandalone.cpp" /Fe:"{3}\TVFCollisionStandalone.exe" /Fo:"{3}\\" && "{3}\TVFCollisionStandalone.exe"' -f $VcVars,$Core,$PSScriptRoot,$OutputDirectory
& cmd.exe /d /s /c $Command
if ($LASTEXITCODE -ne 0) { throw "Collision standalone failed: $LASTEXITCODE" }
