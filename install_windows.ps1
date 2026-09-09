$ErrorActionPreference = "Stop"

$source = Join-Path $PSScriptRoot "FoveatedLiDAR.exe"
if (-not (Test-Path $source)) {
    throw "FoveatedLiDAR.exe was not found. Run .\build_windows.ps1 first."
}

$installDir = Join-Path $env:LOCALAPPDATA "FoveatedLiDAR"
New-Item -ItemType Directory -Force -Path $installDir | Out-Null
$target = Join-Path $installDir "FoveatedLiDAR.exe"
Copy-Item $source $target -Force

$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut((Join-Path $env:USERPROFILE "Desktop\Foveated LiDAR.lnk"))
$shortcut.TargetPath = $target
$shortcut.WorkingDirectory = $installDir
$shortcut.Description = "Foveated LiDAR Map Simulator"
$shortcut.Save()

Write-Host "Installed to $target"
Write-Host "Desktop shortcut created."
