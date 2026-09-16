$ErrorActionPreference = "Stop"

if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
    throw "Cargo is not installed. Install Rust from https://rustup.rs/."
}

$root = $PSScriptRoot
$project = Join-Path $root "foveated_lidar_gui"
$dist = Join-Path $root "dist"

Push-Location $project
try {
    cargo build --release
    if ($LASTEXITCODE -ne 0) {
        throw "Cargo release build failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}

New-Item -ItemType Directory -Force -Path $dist | Out-Null
$output = Join-Path $dist "FoveatedLiDAR.exe"
$builtBinary = Join-Path $project "target\release\foveated_lidar_gui.exe"
Copy-Item $builtBinary $output -Force

$installDir = Join-Path $env:LOCALAPPDATA "FoveatedLiDAR"
New-Item -ItemType Directory -Force -Path $installDir | Out-Null
$target = Join-Path $installDir "FoveatedLiDAR.exe"
Copy-Item $output $target -Force

$shell = New-Object -ComObject WScript.Shell
$shortcutPath = Join-Path $env:USERPROFILE "Desktop\Tactical Mapper.lnk"
$shortcut = $shell.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $target
$shortcut.WorkingDirectory = $installDir
$shortcut.Description = "Tactical Mapper"
$shortcut.Save()

Write-Host "Built: $output"
Write-Host "Installed: $target"
Write-Host "Desktop shortcut created: $shortcutPath"
