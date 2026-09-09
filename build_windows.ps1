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
Copy-Item (Join-Path $project "target\release\foveated_lidar_gui.exe") $output -Force
Write-Host "Built single-file installer: $output"
Write-Host "Copy FoveatedLiDAR.exe to any Windows machine and double-click it."
