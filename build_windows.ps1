$ErrorActionPreference = "Stop"

if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
    throw "Cargo is not installed. Install Rust from https://rustup.rs/."
}

$root = $PSScriptRoot
$project = Join-Path $root "foveated_lidar_gui"
$dist = Join-Path $root "dist\FoveatedLiDAR"

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
Copy-Item (Join-Path $project "target\release\foveated_lidar_gui.exe") (Join-Path $dist "FoveatedLiDAR.exe") -Force
Copy-Item (Join-Path $root "install_windows.ps1") $dist -Force
Write-Host "Built $dist\FoveatedLiDAR.exe"
Write-Host "To install it for the current user, run: .\install_windows.ps1"
