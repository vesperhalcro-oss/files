$ErrorActionPreference = "Stop"

if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
    throw "Cargo is not installed. Install Rust from https://rustup.rs/."
}
if (-not (Test-Path "foveated_lidar_gui\Cargo.toml")) {
    throw "foveated_lidar_gui\Cargo.toml was not found."
}
if (-not (Test-Path ".env")) {
    Write-Warning ".env was not found. Copy .env.example to .env before starting."
}

Push-Location "foveated_lidar_gui"
try { cargo run --release }
finally { Pop-Location }
