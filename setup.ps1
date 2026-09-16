<#
.SYNOPSIS
    Builds Foveated LiDAR and installs it for the current user.

.DESCRIPTION
    Single-step replacement for build_windows.ps1 + install_windows.ps1.
    Compiles the release binary, installs it under %LOCALAPPDATA%,
    and creates a desktop shortcut. No PostgreSQL, Python, or manual
    steps required - the app manages its own embedded SQLite database.

.USAGE
    .\setup.ps1
#>

$ErrorActionPreference = "Stop"

$AppName    = "Tactical Mapper"
$ExeName    = "FoveatedLiDAR.exe"
$Root       = $PSScriptRoot
$ProjectDir = Join-Path $Root "foveated_lidar_gui"
$InstallDir = Join-Path $env:LOCALAPPDATA "FoveatedLiDAR"

function Assert-Cargo {
    if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
        throw "Cargo is not installed. Install Rust from https://rustup.rs/ and re-run this script."
    }
}

function Build-Release {
    Write-Host "Building release binary..." -ForegroundColor Cyan
    Push-Location $ProjectDir
    try {
        cargo build --release
        if ($LASTEXITCODE -ne 0) {
            throw "cargo build failed (exit code $LASTEXITCODE)."
        }
    }
    finally {
        Pop-Location
    }
}

function Install-App {
    Write-Host "Installing to $InstallDir..." -ForegroundColor Cyan

    $builtExe = Join-Path $ProjectDir "target\release\foveated_lidar_gui.exe"
    if (-not (Test-Path $builtExe)) {
        throw "Build output not found at $builtExe."
    }

    New-Item -ItemType Directory -Force -Path $InstallDir | Out-Null
    $target = Join-Path $InstallDir $ExeName
    Copy-Item $builtExe $target -Force

    $shell    = New-Object -ComObject WScript.Shell
    $shortcut = $shell.CreateShortcut((Join-Path $env:USERPROFILE "Desktop\$AppName.lnk"))
    $shortcut.TargetPath       = $target
    $shortcut.WorkingDirectory = $InstallDir
    $shortcut.Description      = "$AppName - offline LiDAR mapping"
    $shortcut.Save()

    return $target
}

Assert-Cargo
Build-Release
$installedPath = Install-App

Write-Host ""
Write-Host "$AppName installed successfully." -ForegroundColor Green
Write-Host "  Executable: $installedPath"
Write-Host "  Shortcut:   Desktop\$AppName.lnk"
Write-Host "  Database:   %LOCALAPPDATA%\FoveatedLiDAR\foveated_lidar.sqlite3 (created on first run)"
