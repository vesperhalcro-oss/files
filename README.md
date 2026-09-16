# Tactical Mapper

> Real-time local mapping when GPS is unavailable.

Tactical Mapper is an offline-first LiDAR visualization tool for robotics teams,
field operators, and developers who need a clear view of vehicle position, scan
returns, and map reconstruction without relying on a network connection.

[![Live demo](https://img.shields.io/badge/live%20demo-Vercel-111827?logo=vercel)](https://vercel-hosting-setup-and-link.vercel.app/)
[![Rust](https://img.shields.io/badge/built%20with-Rust-orange?logo=rust)](https://www.rust-lang.org/)
[![Version](https://img.shields.io/badge/version-0.1.0-22c55e)](foveated_lidar_gui/Cargo.toml)

## Explore the product

Open the [live Tactical Mapper demo](https://vercel-hosting-setup-and-link.vercel.app/)
to see a simulated LiDAR map, moving vehicle marker, scan returns, telemetry,
pause/resume controls, and speed control in your browser.

The Windows application provides the same workflow as a native desktop tool,
with local SQLite persistence and no network service required.

## Download and run

Run the one-step setup script from a Windows checkout:

```powershell
.\setup.ps1
```

It builds the release binary, installs it for the current user, and creates a
**Tactical Mapper** desktop shortcut. The local database is created automatically
on first launch.

## Run the GUI and mock simulation

### From the downloaded executable

If you are not a developer, follow these steps:

1. Download `FoveatedLiDAR.exe` from the project release.
2. Open your **Downloads** folder and double-click `FoveatedLiDAR.exe`.
3. If Windows shows a security warning, select **More info**, then **Run anyway**.
4. The first launch installs the application and creates a desktop shortcut.
5. Open **Tactical Mapper** from the desktop whenever you want to use it.

The application stores its database at
`%LOCALAPPDATA%\FoveatedLiDAR\foveated_lidar.sqlite3`.

### From the source tree

```powershell
.\run_project.ps1
```

The window starts with a local mock LiDAR stream, a moving vehicle, a live map,
pause/resume, speed control, reset, and telemetry. Data is saved automatically
to the embedded local database.

Or run directly:

```powershell
Push-Location foveated_lidar_gui
cargo run --release
Pop-Location
```

## Build and install on Windows

Generated binaries are intentionally not committed to source control. Build the
release executable with:

```powershell
.\setup.ps1
```

This builds `foveated_lidar_gui\target\release\foveated_lidar_gui.exe`, installs it to
`%LOCALAPPDATA%\FoveatedLiDAR`, and creates a **Tactical Mapper** desktop
shortcut in one step. The application creates its local SQLite database on
startup. PostgreSQL is not required.
