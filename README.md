# Tactical Mapper

> Real-time local mapping when GPS is unavailable.

Tactical Mapper is a local LiDAR visualization tool for robotics teams,
field operators, and developers who need a clear view of vehicle position, scan
returns, semantic terrain labels, and map reconstruction. The native prototype
replays the checked-in LiDAR dataset at `foveated_lidar_gui/src/mock_lidar_stream.json`
through the same foveated mapping pipeline used for live input.

[![Live demo](https://img.shields.io/badge/live%20demo-Vercel-111827?logo=vercel)](https://vercel-hosting-setup-and-link.vercel.app/)
[![Rust](https://img.shields.io/badge/built%20with-Rust-orange?logo=rust)](https://www.rust-lang.org/)
[![Version](https://img.shields.io/badge/version-0.1.0-22c55e)](foveated_lidar_gui/Cargo.toml)

## Explore the product

Open the [live Tactical Mapper demo](https://vercel-hosting-setup-and-link.vercel.app/)
to see a simulated LiDAR map, moving vehicle marker, scan returns, telemetry,
pause/resume controls, and speed control in your browser.

The Windows application provides the same workflow as a native desktop tool,
with asynchronous PostgreSQL persistence on the local machine.

## Download and run

Run the one-step setup script from a Windows checkout:

```powershell
.\setup.ps1
```

It builds the release binary, installs it for the current user, and creates a
**Tactical Mapper** desktop shortcut. PostgreSQL must be running locally before
launching the application; see [`db/README.md`](db/README.md).

## Run the GUI and mock simulation

### From the downloaded executable

If you are not a developer, follow these steps:

1. Download `TacticalMapper.exe` from the project release.
2. Open your **Downloads** folder and double-click `TacticalMapper.exe`.
3. If Windows shows a security warning, select **More info**, then **Run anyway**.
4. The first launch installs the application and creates a desktop shortcut.
5. Open **Tactical Mapper** from the desktop whenever you want to use it.

The application stores mapping data in the configured PostgreSQL database.

### From the source tree

```powershell
.\run_project.ps1
```

The window starts with a checked-in LiDAR dataset replay, a moving vehicle, a
semantic map, pause/resume, speed control, reset, and telemetry. Current
semantic labels use explicit rule-based heuristics; trained perception models
are not claimed or bundled.

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

This builds `foveated_lidar_gui\target\release\tactical_mapper.exe`, installs it to
`%LOCALAPPDATA%\TacticalMapper`, and creates a **Tactical Mapper** desktop
shortcut in one step. PostgreSQL setup is documented in
[`db/README.md`](db/README.md).
