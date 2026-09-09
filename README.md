# Foveated LiDAR GUI

Local Rust/egui visualization and embedded SQLite persistence for GPS-denied LiDAR mapping.

## Requirements

- Rust and Cargo are only required to build the application.
- The downloaded Windows executable has no runtime dependencies. It includes
  SQLite and creates its database automatically.

## Run the GUI and mock simulation

### From the downloaded executable

Download `FoveatedLiDAR.exe` from the project release and double-click it.
The first launch installs the application for the current Windows user,
creates a desktop shortcut, and starts the simulator. Future launches can use
the shortcut. No Rust, Python, PostgreSQL, or other runtime installation is
required.

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

## Build a single-file Windows executable

Generated binaries are intentionally not committed to source control. Build the
release executable with:

```powershell
.\build_windows.ps1
```

This creates `dist\FoveatedLiDAR.exe`. Copy that one file to a Windows machine
and double-click it. On first launch it copies itself to
`%LOCALAPPDATA%\FoveatedLiDAR`, creates a desktop shortcut, creates the local
SQLite database, and starts the application. PostgreSQL is not required.

## Run the Python importer

```powershell
python -m pip install -r requirements.txt
python foveated_lidar_gui\src\sync_lidar_data.py
```

The importer expects `generated/mock_lidar_stream.json`.
