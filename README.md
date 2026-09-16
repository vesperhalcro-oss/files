# Tactical Mapper

Local Rust/egui visualization and embedded SQLite persistence for GPS-denied LiDAR mapping.

## Live demo

This repository includes a lightweight static landing page configured for Vercel hosting.
Once the project is connected to a Vercel account, the live deployment URL will appear here.

## Requirements

- Rust and Cargo are only required to build the application.
- The downloaded Windows executable has no runtime dependencies. It includes
  SQLite and creates its database automatically.

## Run the GUI and mock simulation

### From the downloaded executable

If you are not a developer, follow these steps:

1. Download `FoveatedLiDAR.exe` from the project release.
2. Open your **Downloads** folder and double-click `FoveatedLiDAR.exe`.
3. If Windows shows a security warning, select **More info**, then **Run anyway**.
4. The first launch installs the application and creates a desktop shortcut.
5. Open **Tactical Mapper** from the desktop whenever you want to use it.

No Rust, Python, PostgreSQL, or command-line setup is required.

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
`%LOCALAPPDATA%\FoveatedLiDAR`, and creates a **Foveated LiDAR** desktop
shortcut in one step. The application creates its local SQLite database on
startup. PostgreSQL is not required.
