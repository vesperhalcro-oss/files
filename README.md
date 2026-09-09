# Foveated LiDAR GUI

Local Rust/egui visualization and PostgreSQL persistence for GPS-denied LiDAR mapping.

## Requirements

- Rust and Cargo
- PostgreSQL with PostGIS if spatial LiDAR imports are used
- Python 3.10+ for `sync_lidar_data.py`

## Configuration

Copy `.env.example` to `.env` and set the PostgreSQL password. The included `.env` contains local development defaults only; do not use it outside a local machine.

## Database setup

```powershell
createdb -U postgres tactical_mapper_db
psql -U postgres -f database.sql
```

## Run the GUI and mock simulation

```powershell
.\run_project.ps1
```

The window starts with a local mock LiDAR stream, a moving vehicle, a live map,
pause/resume, speed control, reset, and telemetry. PostgreSQL persistence is
optional for the simulation; configure `DATABASE_URL` in `.env` to enable it.

Or run directly:

```powershell
Push-Location foveated_lidar_gui
cargo run --release
Pop-Location
```

## Build and install on Windows

Generated binaries are intentionally not committed to source control. Build a
release executable and prepare an install folder with:

```powershell
.\build_windows.ps1
```

Then install it for the current Windows user and create a desktop shortcut:

```powershell
Push-Location dist\FoveatedLiDAR
.\install_windows.ps1
Pop-Location
```

The installed executable is placed at
`%LOCALAPPDATA%\FoveatedLiDAR\FoveatedLiDAR.exe`.

## Run the Python importer

```powershell
python -m pip install -r requirements.txt
python foveated_lidar_gui\src\sync_lidar_data.py
```

The importer expects `generated/mock_lidar_stream.json`.
