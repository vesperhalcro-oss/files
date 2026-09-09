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

## Run the GUI

```powershell
.\run_project.ps1
```

Or:

```powershell
Push-Location foveated_lidar_gui
cargo run --release
Pop-Location
```

## Run the Python importer

```powershell
python -m pip install -r requirements.txt
python foveated_lidar_gui\src\sync_lidar_data.py
```

The importer expects `generated/mock_lidar_stream.json`.
