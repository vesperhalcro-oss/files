# Tactical Mapper

> Real-time local mapping when GPS is unavailable.

Tactical Mapper is a local LiDAR visualization tool for robotics teams,
field operators, and developers who need a clear view of vehicle position, scan
returns, semantic terrain labels, and map reconstruction. The native prototype
replays the checked-in LiDAR dataset at `foveated_lidar_gui/src/mock_lidar_stream.json`
through the same foveated mapping pipeline used for live input.

The native application uses **PostgreSQL** for asynchronous persistence. SQLite
is not used or supported by the native mapping application.

[![Live demo](https://img.shields.io/badge/live%20demo-Vercel-111827?logo=vercel)](https://run-and-debug-setup.vercel.app/)
[![Rust](https://img.shields.io/badge/built%20with-Rust-orange?logo=rust)](https://www.rust-lang.org/)
[![Version](https://img.shields.io/badge/version-0.1.0-22c55e)](foveated_lidar_gui/Cargo.toml)

## Explore the product

Open the [live Tactical Mapper demo](https://run-and-debug-setup.vercel.app/)
to replay recorded LiDAR frames as a local-coordinate 2.5D occupancy/elevation
map, with foveated cells, semantic terrain, separate objects, telemetry, and
measured browser metrics.

The Windows application provides the same workflow as a native desktop tool,
with asynchronous PostgreSQL persistence on the local machine.

## PostgreSQL database setup

PostgreSQL must be running locally before starting the native application. Create
the database and apply the schema from a PowerShell terminal:

```powershell
createdb -h 127.0.0.1 -p 5432 -U postgres tactical_mapper_db
psql -h 127.0.0.1 -p 5432 -U postgres -d tactical_mapper_db -f db/schema.sql
```

The application reads its connection string from `TACTICAL_MAPPER_DB_URL`.
For a local PostgreSQL installation, the default is:

```text
host=127.0.0.1 port=5432 user=postgres dbname=tactical_mapper_db
```

Set the variable when using a password, a different role, or a non-default
PostgreSQL host or port:

```powershell
$env:TACTICAL_MAPPER_DB_URL = "host=127.0.0.1 port=5432 user=mapper password=<password> dbname=tactical_mapper_db"
```

Database connection and batch-write failures are surfaced in the native GUI as
`Storage: degraded`. Full schema and troubleshooting details are in
[`db/README.md`](db/README.md).

## Download and run

Run the one-step setup script from a Windows checkout:

```powershell
.\setup.ps1
```

It builds the release binary, installs it for the current user, and creates a
**Tactical Mapper** desktop shortcut. Complete the PostgreSQL setup above before
launching the application.

## Run the GUI and LiDAR replay

### From the downloaded executable

If you are not a developer, follow these steps:

1. Install PostgreSQL and create `tactical_mapper_db` as described above.
2. Download `TacticalMapper.exe` from the project release.
3. Open your **Downloads** folder and double-click `TacticalMapper.exe`.
4. If Windows shows a security warning, select **More info**, then **Run anyway**.
5. The first launch installs the application and creates a desktop shortcut.
6. Open **Tactical Mapper** from the desktop whenever you want to use it.

The application stores mapping data in the configured PostgreSQL database.

### From the source tree

```powershell
.\run_project.ps1
```

The window starts with a checked-in LiDAR dataset replay, a local-coordinate
2.5D cell map, a moving vehicle, semantic terrain, separate static-object
detections, pause/resume, speed control, reset, and telemetry. Each cell stores
minimum, maximum, and mean elevation plus point count. Current semantic labels
use explicit geometric/rule-based heuristics; trained perception models are not
claimed or bundled.

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
shortcut in one step. PostgreSQL setup is documented in the
[`db/README.md`](db/README.md) guide.

## Storage architecture

The native pipeline writes mapping frames, foveated spatial cells, and detected
objects to PostgreSQL through an asynchronous Tokio worker. Writes are batched
in transactions so database I/O does not block LiDAR processing or rendering.
The schema includes indexes for timestamped frames, resolution-band cells, and
tracked objects.

The browser/Vercel demo is static and does not connect to PostgreSQL. It replays
bundled data from `demo/replay-data.js`.
