-- Create the database once if it does not already exist:
-- createdb -U postgres tactical_mapper_db
\c tactical_mapper_db;

CREATE EXTENSION IF NOT EXISTS postgis;

-- 1. Table for tracking dead-reckoning trajectory
CREATE TABLE IF NOT EXISTS vehicle_poses (
    id BIGSERIAL PRIMARY KEY,
    frame_id BIGINT NOT NULL,
    pos_x REAL NOT NULL,
    pos_y REAL NOT NULL,
    pos_z REAL NOT NULL,
    yaw REAL NOT NULL,
    recorded_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
);

-- 2. Table for persisting foveated grid map state
CREATE TABLE IF NOT EXISTS grid_map_cells (
    grid_x INT NOT NULL,
    grid_y INT NOT NULL,
    elevation REAL NOT NULL,
    resolution REAL NOT NULL,
    iff_status INT DEFAULT 0, -- 0: Neutral, 1: Own Team, 2: Enemy, 3: Asset
    updated_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (grid_x, grid_y)
);

CREATE TABLE IF NOT EXISTS lidar_point_stream (
    id BIGSERIAL PRIMARY KEY,
    frame_index BIGINT NOT NULL,
    point_id BIGINT NOT NULL,
    intensity REAL NOT NULL,
    geom_point geometry(PointZ, 4326) NOT NULL,
    recorded_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS lidar_point_stream_geom_idx
    ON lidar_point_stream USING GIST (geom_point);