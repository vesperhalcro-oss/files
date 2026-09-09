-- Main spatial table for incoming point logs
CREATE TABLE lidar_point_stream (
    id SERIAL PRIMARY KEY,
    frame_index INT NOT NULL,
    point_id INT NOT NULL,
    intensity REAL,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    -- 3D geometry storage using Z coordinate (SRID 4326 or standard local projection UTM)
    geom_point geometry(PointZ, 4326)
);

-- Indexing for spatial bounding boxes
CREATE INDEX idx_lidar_points_spatial ON lidar_point_stream USING GIST(geom_point);
