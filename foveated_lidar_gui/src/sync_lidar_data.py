import json
import os

import psycopg2
from dotenv import load_dotenv

load_dotenv()

DB_PARAMS = {
    "dbname": os.getenv("POSTGRES_DB", "tactical_mapper_db"),
    "user": os.getenv("POSTGRES_USER", "postgres"),
    "password": os.getenv("POSTGRES_PASSWORD", ""),
    "host": os.getenv("POSTGRES_HOST", "127.0.0.1"),
    "port": int(os.getenv("POSTGRES_PORT", "5432")),
}

def parse_and_insert_lidar(file_path):
    # 1. Read mock telemetry data
    with open(file_path, "r", encoding="utf-8") as file:
        data = json.load(file)
        
    frame = data["telemetry"]["frame"]
    points = data["lidar_scan"]
    
    insert_query = """
        INSERT INTO lidar_point_stream (frame_index, point_id, intensity, geom_point)
        VALUES (%s, %s, %s, ST_SetSRID(ST_MakePoint(%s, %s, %s), 4326));
    """
    batch_data = [(frame, p["point_id"], p["intensity"], p["x"], p["y"], p["z"]) for p in points]

    try:
        with psycopg2.connect(**DB_PARAMS) as conn, conn.cursor() as cursor:
            cursor.executemany(insert_query, batch_data)
        print(f"Syncing Frame {frame} with database engine...")
        print(f"Successfully processed {len(points)} points into PostgreSQL.")
    except (OSError, KeyError, psycopg2.Error) as error:
        raise RuntimeError(f"LiDAR sync failed: {error}") from error

# Execute mapping import pipeline
parse_and_insert_lidar('generated/mock_lidar_stream.json')
