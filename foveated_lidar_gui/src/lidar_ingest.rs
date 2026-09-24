use std::fs;
use std::path::{Path, PathBuf};
use tokio::sync::mpsc;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LidarFrame {
    pub timestamp: f64,
    pub points: Vec<Point3D>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IngestConfig {
    pub min_range: f32,
    pub max_range: f32,
    pub downsample_voxel: Option<f32>,
}

impl Default for IngestConfig {
    fn default() -> Self {
        Self {
            min_range: 0.1,
            max_range: 100.0,
            downsample_voxel: Some(0.05),
        }
    }
}

impl IngestConfig {
    pub fn with_range(min_range: f32, max_range: f32) -> Self {
        Self {
            min_range,
            max_range,
            downsample_voxel: Some(0.05),
        }
    }
}

pub fn parse_point_cloud_file(path: &Path, config: &IngestConfig) -> Result<LidarFrame, String> {
    let extension = path
        .extension()
        .and_then(|value| value.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();

    let points = match extension.as_str() {
        "bin" => parse_velodyne_bin(path, config)?,
        "pcd" => parse_pcd_ascii(path, config)?,
        "ply" => parse_ply_ascii(path, config)?,
        _ => {
            return Err(format!(
                "Unsupported LiDAR format for '{}'. Supported: .bin, .pcd, .ply",
                path.display()
            ));
        }
    };

    Ok(LidarFrame {
        timestamp: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|error| format!("Clock error: {error}"))?
            .as_secs_f64(),
        points,
    })
}

#[allow(dead_code)]
pub async fn spawn_ingest_worker(
    mut rx: mpsc::Receiver<PathBuf>,
    tx: mpsc::Sender<LidarFrame>,
    config: IngestConfig,
) {
    while let Some(path) = rx.recv().await {
        match parse_point_cloud_file(&path, &config) {
            Ok(frame) => {
                if tx.send(frame).await.is_err() {
                    break;
                }
            }
            Err(error) => {
                eprintln!("Failed to ingest {}: {error}", path.display());
            }
        }
    }
}

fn is_valid_point(point: Point3D) -> bool {
    point.x.is_finite()
        && point.y.is_finite()
        && point.z.is_finite()
        && point.intensity.is_finite()
}

fn point_in_range(point: Point3D, config: &IngestConfig) -> bool {
    let range = (point.x * point.x + point.y * point.y + point.z * point.z).sqrt();
    range >= config.min_range && range <= config.max_range
}

fn filter_points(points: Vec<Point3D>, config: &IngestConfig) -> Vec<Point3D> {
    let mut filtered = Vec::with_capacity(points.len());
    for point in points {
        if !is_valid_point(point) || !point_in_range(point, config) {
            continue;
        }
        filtered.push(point);
    }

    if let Some(voxel) = config.downsample_voxel.filter(|size| *size > 0.0) {
        downsample_voxel_grid(filtered, voxel)
    } else {
        filtered
    }
}

fn downsample_voxel_grid(points: Vec<Point3D>, voxel_size: f32) -> Vec<Point3D> {
    let mut centers: std::collections::HashMap<(i64, i64, i64), Vec<Point3D>> = std::collections::HashMap::new();

    for point in points {
        let key = (
            (point.x / voxel_size).floor() as i64,
            (point.y / voxel_size).floor() as i64,
            (point.z / voxel_size).floor() as i64,
        );
        centers.entry(key).or_default().push(point);
    }

    let mut reduced = Vec::with_capacity(centers.len());
    for bucket in centers.into_values() {
        let mut sum_x = 0.0_f32;
        let mut sum_y = 0.0_f32;
        let mut sum_z = 0.0_f32;
        let mut sum_intensity = 0.0_f32;
        for point in &bucket {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            sum_intensity += point.intensity;
        }
        let count = bucket.len() as f32;
        reduced.push(Point3D {
            x: sum_x / count,
            y: sum_y / count,
            z: sum_z / count,
            intensity: sum_intensity / count,
        });
    }

    reduced.sort_by(|left, right| {
        left.x
            .partial_cmp(&right.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                left.y
                    .partial_cmp(&right.y)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| {
                left.z
                    .partial_cmp(&right.z)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    });

    reduced
}

fn parse_velodyne_bin(path: &Path, config: &IngestConfig) -> Result<Vec<Point3D>, String> {
    let bytes = fs::read(path)
        .map_err(|error| format!("Could not read {}: {error}", path.display()))?;
    if bytes.len() % std::mem::size_of::<f32>() != 0 {
        return Err(format!(
            "Invalid Velodyne .bin payload in {}: odd number of bytes",
            path.display()
        ));
    }

    let mut values = Vec::with_capacity(bytes.len() / std::mem::size_of::<f32>());
    for chunk in bytes.chunks_exact(std::mem::size_of::<f32>()) {
        let value = f32::from_le_bytes([
            chunk[0], chunk[1], chunk[2], chunk[3],
        ]);
        values.push(value);
    }

    if values.len() % 4 != 0 {
        return Err(format!(
            "Velodyne .bin payload in {} has an incomplete point record",
            path.display()
        ));
    }

    let mut points = Vec::with_capacity(values.len() / 4);
    for index in (0..values.len()).step_by(4) {
        let point = Point3D {
            x: values[index],
            y: values[index + 1],
            z: values[index + 2],
            intensity: values[index + 3],
        };
        points.push(point);
    }

    Ok(filter_points(points, config))
}

fn parse_pcd_ascii(path: &Path, config: &IngestConfig) -> Result<Vec<Point3D>, String> {
    let content = fs::read_to_string(path)
        .map_err(|error| format!("Could not read {}: {error}", path.display()))?;

    let mut points = Vec::new();
    let mut reading_points = false;
    let mut fields = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if trimmed.starts_with("VERSION") || trimmed.starts_with("FIELDS") {
            if trimmed.starts_with("FIELDS") {
                fields = trimmed
                    .split_whitespace()
                    .skip(1)
                    .map(str::to_owned)
                    .collect();
            }
            continue;
        }
        if trimmed.starts_with("SIZE") || trimmed.starts_with("TYPE") || trimmed.starts_with("COUNT") {
            continue;
        }
        if trimmed.starts_with("POINTS") {
            continue;
        }
        if trimmed.starts_with("DATA") {
            reading_points = true;
            continue;
        }
        if !reading_points {
            continue;
        }

        let values: Vec<&str> = trimmed.split_whitespace().collect();
        if values.len() < 3 {
            continue;
        }

        let mut x = 0.0_f32;
        let mut y = 0.0_f32;
        let mut z = 0.0_f32;
        let mut intensity = 0.0_f32;

        for (index, field) in fields.iter().enumerate() {
            if index >= values.len() {
                break;
            }
            let value = values[index].parse::<f32>().unwrap_or(0.0);
            match field.as_str() {
                "x" => x = value,
                "y" => y = value,
                "z" => z = value,
                "intensity" => intensity = value,
                _ => {}
            }
        }

        points.push(Point3D {
            x,
            y,
            z,
            intensity,
        });
    }

    Ok(filter_points(points, config))
}

fn parse_ply_ascii(path: &Path, config: &IngestConfig) -> Result<Vec<Point3D>, String> {
    let content = fs::read_to_string(path)
        .map_err(|error| format!("Could not read {}: {error}", path.display()))?;

    let mut fields = Vec::new();
    let mut reading_vertices = false;
    let mut points = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if trimmed.starts_with("element vertex") {
            reading_vertices = true;
            continue;
        }
        if trimmed.starts_with("property") {
            let property = trimmed
                .split_whitespace()
                .last()
                .unwrap_or_default()
                .to_owned();
            fields.push(property);
            continue;
        }
        if trimmed == "end_header" {
            reading_vertices = true;
            continue;
        }
        if !reading_vertices || trimmed.starts_with("end_header") {
            continue;
        }

        let values: Vec<&str> = trimmed.split_whitespace().collect();
        if values.is_empty() {
            continue;
        }

        let mut x = 0.0_f32;
        let mut y = 0.0_f32;
        let mut z = 0.0_f32;
        let mut intensity = 0.0_f32;

        for (index, field) in fields.iter().enumerate() {
            if index >= values.len() {
                break;
            }
            let value = values[index].parse::<f32>().unwrap_or(0.0);
            match field.as_str() {
                "x" => x = value,
                "y" => y = value,
                "z" => z = value,
                "intensity" => intensity = value,
                _ => {}
            }
        }

        points.push(Point3D {
            x,
            y,
            z,
            intensity,
        });
    }

    Ok(filter_points(points, config))
}

pub fn transform_point(point: Point3D, x: f32, y: f32, yaw: f32) -> Point3D {
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    Point3D {
        x: point.x * cos_yaw - point.y * sin_yaw + x,
        y: point.x * sin_yaw + point.y * cos_yaw + y,
        z: point.z,
        intensity: point.intensity,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;

    fn temp_path(name: &str, extension: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!("tactical_mapper_{name}_{}.{}", std::process::id(), extension));
        path
    }

    #[test]
    fn velodyne_bin_parses_known_points() {
        let path = temp_path("velodyne", "bin");
        let points = [
            1.0_f32, 2.0, 3.0, 0.5,
            4.0, 5.0, 6.0, 0.9,
        ];
        let bytes: Vec<u8> = points
            .iter()
            .flat_map(|value| value.to_le_bytes())
            .collect();
        fs::write(&path, bytes).unwrap();

        let config = IngestConfig::default();
        let frame = parse_point_cloud_file(&path, &config).unwrap();
        assert_eq!(frame.points.len(), 2);
        assert_eq!(frame.points[0], Point3D { x: 1.0, y: 2.0, z: 3.0, intensity: 0.5 });
        assert_eq!(frame.points[1], Point3D { x: 4.0, y: 5.0, z: 6.0, intensity: 0.9 });

        let _ = fs::remove_file(&path);
    }

    #[test]
    fn pcd_ascii_parses_xyz() {
        let path = temp_path("pcd", "pcd");
        let content = "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 2 3 0.25\n4 5 6 0.75\n";
        fs::write(&path, content).unwrap();

        let config = IngestConfig::default();
        let frame = parse_point_cloud_file(&path, &config).unwrap();
        assert_eq!(frame.points.len(), 2);
        assert_eq!(frame.points[0].x, 1.0);
        assert_eq!(frame.points[1].z, 6.0);

        let _ = fs::remove_file(&path);
    }

    #[test]
    fn range_filter_removes_out_of_bounds_points() {
        let points = vec![
            Point3D { x: 0.5, y: 0.0, z: 0.0, intensity: 1.0 },
            Point3D { x: 120.0, y: 0.0, z: 0.0, intensity: 1.0 },
        ];
        let config = IngestConfig::with_range(0.1, 10.0);
        let filtered = filter_points(points, &config);
        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].x, 0.5);
    }

    #[test]
    fn transform_point_applies_pose_and_yaw() {
        let point = Point3D { x: 1.0, y: 0.0, z: 0.0, intensity: 0.5 };
        let transformed = transform_point(point, 2.0, 3.0, std::f32::consts::FRAC_PI_2);
        assert!((transformed.x - 2.0).abs() < 1e-5);
        assert!((transformed.y - 4.0).abs() < 1e-5);
    }
}
