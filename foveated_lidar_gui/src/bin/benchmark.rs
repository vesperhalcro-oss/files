use serde::Deserialize;
use std::collections::HashSet;
use std::time::Instant;

#[derive(Deserialize)]
struct Dataset {
    lidar_scan: Vec<Point>,
}

#[derive(Deserialize)]
struct Point {
    x: f32,
    y: f32,
    z: f32,
}

fn percentile(values: &mut [f64], fraction: f64) -> f64 {
    values.sort_by(|left, right| left.partial_cmp(right).unwrap());
    let index = ((values.len().saturating_sub(1)) as f64 * fraction) as usize;
    values[index]
}

fn main() {
    let dataset: Dataset =
        serde_json::from_str(include_str!("../mock_lidar_stream.json")).expect("valid dataset");
    let frames = 100;
    let started = Instant::now();
    let mut latencies = Vec::with_capacity(frames);
    let mut points_processed = 0usize;
    let mut cells = HashSet::new();

    for _ in 0..frames {
        let frame_started = Instant::now();
        for point in &dataset.lidar_scan {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                continue;
            }
            points_processed += 1;
            let range = point.x.hypot(point.y);
            let resolution = if range <= 10.0 { 0.05 } else if range <= 30.0 { 0.1 } else { 0.5 };
            cells.insert((
                (point.x / resolution).floor() as i32,
                (point.y / resolution).floor() as i32,
                (resolution * 100.0) as u8,
            ));
        }
        latencies.push(frame_started.elapsed().as_secs_f64() * 1000.0);
    }

    let elapsed = started.elapsed().as_secs_f64();
    let p50 = percentile(&mut latencies.clone(), 0.50);
    let p95 = percentile(&mut latencies.clone(), 0.95);
    let p99 = percentile(&mut latencies, 0.99);
    println!("Frames processed: {frames}");
    println!("Points processed: {points_processed}");
    println!("P50 latency: {p50:.3} ms");
    println!("P95 latency: {p95:.3} ms");
    println!("P99 latency: {p99:.3} ms");
    println!("Points/sec: {:.0}", points_processed as f64 / elapsed.max(f64::EPSILON));
    println!("Cells/sec: {:.0}", cells.len() as f64 * frames as f64 / elapsed.max(f64::EPSILON));
    println!("Peak memory: unavailable (run under a system profiler)");
    println!("Foveated reduction: {:.1}%", 100.0 * (1.0 - cells.len() as f64 / points_processed.max(1) as f64));
}
