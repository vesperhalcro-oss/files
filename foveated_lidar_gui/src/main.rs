use eframe::egui;
use serde::Deserialize;
use std::collections::{hash_map::Entry, HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::Instant;
use tokio::sync::mpsc;
use tokio_postgres::NoTls;

#[derive(Debug, Clone, Copy, Default)]
struct Pose3D {
    x: f32,
    y: f32,
    z: f32,
    yaw: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SemanticClass {
    Unknown,
    DrivableTerrain,
    NonDrivableTerrain,
    Wall,
    Pole,
    Barrier,
    StaticObstacle,
    Pedestrian,
    Vehicle,
    OtherDynamic,
}

impl SemanticClass {
    const ALL: [Self; 10] = [
        Self::Unknown,
        Self::DrivableTerrain,
        Self::NonDrivableTerrain,
        Self::Wall,
        Self::Pole,
        Self::Barrier,
        Self::StaticObstacle,
        Self::Pedestrian,
        Self::Vehicle,
        Self::OtherDynamic,
    ];

    fn as_str(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::DrivableTerrain => "drivable_terrain",
            Self::NonDrivableTerrain => "non_drivable_terrain",
            Self::Wall => "wall",
            Self::Pole => "pole",
            Self::Barrier => "barrier",
            Self::StaticObstacle => "static_obstacle",
            Self::Pedestrian => "pedestrian",
            Self::Vehicle => "vehicle",
            Self::OtherDynamic => "other_dynamic",
        }
    }

    fn color(self) -> egui::Color32 {
        match self {
            Self::DrivableTerrain => egui::Color32::from_rgb(95, 190, 130),
            Self::NonDrivableTerrain => egui::Color32::from_rgb(185, 145, 85),
            Self::Wall | Self::Barrier | Self::StaticObstacle => {
                egui::Color32::from_rgb(220, 115, 90)
            }
            Self::Pole => egui::Color32::from_rgb(175, 135, 225),
            Self::Pedestrian | Self::Vehicle | Self::OtherDynamic => {
                egui::Color32::from_rgb(240, 90, 150)
            }
            Self::Unknown => egui::Color32::from_rgb(150, 165, 165),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct SpatialCell {
    elevation: f32,
    resolution: f32,
    semantic_class: SemanticClass,
    confidence: f32,
}

#[derive(Debug, Deserialize)]
struct DatasetPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
}

#[derive(Debug, Deserialize)]
struct DatasetFrame {
    lidar_scan: Vec<DatasetPoint>,
}

fn load_dataset() -> Vec<LidarPoint> {
    let dataset: DatasetFrame = serde_json::from_str(include_str!("mock_lidar_stream.json"))
        .expect("mock LiDAR dataset must be valid JSON");
    dataset
        .lidar_scan
        .into_iter()
        .map(|point| LidarPoint {
            x: point.x,
            y: point.y,
            z: point.z,
            intensity: point.intensity,
        })
        .collect()
}

fn classify_point(point: LidarPoint) -> (SemanticClass, f32) {
    if point.z < 0.75 {
        (SemanticClass::DrivableTerrain, 0.82)
    } else if point.z < 1.1 && point.intensity > 0.7 {
        (SemanticClass::Barrier, 0.68)
    } else if point.z > 1.6 {
        (SemanticClass::StaticObstacle, 0.61)
    } else {
        (SemanticClass::NonDrivableTerrain, 0.55)
    }
}

impl Pose3D {
    fn normalize_yaw(&mut self) {
        let pi = std::f32::consts::PI;
        self.yaw = (self.yaw + pi).rem_euclid(2.0 * pi) - pi;
    }
}

#[derive(Debug, Clone, Copy)]
struct LidarPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
}

fn update_dead_reckoning(pose: &mut Pose3D, acceleration: f32, gyro: f32, dt: f32) {
    pose.yaw += gyro * dt;
    pose.normalize_yaw();
    let (sin_yaw, cos_yaw) = pose.yaw.sin_cos();
    pose.x += acceleration * dt * cos_yaw;
    pose.y += acceleration * dt * sin_yaw;
}

const RES_NEAR: f32 = 0.05;
const RES_MID: f32 = 0.10;
const RES_FAR: f32 = 0.50;
const DB_CHANNEL_CAPACITY: usize = 4096;

fn resolution_for_range(range: f32) -> f32 {
    if range <= 10.0 {
        RES_NEAR
    } else if range <= 30.0 {
        RES_MID
    } else {
        RES_FAR
    }
}

#[derive(Debug, Clone)]
enum DbCommand {
    SaveFrame {
        frame: u64,
        pose: Pose3D,
        cells: Vec<(i32, i32, f32, f32, &'static str, f32)>,
    },
}

struct Engine {
    pose: Pose3D,
    map: HashMap<(i32, i32, u8), SpatialCell>,
    points: Vec<LidarPoint>,
    trail: VecDeque<(f32, f32)>,
    frame: u64,
    simulation_time: f32,
    start: Instant,
    db_tx: mpsc::Sender<DbCommand>,
    storage_ok: Arc<AtomicBool>,
    dataset: Vec<LidarPoint>,
    dataset_offset: usize,
}

impl Engine {
    fn new(db_tx: mpsc::Sender<DbCommand>, storage_ok: Arc<AtomicBool>) -> Self {
        Self {
            pose: Pose3D::default(),
            map: HashMap::with_capacity(4096),
            points: Vec::with_capacity(256),
            trail: VecDeque::with_capacity(2048),
            frame: 0,
            simulation_time: 0.0,
            start: Instant::now(),
            db_tx,
            storage_ok,
            dataset: load_dataset(),
            dataset_offset: 0,
        }
    }

    fn reset(&mut self) {
        self.pose = Pose3D::default();
        self.map.clear();
        self.points.clear();
        self.trail.clear();
        self.frame = 0;
        self.simulation_time = 0.0;
        self.start = Instant::now();
    }

    fn step(&mut self, dt: f32, speed: f32) {
        let simulation_dt = dt * speed;
        self.simulation_time += simulation_dt;
        update_dead_reckoning(&mut self.pose, 0.5, 0.01, simulation_dt);
        self.trail.push_back((self.pose.x, self.pose.y));
        if self.trail.len() > 2048 {
            self.trail.pop_front();
        }

        self.points.clear();
        let dataset_len = self.dataset.len();
        let mut changed_cells = Vec::new();
        for index in 0..dataset_len {
            let dataset_point = self.dataset[(self.dataset_offset + index) % dataset_len];
            let point = LidarPoint {
                x: dataset_point.x * 0.12,
                y: dataset_point.y * 0.12,
                z: dataset_point.z,
                intensity: dataset_point.intensity,
            };
            self.points.push(point);

            let (sin_yaw, cos_yaw) = self.pose.yaw.sin_cos();
            let world_x = self.pose.x + point.x * cos_yaw - point.y * sin_yaw;
            let world_y = self.pose.y + point.x * sin_yaw + point.y * cos_yaw;
            let range = point.x.hypot(point.y);
            let resolution = resolution_for_range(range);
            let resolution_band = (resolution * 100.0) as u8;
            let (semantic_class, confidence) = classify_point(point);
            let cell = (
                (world_x / resolution).floor() as i32,
                (world_y / resolution).floor() as i32,
            );
            let key = (cell.0, cell.1, resolution_band);
            match self.map.entry(key) {
                Entry::Occupied(mut entry) => {
                    if point.z > entry.get().elevation || confidence > entry.get().confidence {
                        let stored = entry.get_mut();
                        stored.elevation = stored.elevation.max(point.z);
                        stored.semantic_class = semantic_class;
                        stored.confidence = confidence;
                        changed_cells.push((
                            cell.0,
                            cell.1,
                            point.z,
                            resolution,
                            semantic_class.as_str(),
                            confidence,
                        ));
                    }
                }
                Entry::Vacant(entry) => {
                    entry.insert(SpatialCell {
                        elevation: point.z,
                        resolution,
                        semantic_class,
                        confidence,
                    });
                    changed_cells.push((
                        cell.0,
                        cell.1,
                        point.z,
                        resolution,
                        semantic_class.as_str(),
                        confidence,
                    ));
                }
            }
        }
        self.dataset_offset = (self.dataset_offset + 1) % dataset_len;
        if self
            .db_tx
            .try_send(DbCommand::SaveFrame {
                frame: self.frame,
                pose: self.pose,
                cells: changed_cells,
            })
            .is_err()
        {
            self.storage_ok.store(false, Ordering::Release);
        }
        self.frame += 1;
    }
}

fn connection_string() -> String {
    std::env::var("TACTICAL_MAPPER_DB_URL").unwrap_or_else(|_| {
        "host=127.0.0.1 port=5432 user=postgres dbname=tactical_mapper_db".to_string()
    })
}

async fn run_db_worker(
    mut rx: mpsc::Receiver<DbCommand>,
    storage_ok: Arc<AtomicBool>,
) -> Result<(), tokio_postgres::Error> {
    let (client, connection) = tokio_postgres::connect(&connection_string(), NoTls).await?;
    let connection_health = Arc::clone(&storage_ok);
    tokio::spawn(async move {
        if let Err(error) = connection.await {
            eprintln!("PostgreSQL connection failed: {error}");
        }
        connection_health.store(false, Ordering::Release);
    });
    client
        .batch_execute(include_str!("../../db/schema.sql"))
        .await?;
    storage_ok.store(true, Ordering::Release);

    let mut client = client;
    let mut batch = Vec::with_capacity(64);
    while let Some(command) = rx.recv().await {
        batch.clear();
        batch.push(command);
        while batch.len() < 64 {
            match rx.try_recv() {
                Ok(command) => batch.push(command),
                Err(_) => break,
            }
        }
        if let Err(error) = write_batch(&mut client, &batch).await {
            storage_ok.store(false, Ordering::Release);
            eprintln!("PostgreSQL batch write failed: {error}");
        } else {
            storage_ok.store(true, Ordering::Release);
        }
    }
    Ok(())
}

async fn write_batch(
    client: &mut tokio_postgres::Client,
    batch: &[DbCommand],
) -> Result<(), tokio_postgres::Error> {
    let transaction = client.transaction().await?;
    for command in batch {
        match command {
            DbCommand::SaveFrame { frame, pose, cells } => {
                transaction
                    .execute(
                        "INSERT INTO mapping_frames
                         (frame_id, pose_x, pose_y, pose_z, yaw, point_count)
                         VALUES ($1, $2, $3, $4, $5, $6)",
                        &[
                            &(*frame as i64),
                            &pose.x,
                            &pose.y,
                            &pose.z,
                            &pose.yaw,
                            &(cells.len() as i32),
                        ],
                    )
                    .await?;
                for (gx, gy, elevation, resolution, class_name, confidence) in cells {
                    transaction
                        .execute(
                            "INSERT INTO spatial_cells
                             (grid_x, grid_y, resolution_band, elevation, resolution, class_name, confidence)
                             VALUES ($1, $2, $3, $4, $5, $6, $7)
                             ON CONFLICT (grid_x, grid_y, resolution_band)
                             DO UPDATE SET elevation = EXCLUDED.elevation,
                                           resolution = EXCLUDED.resolution,
                                           class_name = EXCLUDED.class_name,
                                           confidence = EXCLUDED.confidence,
                                           updated_at = CURRENT_TIMESTAMP",
                            &[gx, gy, &((*resolution * 100.0) as i16), elevation, resolution, class_name, confidence],
                        )
                        .await?;
                }
            }
        }
    }
    transaction.commit().await
}

struct App {
    engine: Engine,
    running: bool,
    speed: f32,
    storage_ok: Arc<AtomicBool>,
}

impl App {
    fn draw_map(&self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let size = available.x.min(available.y).max(320.0);
        let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let center = rect.center();
        let scale = size / 70.0;

        painter.rect_filled(rect, 10.0, egui::Color32::from_rgb(10, 22, 28));
        painter.rect_stroke(
            rect,
            10.0,
            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(42, 75, 78)),
        );
        for step in -30..=30 {
            let offset = step as f32 * scale;
            painter.line_segment(
                [
                    center + egui::vec2(offset, -30.0 * scale),
                    center + egui::vec2(offset, 30.0 * scale),
                ],
                egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(28, 54, 58)),
            );
            painter.line_segment(
                [
                    center + egui::vec2(-30.0 * scale, offset),
                    center + egui::vec2(30.0 * scale, offset),
                ],
                egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(28, 54, 58)),
            );
        }
        for (from_point, to_point) in self
            .engine
            .trail
            .iter()
            .zip(self.engine.trail.iter().skip(1))
        {
            let from = center + egui::vec2(from_point.0 * scale, -from_point.1 * scale);
            let to = center + egui::vec2(to_point.0 * scale, -to_point.1 * scale);
            painter.line_segment(
                [from, to],
                egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(82, 196, 164)),
            );
        }
        for point in &self.engine.points {
            let (sin_yaw, cos_yaw) = self.engine.pose.yaw.sin_cos();
            let world_x = self.engine.pose.x + point.x * cos_yaw - point.y * sin_yaw;
            let world_y = self.engine.pose.y + point.x * sin_yaw + point.y * cos_yaw;
            let position = center + egui::vec2(world_x * scale, -world_y * scale);
            let color = classify_point(*point).0.color();
            painter.circle_filled(position, 1.8 + point.z * 0.35, color);
        }
        let vehicle = center + egui::vec2(self.engine.pose.x * scale, -self.engine.pose.y * scale);
        painter.circle_filled(vehicle, 7.0, egui::Color32::from_rgb(245, 184, 74));
        let direction = egui::vec2(self.engine.pose.yaw.cos(), -self.engine.pose.yaw.sin()) * 14.0;
        painter.line_segment(
            [vehicle, vehicle + direction],
            egui::Stroke::new(3.0_f32, egui::Color32::from_rgb(255, 244, 210)),
        );
        let scale_start = rect.left_bottom() + egui::vec2(18.0, -18.0);
        painter.line_segment(
            [scale_start, scale_start + egui::vec2(10.0 * scale, 0.0)],
            egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(190, 215, 205)),
        );
        painter.text(
            scale_start + egui::vec2(0.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            "10 m",
            egui::FontId::proportional(12.0),
            egui::Color32::from_rgb(190, 215, 205),
        );
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let mut visuals = egui::Visuals::dark();
        visuals.override_text_color = Some(egui::Color32::from_rgb(218, 232, 225));
        visuals.panel_fill = egui::Color32::from_rgb(11, 24, 29);
        visuals.window_fill = egui::Color32::from_rgb(11, 24, 29);
        visuals.extreme_bg_color = egui::Color32::from_rgb(6, 15, 19);
        visuals.faint_bg_color = egui::Color32::from_rgb(20, 43, 45);
        visuals.widgets.noninteractive.bg_fill = egui::Color32::from_rgb(18, 38, 40);
        visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(24, 57, 56);
        visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(36, 88, 79);
        visuals.widgets.active.bg_fill = egui::Color32::from_rgb(63, 145, 116);
        ctx.set_visuals(visuals);
        if self.running {
            // Avoid a large simulation jump after the window has been
            // suspended or dragged between monitors.
            let dt = ctx.input(|input| input.stable_dt).min(0.1);
            self.engine.step(dt, self.speed);
        }
        egui::SidePanel::left("controls")
            .min_width(250.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("TACTICAL MAPPER");
                ui.small("GPS-denied mapping console");
                ui.separator();
                let status_color = if self.running {
                    egui::Color32::from_rgb(104, 214, 157)
                } else {
                    egui::Color32::from_rgb(245, 184, 74)
                };
                ui.colored_label(
                    status_color,
                    if self.running {
                        "● RUNNING"
                    } else {
                        "● PAUSED"
                    },
                );
                ui.label("Live local simulation");
                ui.add_space(6.0);
                ui.add(egui::Slider::new(&mut self.speed, 0.25..=4.0).text("Speed"));
                if ui
                    .add_sized(
                        [ui.available_width(), 30.0],
                        egui::Button::new(if self.running { "Pause" } else { "Resume" }),
                    )
                    .clicked()
                {
                    self.running = !self.running;
                }
                if ui
                    .add_sized([ui.available_width(), 30.0], egui::Button::new("Reset map"))
                    .clicked()
                {
                    self.engine.reset();
                }
                ui.separator();
                ui.heading("Telemetry");
                egui::Grid::new("telemetry_grid")
                    .num_columns(2)
                    .spacing([18.0, 8.0])
                    .show(ui, |ui| {
                        ui.weak("Frame");
                        ui.label(self.engine.frame.to_string());
                        ui.end_row();
                        ui.weak("LiDAR points");
                        ui.label(self.engine.points.len().to_string());
                        ui.end_row();
                        ui.weak("Map cells");
                        ui.label(self.engine.map.len().to_string());
                        ui.end_row();
                        ui.weak("Resolution bands");
                        let bands = self
                            .engine
                            .map
                            .values()
                            .filter(|cell| cell.resolution == RES_NEAR)
                            .count();
                        ui.label(format!("{bands} near / {} total", self.engine.map.len()));
                        ui.end_row();
                        ui.weak("Position");
                        ui.label(format!(
                            "{:.1}, {:.1} m",
                            self.engine.pose.x, self.engine.pose.y
                        ));
                        ui.end_row();
                        ui.weak("Heading");
                        ui.label(format!("{:.1}°", self.engine.pose.yaw.to_degrees()));
                        ui.end_row();
                        ui.weak("Elapsed");
                        ui.label(format!(
                            "{:.1} s",
                            self.engine.start.elapsed().as_secs_f32()
                        ));
                        ui.end_row();
                    });
                ui.separator();
                let storage_ok = self.storage_ok.load(Ordering::Acquire);
                ui.colored_label(
                    if storage_ok {
                        egui::Color32::from_rgb(104, 214, 157)
                    } else {
                        egui::Color32::from_rgb(245, 125, 90)
                    },
                    if storage_ok {
                        "Storage: OK"
                    } else {
                        "Storage: degraded"
                    },
                );
                ui.collapsing("Semantic legend (rule-based)", |ui| {
                    for class in SemanticClass::ALL {
                        ui.colored_label(class.color(), class.as_str());
                    }
                });
            });
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Live tactical map");
            ui.label("Position, scan returns, and vehicle trail");
            ui.add_space(8.0);
            self.draw_map(ui);
        });
        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}

#[cfg(windows)]
fn powershell_literal(path: &std::path::Path) -> String {
    path.to_string_lossy().replace('\'', "''")
}

#[cfg(windows)]
fn prepare_installation() -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
    let current_exe = std::env::current_exe()?;
    let install_dir = PathBuf::from(std::env::var_os("LOCALAPPDATA").ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "LOCALAPPDATA is not available",
        )
    })?)
    .join("TacticalMapper");
    std::fs::create_dir_all(&install_dir)?;
    let installed_exe = install_dir.join("TacticalMapper.exe");
    let is_installed = installed_exe
        .canonicalize()
        .ok()
        .zip(current_exe.canonicalize().ok())
        .is_some_and(|(installed, current)| installed == current);

    if !is_installed {
        std::fs::copy(&current_exe, &installed_exe)?;
    }

    if !is_installed {
        let desktop = PathBuf::from(std::env::var_os("USERPROFILE").ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotFound, "USERPROFILE is not available")
        })?)
        .join("Desktop")
        .join("Tactical Mapper.lnk");
        let script = format!(
            "$shell = New-Object -ComObject WScript.Shell; \
             $shortcut = $shell.CreateShortcut('{desktop}'); \
             $shortcut.TargetPath = '{target}'; \
             $shortcut.WorkingDirectory = '{working}'; \
             $shortcut.Description = 'Tactical Mapper - offline LiDAR mapping'; \
             $shortcut.Save()",
            desktop = powershell_literal(&desktop),
            target = powershell_literal(&installed_exe),
            working = powershell_literal(&install_dir),
        );
        match std::process::Command::new("powershell")
            .args([
                "-NoProfile",
                "-NonInteractive",
                "-ExecutionPolicy",
                "Bypass",
                "-Command",
                &script,
            ])
            .status()
        {
            Ok(status) if status.success() => {}
            Ok(status) => eprintln!(
                "Warning: could not create the Tactical Mapper desktop shortcut (exit code {status})."
            ),
            Err(error) => eprintln!("Warning: could not create the Tactical Mapper desktop shortcut: {error}"),
        }
        std::process::Command::new(&installed_exe).spawn()?;
        println!("Installed Tactical Mapper to {}", installed_exe.display());
        return Ok(true);
    }
    Ok(false)
}

#[cfg(not(windows))]
fn prepare_installation() -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
    Ok(false)
}

fn main() -> eframe::Result<()> {
    println!("[STATUS] Tactical Mapper starting with PostgreSQL persistence.");
    let was_installed = match prepare_installation() {
        Ok(was_installed) => was_installed,
        Err(error) => {
            eprintln!("Setup failed: {error}");
            return Err(eframe::Error::AppCreation(error));
        }
    };
    if was_installed {
        return Ok(());
    }

    let (db_tx, db_rx) = mpsc::channel(DB_CHANNEL_CAPACITY);
    let storage_ok = Arc::new(AtomicBool::new(false));
    let worker_storage_ok = Arc::clone(&storage_ok);
    std::thread::spawn(move || {
        let runtime = tokio::runtime::Runtime::new().expect("failed to start Tokio runtime");
        runtime.block_on(async move {
            if let Err(error) = run_db_worker(db_rx, worker_storage_ok).await {
                eprintln!("PostgreSQL worker stopped: {error}");
            }
        });
    });
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 760.0])
            .with_min_inner_size([800.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Tactical Mapper",
        options,
        Box::new(|_| {
            Ok(Box::new(App {
                engine: Engine::new(db_tx, Arc::clone(&storage_ok)),
                running: true,
                speed: 1.0,
                storage_ok,
            }))
        }),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dead_reckoning_uses_delta_time() {
        let mut pose = Pose3D::default();
        update_dead_reckoning(&mut pose, 2.0, 0.0, 0.5);
        assert!((pose.x - 1.0).abs() < f32::EPSILON);
        assert!(pose.y.abs() < f32::EPSILON);
    }

    #[test]
    fn yaw_is_normalized_to_half_open_turn() {
        let mut pose = Pose3D {
            yaw: 3.0 * std::f32::consts::PI,
            ..Pose3D::default()
        };
        pose.normalize_yaw();
        assert!((pose.yaw + std::f32::consts::PI).abs() < f32::EPSILON);
    }

    #[test]
    fn grid_resolution_follows_foveation_bands() {
        assert_eq!(resolution_for_range(10.0), RES_NEAR);
        assert_eq!(resolution_for_range(10.01), RES_MID);
        assert_eq!(resolution_for_range(30.0), RES_MID);
        assert_eq!(resolution_for_range(30.01), RES_FAR);
    }

    #[test]
    fn dataset_is_loaded_for_replay() {
        assert!(!load_dataset().is_empty());
    }

    #[test]
    fn classifier_returns_terrain_and_obstacle_classes() {
        assert_eq!(
            classify_point(LidarPoint {
                x: 1.0,
                y: 1.0,
                z: 0.5,
                intensity: 0.2
            })
            .0,
            SemanticClass::DrivableTerrain
        );
        assert_eq!(
            classify_point(LidarPoint {
                x: 1.0,
                y: 1.0,
                z: 0.9,
                intensity: 0.9
            })
            .0,
            SemanticClass::Barrier
        );
    }
}
