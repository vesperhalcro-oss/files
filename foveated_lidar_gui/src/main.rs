use eframe::egui;
use rusqlite::{params, Connection};
use std::collections::{hash_map::Entry, HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::mpsc;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::Instant;

#[derive(Debug, Clone, Copy, Default)]
struct Pose3D {
    x: f32,
    y: f32,
    z: f32,
    yaw: f32,
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
        cells: Vec<(i32, i32, f32, f32)>,
    },
}

struct Engine {
    pose: Pose3D,
    map: HashMap<(i32, i32), (f32, f32)>,
    points: Vec<LidarPoint>,
    trail: VecDeque<(f32, f32)>,
    frame: u64,
    simulation_time: f32,
    start: Instant,
    db_tx: mpsc::SyncSender<DbCommand>,
    storage_ok: Arc<AtomicBool>,
}

impl Engine {
    fn new(db_tx: mpsc::SyncSender<DbCommand>, storage_ok: Arc<AtomicBool>) -> Self {
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
        let mut changed_cells = Vec::new();
        for index in 0..240 {
            let angle = index as f32 * std::f32::consts::TAU / 240.0;
            let range = 8.0
                + (angle * 3.0 + self.simulation_time).sin() * 2.0
                + (angle * 11.0 - self.simulation_time * 0.7).cos().abs() * 1.5;
            let intensity =
                (0.5 + 0.5 * (angle * 5.0 + self.simulation_time).sin()).clamp(0.0, 1.0);
            let point = LidarPoint {
                x: range * angle.cos(),
                y: range * angle.sin(),
                z: 0.5 + intensity,
                intensity,
            };
            self.points.push(point);

            let (sin_yaw, cos_yaw) = self.pose.yaw.sin_cos();
            let world_x = self.pose.x + point.x * cos_yaw - point.y * sin_yaw;
            let world_y = self.pose.y + point.x * sin_yaw + point.y * cos_yaw;
            let resolution = resolution_for_range(range);
            let cell = (
                (world_x / resolution).floor() as i32,
                (world_y / resolution).floor() as i32,
            );
            match self.map.entry(cell) {
                Entry::Occupied(mut entry) => {
                    if point.z > entry.get().0 {
                        entry.get_mut().0 = point.z;
                        changed_cells.push((cell.0, cell.1, point.z, resolution));
                    }
                }
                Entry::Vacant(entry) => {
                    entry.insert((point.z, resolution));
                    changed_cells.push((cell.0, cell.1, point.z, resolution));
                }
            }
        }
        if self
            .db_tx
            .send(DbCommand::SaveFrame {
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

fn database_path() -> Result<PathBuf, Box<dyn std::error::Error + Send + Sync>> {
    let data_root = std::env::var_os("LOCALAPPDATA")
        .or_else(|| std::env::var_os("HOME"))
        .ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "A local data directory is not available",
            )
        })?;
    let data_dir = PathBuf::from(data_root).join("TacticalMapper");
    std::fs::create_dir_all(&data_dir)?;
    Ok(data_dir.join("tactical_mapper.sqlite3"))
}

fn run_db_worker(
    rx: mpsc::Receiver<DbCommand>,
    storage_ok: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let connection = Connection::open(database_path()?)?;
    connection.execute_batch(
        "CREATE TABLE IF NOT EXISTS vehicle_poses (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            frame_id INTEGER NOT NULL,
            pos_x REAL NOT NULL,
            pos_y REAL NOT NULL,
            pos_z REAL NOT NULL,
            yaw REAL NOT NULL,
            recorded_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
        );
        CREATE TABLE IF NOT EXISTS grid_map_cells (
            grid_x INTEGER NOT NULL,
            grid_y INTEGER NOT NULL,
            elevation REAL NOT NULL,
            resolution REAL NOT NULL,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            PRIMARY KEY (grid_x, grid_y)
        );",
    )?;
    storage_ok.store(true, Ordering::Release);
    while let Ok(command) = rx.recv() {
        let result = (|| -> rusqlite::Result<()> {
            let transaction = connection.unchecked_transaction()?;
            match command {
                DbCommand::SaveFrame { frame, pose, cells } => {
                    transaction.execute(
                        "INSERT INTO vehicle_poses (frame_id, pos_x, pos_y, pos_z, yaw) VALUES ($1, $2, $3, $4, $5)",
                        params![frame as i64, pose.x, pose.y, pose.z, pose.yaw],
                    )?;
                    for (gx, gy, elevation, resolution) in cells {
                        transaction.execute(
                            "INSERT INTO grid_map_cells (grid_x, grid_y, elevation, resolution) VALUES ($1, $2, $3, $4)
                             ON CONFLICT (grid_x, grid_y) DO UPDATE SET elevation = excluded.elevation, resolution = excluded.resolution, updated_at = CURRENT_TIMESTAMP",
                            params![gx, gy, elevation, resolution],
                        )?;
                    }
                }
            }
            transaction.commit()
        })();
        if let Err(error) = result {
            storage_ok.store(false, Ordering::Release);
            eprintln!("Local storage write failed: {error}");
        } else {
            storage_ok.store(true, Ordering::Release);
        };
    }
    Ok(())
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
            let color = egui::Color32::from_rgb(
                (70.0 + point.intensity * 100.0) as u8,
                (170.0 + point.intensity * 70.0) as u8,
                (145.0 + point.intensity * 70.0) as u8,
            );
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
            let dt = ctx.input(|input| input.stable_dt);
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

    const DB_CHANNEL_CAPACITY: usize = 32;
    let (db_tx, db_rx) = mpsc::sync_channel(DB_CHANNEL_CAPACITY);
    let storage_ok = Arc::new(AtomicBool::new(false));
    let worker_storage_ok = Arc::clone(&storage_ok);
    std::thread::spawn(move || {
        if let Err(error) = run_db_worker(db_rx, worker_storage_ok) {
            eprintln!("Local storage stopped: {error}");
        }
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
}
