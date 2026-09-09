use eframe::egui;
use rusqlite::{params, Connection};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::mpsc;
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

#[derive(Debug, Clone)]
enum DbCommand {
    SavePose {
        frame: u64,
        pose: Pose3D,
    },
    UpsertCell {
        gx: i32,
        gy: i32,
        elevation: f32,
        resolution: f32,
    },
}

struct Engine {
    pose: Pose3D,
    map: HashMap<(i32, i32), (f32, f32)>,
    points: Vec<LidarPoint>,
    trail: Vec<(f32, f32)>,
    frame: u64,
    simulation_time: f32,
    start: Instant,
    db_tx: mpsc::Sender<DbCommand>,
}

impl Engine {
    fn new(db_tx: mpsc::Sender<DbCommand>) -> Self {
        Self {
            pose: Pose3D::default(),
            map: HashMap::with_capacity(4096),
            points: Vec::with_capacity(256),
            trail: Vec::with_capacity(2048),
            frame: 0,
            simulation_time: 0.0,
            start: Instant::now(),
            db_tx,
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
        self.trail.push((self.pose.x, self.pose.y));
        if self.trail.len() > 2048 {
            self.trail.remove(0);
        }

        let _ = self.db_tx.send(DbCommand::SavePose {
            frame: self.frame,
            pose: self.pose,
        });

        self.points.clear();
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
            let resolution = if range <= 10.0 {
                RES_NEAR
            } else if range <= 30.0 {
                RES_MID
            } else {
                RES_FAR
            };
            let cell = (
                (world_x / resolution).floor() as i32,
                (world_y / resolution).floor() as i32,
            );
            let entry = self.map.entry(cell).or_insert((point.z, resolution));
            entry.0 = entry.0.max(point.z);
            let _ = self.db_tx.send(DbCommand::UpsertCell {
                gx: cell.0,
                gy: cell.1,
                elevation: entry.0,
                resolution,
            });
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
    let data_dir = PathBuf::from(data_root).join("FoveatedLiDAR");
    std::fs::create_dir_all(&data_dir)?;
    Ok(data_dir.join("foveated_lidar.sqlite3"))
}

fn run_db_worker(
    rx: mpsc::Receiver<DbCommand>,
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
    println!("[DB] Using local SQLite database");

    while let Ok(command) = rx.recv() {
        let result = match command {
            DbCommand::SavePose { frame, pose } => connection
                .execute(
                    "INSERT INTO vehicle_poses (frame_id, pos_x, pos_y, pos_z, yaw) VALUES ($1, $2, $3, $4, $5)",
                    params![frame as i64, pose.x, pose.y, pose.z, pose.yaw],
                ),
            DbCommand::UpsertCell {
                gx,
                gy,
                elevation,
                resolution,
            } => connection
                .execute(
                    "INSERT INTO grid_map_cells (grid_x, grid_y, elevation, resolution) VALUES ($1, $2, $3, $4)
                     ON CONFLICT (grid_x, grid_y) DO UPDATE SET elevation = excluded.elevation, updated_at = CURRENT_TIMESTAMP",
                    params![gx, gy, elevation, resolution],
                ),
        };
        if let Err(error) = result {
            eprintln!("[DB ERROR] Write failed: {error}");
        }
    }
    Ok(())
}

struct App {
    engine: Engine,
    running: bool,
    speed: f32,
}

impl App {
    fn draw_map(&self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let size = available.x.min(available.y).max(320.0);
        let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let center = rect.center();
        let scale = size / 70.0;

        painter.rect_filled(rect, 6.0, egui::Color32::from_rgb(12, 22, 30));
        for step in -30..=30 {
            let offset = step as f32 * scale;
            painter.line_segment(
                [
                    center + egui::vec2(offset, -30.0 * scale),
                    center + egui::vec2(offset, 30.0 * scale),
                ],
                egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(27, 49, 59)),
            );
            painter.line_segment(
                [
                    center + egui::vec2(-30.0 * scale, offset),
                    center + egui::vec2(30.0 * scale, offset),
                ],
                egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(27, 49, 59)),
            );
        }
        for pair in self.engine.trail.windows(2) {
            let from = center + egui::vec2(pair[0].0 * scale, -pair[0].1 * scale);
            let to = center + egui::vec2(pair[1].0 * scale, -pair[1].1 * scale);
            painter.line_segment(
                [from, to],
                egui::Stroke::new(2.0_f32, egui::Color32::LIGHT_BLUE),
            );
        }
        for point in &self.engine.points {
            let (sin_yaw, cos_yaw) = self.engine.pose.yaw.sin_cos();
            let world_x = self.engine.pose.x + point.x * cos_yaw - point.y * sin_yaw;
            let world_y = self.engine.pose.y + point.x * sin_yaw + point.y * cos_yaw;
            let position = center + egui::vec2(world_x * scale, -world_y * scale);
            let color = egui::Color32::from_rgb(
                (80.0 + point.intensity * 175.0) as u8,
                (120.0 + point.intensity * 120.0) as u8,
                70,
            );
            painter.circle_filled(position, 2.0 + point.z * 0.4, color);
        }
        let vehicle = center + egui::vec2(self.engine.pose.x * scale, -self.engine.pose.y * scale);
        painter.circle_filled(vehicle, 6.0, egui::Color32::from_rgb(240, 80, 70));
        let direction = egui::vec2(self.engine.pose.yaw.cos(), -self.engine.pose.yaw.sin()) * 14.0;
        painter.line_segment(
            [vehicle, vehicle + direction],
            egui::Stroke::new(3.0_f32, egui::Color32::WHITE),
        );
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.running {
            self.engine.step(1.0 / 60.0, self.speed);
        }
        egui::SidePanel::left("controls")
            .min_width(220.0)
            .show(ctx, |ui| {
                ui.heading("Simulation");
                ui.separator();
                ui.label("Mock LiDAR stream");
                ui.colored_label(egui::Color32::LIGHT_GREEN, "● Online");
                ui.add(egui::Slider::new(&mut self.speed, 0.25..=4.0).text("Speed"));
                if ui
                    .button(if self.running {
                        "Pause simulation"
                    } else {
                        "Resume simulation"
                    })
                    .clicked()
                {
                    self.running = !self.running;
                }
                if ui.button("Reset mock run").clicked() {
                    self.engine.reset();
                }
                ui.separator();
                ui.heading("Live telemetry");
                ui.label(format!("Frame: {}", self.engine.frame));
                ui.label(format!("LiDAR points: {}", self.engine.points.len()));
                ui.label(format!("Map cells: {}", self.engine.map.len()));
                ui.label(format!("X: {:.2} m", self.engine.pose.x));
                ui.label(format!("Y: {:.2} m", self.engine.pose.y));
                ui.label(format!("Yaw: {:.2} rad", self.engine.pose.yaw));
                ui.label(format!(
                    "Elapsed: {:.1} s",
                    self.engine.start.elapsed().as_secs_f32()
                ));
                ui.separator();
                ui.small(
                    "The simulator runs locally and saves data to an embedded SQLite database.",
                );
            });
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Foveated LiDAR Map");
            ui.label("Live mock scan, vehicle trail, and reconstructed occupancy grid");
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
    if std::env::args().any(|argument| argument == "--installed") {
        return Ok(false);
    }

    let current_exe = std::env::current_exe()?;
    let install_dir = PathBuf::from(std::env::var_os("LOCALAPPDATA").ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "LOCALAPPDATA is not available",
        )
    })?)
    .join("FoveatedLiDAR");
    std::fs::create_dir_all(&install_dir)?;
    let installed_exe = install_dir.join("FoveatedLiDAR.exe");
    let is_installed = installed_exe
        .canonicalize()
        .ok()
        .zip(current_exe.canonicalize().ok())
        .is_some_and(|(installed, current)| installed == current);

    if !is_installed {
        std::fs::copy(&current_exe, &installed_exe)?;
    }

    let desktop = PathBuf::from(std::env::var_os("USERPROFILE").ok_or_else(|| {
        std::io::Error::new(std::io::ErrorKind::NotFound, "USERPROFILE is not available")
    })?)
    .join("Desktop")
    .join("Foveated LiDAR.lnk");
    let script = format!(
        "$shell = New-Object -ComObject WScript.Shell; \
         $shortcut = $shell.CreateShortcut('{desktop}'); \
         $shortcut.TargetPath = '{target}'; \
         $shortcut.WorkingDirectory = '{working}'; \
         $shortcut.Description = 'Foveated LiDAR Map Simulator'; \
         $shortcut.Save()",
        desktop = powershell_literal(&desktop),
        target = powershell_literal(&installed_exe),
        working = powershell_literal(&install_dir),
    );
    let shortcut_status = std::process::Command::new("powershell")
        .args([
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            &script,
        ])
        .status()?;
    if !shortcut_status.success() {
        return Err("Could not create the Foveated LiDAR desktop shortcut.".into());
    }

    if !is_installed {
        std::process::Command::new(&installed_exe)
            .arg("--installed")
            .spawn()?;
        println!("Installed Foveated LiDAR to {}", installed_exe.display());
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
            eprintln!("[SETUP ERROR] {error}");
            return Err(eframe::Error::AppCreation(error));
        }
    };
    if was_installed {
        return Ok(());
    }
    if std::env::args().any(|argument| argument == "--installed") {
        println!("[SETUP] Running the installed copy.");
    }
    let (db_tx, db_rx) = mpsc::channel();
    std::thread::spawn(move || {
        if let Err(error) = run_db_worker(db_rx) {
            eprintln!("[DB FATAL] {error}");
        }
    });
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 760.0])
            .with_min_inner_size([800.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Foveated LiDAR Map Simulator",
        options,
        Box::new(|_| {
            Ok(Box::new(App {
                engine: Engine::new(db_tx),
                running: true,
                speed: 1.0,
            }))
        }),
    )
}
