use eframe::egui;
use std::collections::HashMap;
use std::sync::mpsc;
use std::time::Instant;
use tokio_postgres::NoTls;

#[derive(Debug, Clone, Copy, Default)]
struct Pose3D { x: f32, y: f32, z: f32, yaw: f32 }

impl Pose3D {
    fn normalize_yaw(&mut self) {
        let pi = std::f32::consts::PI;
        self.yaw = (self.yaw + pi).rem_euclid(2.0 * pi) - pi;
    }
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
    SavePose { frame: u64, pose: Pose3D },
    UpsertCell { gx: i32, gy: i32, elevation: f32, resolution: f32 },
}

struct Engine {
    pose: Pose3D,
    map: HashMap<(i32, i32), (f32, f32)>,
    frame: u64,
    start: Instant,
    db_tx: mpsc::Sender<DbCommand>,
}

impl Engine {
    fn new(db_tx: mpsc::Sender<DbCommand>) -> Self {
        Self { pose: Pose3D::default(), map: HashMap::with_capacity(4096),
            frame: 0, start: Instant::now(), db_tx }
    }

    fn step(&mut self, dt: f32) {
        update_dead_reckoning(&mut self.pose, 0.5, 0.01, dt);
        let _ = self.db_tx.send(DbCommand::SavePose { frame: self.frame, pose: self.pose });
        let points = [[2.0, 1.0, 0.1], [5.0, -3.0, 0.4], [12.0, 8.0, 1.2]];
        let (sin_yaw, cos_yaw) = self.pose.yaw.sin_cos();
        for [x, y, z] in points {
            let world_x = self.pose.x + x * cos_yaw - y * sin_yaw;
            let world_y = self.pose.y + x * sin_yaw + y * cos_yaw;
            let elevation = self.pose.z + z;
            let distance_sq = x * x + y * y;
            let resolution = if distance_sq <= 100.0 { RES_NEAR } else if distance_sq <= 900.0 { RES_MID } else { RES_FAR };
            let cell = ((world_x / resolution).floor() as i32, (world_y / resolution).floor() as i32);
            let entry = self.map.entry(cell).or_insert((elevation, resolution));
            entry.0 = entry.0.max(elevation);
            let _ = self.db_tx.send(DbCommand::UpsertCell {
                gx: cell.0, gy: cell.1, elevation: entry.0, resolution,
            });
        }
        self.frame += 1;
    }
}

async fn run_db_worker(rx: mpsc::Receiver<DbCommand>) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let url = std::env::var("DATABASE_URL")
        .map_err(|error| {
            std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("DATABASE_URL is not set; configure it in .env: {error}"),
            )
        })?;
    let (client, connection) = tokio_postgres::connect(&url, NoTls).await?;
    tokio::spawn(async move {
        if let Err(error) = connection.await {
            eprintln!("[DB ERROR] Connection fault: {error}");
        }
    });
    println!("[DB] Connected to PostgreSQL");

    while let Ok(command) = rx.recv() {
        let result = match command {
            DbCommand::SavePose { frame, pose } => client.execute(
                "INSERT INTO vehicle_poses (frame_id, pos_x, pos_y, pos_z, yaw) VALUES ($1, $2, $3, $4, $5)",
                &[&(frame as i64), &pose.x, &pose.y, &pose.z, &pose.yaw],
            ).await,
            DbCommand::UpsertCell { gx, gy, elevation, resolution } => client.execute(
                "INSERT INTO grid_map_cells (grid_x, grid_y, elevation, resolution) VALUES ($1, $2, $3, $4)
                 ON CONFLICT (grid_x, grid_y) DO UPDATE SET elevation = EXCLUDED.elevation, updated_at = CURRENT_TIMESTAMP",
                &[&gx, &gy, &elevation, &resolution],
            ).await,
        };
        if let Err(error) = result {
            eprintln!("[DB ERROR] Write failed: {error}");
        }
    }
    Ok(())
}

struct App { engine: Engine, running: bool }

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.running { self.engine.step(0.033); }
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("GPS-Denied Localhost Engine (PostgreSQL Sync Enabled)");
            ui.label(format!("Frame: {}", self.engine.frame));
            ui.label(format!("Pose x: {:.2} y: {:.2} yaw: {:.2} rad", self.engine.pose.x, self.engine.pose.y, self.engine.pose.yaw));
            ui.label(format!("Active grid cells in RAM: {}", self.engine.map.len()));
            ui.label(format!("Elapsed: {:.2?}", self.engine.start.elapsed()));
            if ui.button(if self.running { "Pause" } else { "Resume" }).clicked() { self.running = !self.running; }
            let (rect, _) = ui.allocate_exact_size(egui::vec2(400.0, 400.0), egui::Sense::hover());
            let painter = ui.painter_at(rect);
            let center = rect.center();
            for (&(gx, gy), &(_, resolution)) in &self.engine.map {
                painter.circle_filled(center + egui::vec2(gx as f32 * resolution * 15.0, -gy as f32 * resolution * 15.0), 1.5, egui::Color32::LIGHT_GREEN);
            }
            painter.circle_filled(center + egui::vec2(self.engine.pose.x * 15.0, -self.engine.pose.y * 15.0), 4.0, egui::Color32::RED);
        });
        ctx.request_repaint();
    }
}

fn main() -> eframe::Result<()> {
    if let Err(error) = dotenvy::dotenv() {
        if !matches!(error, dotenvy::Error::Io(_)) { eprintln!("[CONFIG] Could not load .env: {error}"); }
    }
    let (db_tx, db_rx) = mpsc::channel();
    std::thread::spawn(move || {
        let runtime = tokio::runtime::Runtime::new().expect("failed to create Tokio runtime");
        if let Err(error) = runtime.block_on(run_db_worker(db_rx)) { eprintln!("[DB FATAL] {error}"); }
    });
    eframe::run_native("Localhost Engine with DB Persistence", eframe::NativeOptions::default(),
        Box::new(|_| Ok(Box::new(App { engine: Engine::new(db_tx), running: true }))))
}
