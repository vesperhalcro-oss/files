use eframe::egui;
use std::collections::HashMap;
use std::sync::mpsc;
use std::time::Instant;
use tokio_postgres::{Client, NoTls};

// ==========================================
// 1. GPS-Denied Sensor Pose Frame
// ==========================================

#[derive(Debug, Clone, Copy, Default)]
struct Pose3D {
    x: f32,
    y: f32,
    z: f32,
    yaw: f32,
}

impl Pose3D {
    fn normalize_yaw(&mut self) {
        use std::f32::consts::PI;
        self.yaw = (self.yaw + PI).rem_euclid(2.0 * PI) - PI;
    }
}

fn update_dead_reckoning(current: &mut Pose3D, accel_x: f32, gyro_z: f32, dt: f32) {
    current.yaw += gyro_z * dt;
    current.normalize_yaw();
    let (s, c) = current.yaw.sin_cos();
    current.x += accel_x * dt * c;
    current.y += accel_x * dt * s;
}

// ==========================================
// 2. Air-Gapped Local Engine with DB Dispatch
// ==========================================

const RES_NEAR: f32 = 0.05;
const RES_MID: f32 = 0.10;
const RES_FAR: f32 = 0.50;
const NEAR_DIST_SQ: f32 = 100.0;
const MID_DIST_SQ: f32 = 900.0;

#[derive(Debug, Clone)]
enum DbCommand {
    SavePose { frame: u64, pose: Pose3D },
    UpsertCell { gx: i32, gy: i32, elevation: f32, resolution: f32 },
}

struct Engine {
    pose: Pose3D,
    map: HashMap<(i32, i32), (f32, f32)>, // Key: (gx, gy) -> Value: (elevation, resolution)
    frame: u64,
    start: Instant,
    db_tx: mpsc::Sender<DbCommand>,
}

impl Engine {
    fn new(db_tx: mpsc::Sender<DbCommand>) -> Self {
        Self {
            pose: Pose3D::default(),
            map: HashMap::with_capacity(4096),
            frame: 0,
            start: Instant::now(),
            db_tx,
        }
    }

    fn step(&mut self, dt: f32) {
        update_dead_reckoning(&mut self.pose, 0.5, 0.01, dt);

        // Dispatch updated pose to local database task
        let _ = self.db_tx.send(DbCommand::SavePose {
            frame: self.frame,
            pose: self.pose,
        });

        // Stand-in for incoming sensor point cloud
        let points = [[2.0, 1.0, 0.1], [5.0, -3.0, 0.4], [12.0, 8.0, 1.2]];

        let (s, c) = self.pose.yaw.sin_cos();
        let (px, py, pz) = (self.pose.x, self.pose.y, self.pose.z);

        for pt in points.iter() {
            let wx = px + (pt[0] * c - pt[1] * s);
            let wy = py + (pt[0] * s + pt[1] * c);
            let wz = pz + pt[2];

            let dist_sq = pt[0] * pt[0] + pt[1] * pt[1];
            let res = if dist_sq <= NEAR_DIST_SQ {
                RES_NEAR
            } else if dist_sq <= MID_DIST_SQ {
                RES_MID
            } else {
                RES_FAR
            };

            let gx = (wx / res).floor() as i32;
            let gy = (wy / res).floor() as i32;

            let entry = self.map.entry((gx, gy)).or_insert((wz, res));
            if wz > entry.0 {
                entry.0 = wz;
            }

            // Persist cell to PostgreSQL database
            let _ = self.db_tx.send(DbCommand::UpsertCell {
                gx,
                gy,
                elevation: entry.0,
                resolution: res,
            });
        }
        self.frame += 1;
    }
}

// ==========================================
// 3. PostgreSQL Async Storage Task
// ==========================================

async fn run_db_worker(mut rx: mpsc::Receiver<DbCommand>) -> Result<(), tokio_postgres::Error> {
    // Localhost connection string
    let connection_str = "host=127.0.0.1 user=postgres password=postgres dbname=tactical_mapper_db";
    let (client, connection) = tokio_postgres::connect(connection_str, NoTls).await?;

    tokio::spawn(async move {
        if let Err(e) = connection.await {
            eprintln!("[DB ERROR] Connection fault: {}", e);
        }
    });

    println!("[DB] Successfully connected to PostgreSQL at 127.0.0.1");

    while let Ok(cmd) = rx.recv() {
        match cmd {
            DbCommand::SavePose { frame, pose } => {
                let _ = client
                    .execute(
                        "INSERT INTO vehicle_poses (frame_id, pos_x, pos_y, pos_z, yaw) VALUES ($1, $2, $3, $4, $5)",
                        &[&(frame as i64), &pose.x, &pose.y, &pose.z, &pose.yaw],
                    )
                    .await;
            }
            DbCommand::UpsertCell { gx, gy, elevation, resolution } => {
                let _ = client
                    .execute(
                        "INSERT INTO grid_map_cells (grid_x, grid_y, elevation, resolution)
                         VALUES ($1, $2, $3, $4)
                         ON CONFLICT (grid_x, grid_y)
                         DO UPDATE SET elevation = EXCLUDED.elevation, updated_at = CURRENT_TIMESTAMP",
                        &[&gx, &gy, &elevation, &resolution],
                    )
                    .await;
            }
        }
    }
    Ok(())
}

// ==========================================
// 4. GUI App
// ==========================================

struct App {
    engine: Engine,
    running: bool,
}

impl App {
    fn new(db_tx: mpsc::Sender<DbCommand>) -> Self {
        Self {
            engine: Engine::new(db_tx),
            running: true,
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.running {
            self.engine.step(0.033);
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("GPS-Denied Localhost Engine (PostgreSQL Sync Enabled)");
            ui.separator();
            ui.label(format!("Frame: {}", self.engine.frame));
            ui.label(format!(
                "Pose  x: {:.2}  y: {:.2}  yaw: {:.2} rad",
                self.engine.pose.x, self.engine.pose.y, self.engine.pose.yaw
            ));
            ui.label(format!("Active grid cells in RAM: {}", self.engine.map.len()));
            ui.label(format!("Elapsed: {:.2?}", self.engine.start.elapsed()));
            ui.separator();

            if ui.button(if self.running { "Pause" } else { "Resume" }).clicked() {
                self.running = !self.running;
            }
            ui.separator();

            // Top-down visualization panel
            let (rect, _resp) = ui.allocate_exact_size(egui::vec2(400.0, 400.0), egui::Sense::hover());
            let painter = ui.painter_at(rect);
            let center = rect.center();
            let scale = 15.0; // pixels per meter

            for (&(gx, gy), &(_h, res)) in self.engine.map.iter() {
                let wx = gx as f32 * res;
                let wy = gy as f32 * res;
                let p = center + egui::vec2(wx * scale, -wy * scale);
                painter.circle_filled(p, 1.5, egui::Color32::LIGHT_GREEN);
            }

            let vp = center + egui::vec2(self.engine.pose.x * scale, -self.engine.pose.y * scale);
            painter.circle_filled(vp, 4.0, egui::Color32::RED);
        });

        ctx.request_repaint();
    }
}

// ==========================================
// 5. Entry Point
// ==========================================

fn main() -> eframe::Result<()> {
    println!("[STATUS] System booted in AIR-GAPPED GPS-DENIED mode.");

    // Channel for decouled async PostgreSQL inserts
    let (db_tx, db_rx) = mpsc::channel::<DbCommand>();

    // Spawn async Tokio runtime for PostgreSQL worker thread
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            if let Err(e) = run_db_worker(db_rx).await {
                eprintln!("[DB FATAL] Database thread failed: {}", e);
            }
        });
    });

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Localhost Engine with DB Persistence",
        native_options,
        Box::new(|_cc| Ok(Box::new(App::new(db_tx)))),
    )
}