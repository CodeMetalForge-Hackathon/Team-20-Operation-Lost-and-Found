use serde::{Deserialize, Serialize};
use crate::{
    belief_map::BeliefMap,
    dead_reckoning::DeadReckoning,
    hazard::{DangerAlert, HazardDetector},
    path_planner::PathPlanner,
    sensor_model::SensorType,
};

// ── Wire protocol ────────────────────────────────────────────────────────────

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum IncomingMsg {
    /// (Re)configure the grid.  Resets all state.
    Config {
        width: usize,
        height: usize,
        cell_size_m: f64,
        last_known_x: f64,
        last_known_y: f64,
        prior_sigma_m: Option<f64>,
    },
    /// One sensor reading from the current robot position.
    Sensor {
        sensor_type: SensorType,
        detection: bool,
        confidence: f64,
    },
    /// Raw IMU sample.
    Imu {
        ax: f64,
        ay: f64,
        az: f64,   // vertical accel (m/s²) — ~9.81 when flat+stationary
        gz: f64,
        dt: f64,
    },
    /// Directly set robot world position (simulation / GPS fix).
    Teleport {
        x: f64,
        y: f64,
    },
    /// Return current state without modifying it.
    Query,
    /// Hard reset to default engine state.
    Reset,
}

// ── Output types ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize)]
pub struct Contact {
    pub cell_x: usize,
    pub cell_y: usize,
    pub world_x: f64,
    pub world_y: f64,
    pub probability: f64,
    pub sensors: Vec<String>,
    pub combined_confidence: f64,
}

#[derive(Debug, Serialize)]
pub struct StateSnapshot {
    pub grid: Vec<Vec<f64>>,
    pub robot_x: f64,
    pub robot_y: f64,
    pub robot_cell: [usize; 2],
    pub next_waypoint: Option<[usize; 2]>,
    pub entropy: f64,
    pub max_probability: f64,
    pub sweep_count: usize,
    pub contact: Option<Contact>,
    pub elevation: f64,
    pub hazard_alerts: Vec<DangerAlert>,
}

// ── Engine ───────────────────────────────────────────────────────────────────

const CONTACT_THRESHOLD: f64 = 0.20;

pub struct GhostEngine {
    pub map:         BeliefMap,
    pub dr:          DeadReckoning,
    pub hazard:      HazardDetector,
    pub sweep_count: usize,
    pub contact:     Option<Contact>,
    /// Accumulates sensor triggers between position changes for fusion.
    pending: Vec<(SensorType, f64)>,
}

impl Default for GhostEngine {
    fn default() -> Self { Self::new() }
}

impl GhostEngine {
    pub fn new() -> Self {
        let map = BeliefMap::new(20, 20, 2.0);
        let dr  = DeadReckoning::new(0.0, 0.0);
        Self {
            map, dr,
            hazard:      HazardDetector::new(),
            sweep_count: 0,
            contact:     None,
            pending:     Vec::new(),
        }
    }

    pub fn handle(&mut self, msg: IncomingMsg) {
        match msg {
            IncomingMsg::Config {
                width, height, cell_size_m,
                last_known_x, last_known_y, prior_sigma_m,
            } => {
                self.map = BeliefMap::new(width, height, cell_size_m);
                self.map.init_gaussian_prior(
                    last_known_x, last_known_y,
                    prior_sigma_m.unwrap_or(15.0),
                );
                self.dr           = DeadReckoning::new(0.0, 0.0);
                self.hazard       = HazardDetector::new();
                self.sweep_count  = 0;
                self.contact      = None;
                self.pending.clear();
                log::info!("configured: {}x{} @ {}m/cell, LKP=({},{})",
                    width, height, cell_size_m, last_known_x, last_known_y);
            }

            IncomingMsg::Imu { ax, ay, az, gz, dt } => {
                self.dr.update(ax, ay, az, gz, dt);
                self.hazard.check(
                    self.dr.pos_x,
                    self.dr.pos_y,
                    self.dr.altitude,
                    self.dr.altitude_rate,
                    self.dr.lateral_g,
                );
            }

            IncomingMsg::Teleport { x, y } => {
                self.dr.teleport(x, y);
                log::debug!("teleport → ({x:.1}, {y:.1})");
            }

            IncomingMsg::Sensor { sensor_type, detection, confidence } => {
                let cell = self.dr.grid_cell(
                    self.map.cell_size_m, self.map.width, self.map.height,
                );
                let factors = PathPlanner::likelihood_factors(
                    &self.map, cell, &sensor_type, detection, confidence,
                );
                self.map.update_likelihoods(&factors);
                self.map.mark_swept(cell.0, cell.1);
                self.sweep_count += 1;

                if detection {
                    self.pending.push((sensor_type, confidence));
                }
                self.check_contact(cell);

                log::debug!(
                    "sweep #{} cell({},{}) detection={} → entropy={:.3}",
                    self.sweep_count, cell.0, cell.1, detection,
                    self.map.entropy()
                );
            }

            IncomingMsg::Query | IncomingMsg::Reset => {
                if matches!(msg, IncomingMsg::Reset) {
                    *self = GhostEngine::new();
                }
            }
        }
    }

    pub fn snapshot(&self) -> StateSnapshot {
        let cell = self.dr.grid_cell(
            self.map.cell_size_m, self.map.width, self.map.height,
        );

        // Build per-cell hazard + SAR-priority weights for the path planner.
        let n = self.map.width * self.map.height;
        let mut hazard_weights = Vec::with_capacity(n);
        let mut sar_boosts     = Vec::with_capacity(n);
        for i in 0..n {
            let x  = i % self.map.width;
            let y  = i / self.map.width;
            let wx = x as f64 * self.map.cell_size_m;
            let wy = y as f64 * self.map.cell_size_m;
            hazard_weights.push(self.hazard.terrain_weight(wx, wy));
            sar_boosts.push(self.hazard.sar_priority(wx, wy));
        }

        let wp = PathPlanner::next_waypoint(
            &self.map, cell, &hazard_weights, &sar_boosts,
        );
        let (_, _, max_p) = self.map.peak();

        StateSnapshot {
            grid:            self.map.as_grid(),
            robot_x:         self.dr.pos_x,
            robot_y:         self.dr.pos_y,
            robot_cell:      [cell.0, cell.1],
            next_waypoint:   wp.map(|(x, y)| [x, y]),
            entropy:         self.map.entropy(),
            max_probability: max_p,
            sweep_count:     self.sweep_count,
            contact:         self.contact.clone(),
            elevation:       self.dr.altitude,
            hazard_alerts:   self.hazard.alerts.clone(),
        }
    }

    // ── private ──────────────────────────────────────────────────────────────

    fn check_contact(&mut self, _robot_cell: (usize, usize)) {
        if self.contact.is_some() { return; }

        // Check the global map peak — the subject doesn't have to be
        // right under the robot for us to declare a contact.
        let (px, py, p) = self.map.peak();
        if p < CONTACT_THRESHOLD { return; }

        // Combined confidence: 1 − Π(1 − cᵢ)
        let combined = if self.pending.is_empty() {
            p
        } else {
            1.0 - self.pending.iter().map(|(_, c)| 1.0 - c).product::<f64>()
        };

        let sensors: Vec<String> = self.pending.iter()
            .map(|(s, _)| s.name().to_string())
            .collect();

        log::warn!(
            "CONTACT declared at cell({},{}) p={:.3} conf={:.3} sensors={:?}",
            px, py, p, combined, sensors
        );

        self.contact = Some(Contact {
            cell_x: px, cell_y: py,
            world_x: px as f64 * self.map.cell_size_m,
            world_y: py as f64 * self.map.cell_size_m,
            probability: p,
            combined_confidence: combined,
            sensors,
        });
    }
}
