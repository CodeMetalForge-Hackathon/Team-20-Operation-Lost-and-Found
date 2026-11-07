/// GPS-free position estimate via IMU integration.
/// Uses a simple Madgwick-style heading update + trapezoidal
/// velocity/position integration.  Zero-velocity detection from
/// accelerometer magnitude damps drift when the robot is stationary.
#[derive(Debug, Clone)]
pub struct DeadReckoning {
    pub pos_x: f64,    // metres from origin
    pub pos_y: f64,
    pub vel_x: f64,    // m/s
    pub vel_y: f64,
    pub heading: f64,  // radians, 0 = +x axis, CCW positive
}

impl DeadReckoning {
    pub fn new(start_x: f64, start_y: f64) -> Self {
        Self { pos_x: start_x, pos_y: start_y, vel_x: 0.0, vel_y: 0.0, heading: 0.0 }
    }

    /// Process one IMU sample.
    /// ax, ay : body-frame acceleration (m/s², gravity already removed)
    /// gz      : yaw rate (rad/s, +CCW)
    /// dt      : sample interval (s)
    pub fn update(&mut self, ax: f64, ay: f64, gz: f64, dt: f64) {
        // Integrate heading
        self.heading = (self.heading + gz * dt).rem_euclid(std::f64::consts::TAU);

        // Rotate body-frame accel into world frame
        let (sh, ch) = self.heading.sin_cos();
        let world_ax = ch * ax - sh * ay;
        let world_ay = sh * ax + ch * ay;

        // Zero-velocity update: damp velocity when accel magnitude is near zero
        let mag = (ax * ax + ay * ay).sqrt();
        let decay = if mag < 0.08 { 0.80 } else { 1.0 };

        self.vel_x = self.vel_x * decay + world_ax * dt;
        self.vel_y = self.vel_y * decay + world_ay * dt;

        self.pos_x += self.vel_x * dt;
        self.pos_y += self.vel_y * dt;
    }

    /// Hard-set position (used by sim / external fix).
    pub fn teleport(&mut self, x: f64, y: f64) {
        self.pos_x = x;
        self.pos_y = y;
        self.vel_x = 0.0;
        self.vel_y = 0.0;
    }

    /// Convert world position to grid cell, clamped to map bounds.
    pub fn grid_cell(&self, cell_size_m: f64, width: usize, height: usize) -> (usize, usize) {
        let cx = (self.pos_x / cell_size_m).max(0.0) as usize;
        let cy = (self.pos_y / cell_size_m).max(0.0) as usize;
        (cx.min(width.saturating_sub(1)), cy.min(height.saturating_sub(1)))
    }

    /// Terrain slope estimate from vertical accel spike (simple heuristic).
    /// Returns passability weight 0–1 (1 = flat, lower = steep).
    pub fn slope_weight(az_body: f64) -> f64 {
        // az_body is deviation from 1g; large deviation → steep terrain
        let deviation = (az_body - 9.81).abs();
        (1.0 - (deviation / 9.81)).clamp(0.1, 1.0)
    }
}
