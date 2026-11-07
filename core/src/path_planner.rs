use crate::{belief_map::BeliefMap, sensor_model::SensorType};

pub struct PathPlanner;

impl PathPlanner {
    /// Next cell to visit: maximises P(c) / (1 + travel_weight * manhattan_dist)
    /// Skips already-swept cells.
    pub fn next_waypoint(
        map: &BeliefMap,
        robot_cell: (usize, usize),
    ) -> Option<(usize, usize)> {
        let (rx, ry) = robot_cell;
        let mut best = f64::NEG_INFINITY;
        let mut target = None;

        for y in 0..map.height {
            for x in 0..map.width {
                if map.is_swept(x, y) { continue; }
                let p = map.get(x, y);
                if p < 1e-15 { continue; }

                let dist = ((x as isize - rx as isize).unsigned_abs()
                    + (y as isize - ry as isize).unsigned_abs()) as f64;
                let score = p / (1.0 + 0.08 * dist);

                if score > best {
                    best = score;
                    target = Some((x, y));
                }
            }
        }
        target
    }

    /// Compute per-cell likelihood-ratio factors for a sensor sweep.
    ///
    /// For every cell c:
    ///   factor(c) = LR_positive(dist(robot, c))   if detection == true
    ///             = LR_negative(dist(robot, c))    if detection == false
    ///
    /// `confidence` ∈ [0,1] blends the LR toward 1.0 (no update) for
    /// low-confidence readings (e.g., partial occlusion).
    pub fn likelihood_factors(
        map: &BeliefMap,
        robot_cell: (usize, usize),
        sensor: &SensorType,
        detection: bool,
        confidence: f64,
    ) -> Vec<f64> {
        let (rx, ry) = robot_cell;
        let cell_m = map.cell_size_m;
        let conf = confidence.clamp(0.0, 1.0);

        (0..map.height)
            .flat_map(|y| {
                (0..map.width).map(move |x| {
                    let dx = (x as f64 - rx as f64) * cell_m;
                    let dy = (y as f64 - ry as f64) * cell_m;
                    let dist = (dx * dx + dy * dy).sqrt();

                    let raw_lr = if detection {
                        sensor.lr_positive(dist)
                    } else {
                        sensor.lr_negative(dist)
                    };
                    // Blend: lr = 1 + conf*(raw - 1)
                    1.0 + conf * (raw_lr - 1.0)
                })
            })
            .collect()
    }
}
