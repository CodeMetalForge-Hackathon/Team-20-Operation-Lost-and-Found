use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum SensorType {
    Thermal,
    Ir,
    Acoustic,
    Vision,
}

struct Spec {
    max_range_m: f64,
    peak_pd: f64,  // P(detect | present, distance=0)
    pfa: f64,      // P(detect | absent)
}

impl SensorType {
    fn spec(&self) -> Spec {
        match self {
            SensorType::Thermal  => Spec { max_range_m: 10.0, peak_pd: 0.90, pfa: 0.05 },
            SensorType::Ir       => Spec { max_range_m:  3.0, peak_pd: 0.95, pfa: 0.02 },
            SensorType::Acoustic => Spec { max_range_m: 20.0, peak_pd: 0.75, pfa: 0.08 },
            SensorType::Vision   => Spec { max_range_m: 15.0, peak_pd: 0.85, pfa: 0.03 },
        }
    }

    /// P(detect | subject present) at given distance.
    /// Quadratic falloff; zero beyond max_range.
    pub fn pd(&self, distance_m: f64) -> f64 {
        let s = self.spec();
        if distance_m > s.max_range_m { return 0.0; }
        let t = distance_m / s.max_range_m;
        s.peak_pd * (1.0 - t * t)
    }

    /// Likelihood ratio: P(positive_reading | present at d) / P(positive_reading | absent).
    /// Used to update cells on a DETECTION event.
    pub fn lr_positive(&self, distance_m: f64) -> f64 {
        let pd  = self.pd(distance_m);
        let pfa = self.spec().pfa;
        if pfa < 1e-12 { return pd / 1e-12; }
        pd / pfa
    }

    /// Likelihood ratio for NO detection.
    /// For cells outside range: returns 1.0 (no information).
    pub fn lr_negative(&self, distance_m: f64) -> f64 {
        let s = self.spec();
        if distance_m > s.max_range_m { return 1.0; }
        let pd = self.pd(distance_m);
        (1.0 - pd) / (1.0 - s.pfa)
    }

    pub fn name(&self) -> &'static str {
        match self {
            SensorType::Thermal  => "thermal",
            SensorType::Ir       => "ir",
            SensorType::Acoustic => "acoustic",
            SensorType::Vision   => "vision",
        }
    }
}
