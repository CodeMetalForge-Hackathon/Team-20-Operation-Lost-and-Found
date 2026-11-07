# GHOST — Bayesian SAR Path Planner

GPS-free, infrastructure-free search and rescue for ground drones. GHOST maintains a live probability map of where a subject most likely is, fuses evidence from four sensor modalities, flags terrain hazards in real time, and always routes the robot toward the highest-value unsearched cell.

**No GPS. No cell signal. No infrastructure.**

---

## Quick start

```bash
# 1 — build and run the Rust core (WebSocket on :9001)
cd core && cargo run

# 2 — start the sensor fusion layer (stub demo, no hardware needed)
cd sensors && python3 fusion.py

# 3 — open the live dashboard
open dashboard/index.html
```

The stub demo auto-navigates toward the subject at (20 m, 20 m), fires all four sensors with realistic probability falloff, and declares **CONTACT** when the belief converges.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  dashboard/index.html   (browser, no build step)        │
│  Canvas heatmap · hazard overlay · sensor status        │
└────────────────────────┬────────────────────────────────┘
                         │  WebSocket  ws://localhost:9001/ws
┌────────────────────────▼────────────────────────────────┐
│  sensors/fusion.py      (Python, runs on Pi / laptop)   │
│  IMU · Thermal · Vision · Acoustic · IR                 │
│  Hardware-or-stub on every sensor module                │
└────────────────────────┬────────────────────────────────┘
                         │  WebSocket (same connection)
┌────────────────────────▼────────────────────────────────┐
│  core/  (Rust, single binary — ghost-core)              │
│                                                         │
│  BeliefMap      Bayesian grid, Gaussian prior           │
│  SensorModel    SensorSpec physics, LR fusion           │
│  PathPlanner    info-gain waypoint scoring              │
│  DeadReckoning  IMU integration, altitude tracking      │
│  HazardDetector cliff / ravine / slope / obstacle       │
│  GhostEngine    stateless handler — msg in, snap out    │
│  axum WS server :9001                                   │
└─────────────────────────────────────────────────────────┘
```

See [ADR-001](docs/adr/0001-layered-architecture.md) for why the layers are split this way.

---

## How it works

Every grid cell starts with a Gaussian probability prior centred on the last-known position. As the robot sweeps, each sensor vote multiplies that cell's probability using a Bayesian likelihood ratio:

```
LR+ = Pd(dist) / Pfa          # detection
LR− = (1 − Pd(dist)) / (1 − Pfa)   # no detection
```

Renormalize after every update. When the global peak crosses **P > 0.20** with multi-sensor agreement, GHOST declares **CONTACT** — grid cell, world coordinates, combined confidence.

The path planner scores unswept cells by `P(c) × hazard_weight(c) × sar_boost(c) / (1 + 0.08 × dist)` and routes the robot to the highest-value cell.

---

## Default sensor specs

| Sensor | Range | Peak Pd | Pfa |
|--------|-------|---------|-----|
| Thermal (MLX90640) | 10 m | 0.90 | 0.05 |
| IR / PIR | 3 m | 0.95 | 0.02 |
| Acoustic (VAD 85–255 Hz) | 20 m | 0.75 | 0.08 |
| Vision (YOLOv8-nano) | 15 m | 0.85 | 0.03 |

Specs are runtime-overridable via the `Config` message — no recompile needed:

```json
{
  "type": "config",
  "width": 20, "height": 20, "cell_size_m": 0.5,
  "last_known_x": 5.0, "last_known_y": 5.0,
  "sensor_specs": {
    "thermal": { "max_range_m": 5.0, "peak_pd": 0.93, "pfa": 0.02 }
  }
}
```

---

## Hazard detection

The IMU's vertical accelerometer tracks elevation continuously. The `HazardDetector` classifies:

| Hazard | Trigger | Path-planner weight | SAR note |
|--------|---------|---------------------|----------|
| Cliff/drop | altitude rate < −0.5 m/s | 0.05 (avoid) | |
| Obstacle/wall | altitude rate > +0.5 m/s | 0.10 (avoid) | |
| Steep slope | lateral g > 0.30 g | 0.25 (penalise) | |
| Ravine bottom | descending fast → levelling | 0.80 (passable) | +50% SAR priority |

Hazard cells are rendered with coloured borders on the dashboard and routed around by the path planner.

---

## Running on hardware (Jetson + Pi)

```
Jetson:
  ./jetson/start.sh                    # ghost-core + Nav2 bridge + ROS2 nav stack
  GHOST_ADDR=0.0.0.0:9001 ./jetson/start.sh   # expose to Pi over LAN

Pi:
  cd sensors && python3 fusion.py --hardware --host <jetson-ip>
  python3 pi/operator_link.py --host <jetson-ip>  # LoRa operator comms
```

The bridge process (`jetson/ghost_bridge.py`) connects cuVSLAM odometry to GHOST Teleport, reads `next_waypoint` from every snapshot, and drives Nav2 there. CONTACT declarations are published to `/ghost/contact` and broadcast over LoRa to the field operator.

---

## Directory structure

```
ghost/
├── core/                   Rust — the engine
│   └── src/
│       ├── belief_map.rs   Bayesian probability grid
│       ├── sensor_model.rs SensorSpec physics + SensorType identity
│       ├── path_planner.rs info-gain waypoint + LR factor computation
│       ├── dead_reckoning.rs IMU integration, elevation tracking
│       ├── hazard.rs       terrain hazard detection + path weights
│       ├── engine.rs       GhostEngine — message handler + snapshot
│       ├── server.rs       axum WebSocket server
│       └── sim.rs          scripted headless demo (cargo run -- --sim)
├── sensors/                Python — Pi sensor abstraction
│   ├── fusion.py           main loop — hardware or stub mode
│   ├── imu.py              MPU-6050 / random-walk stub
│   ├── thermal.py          MLX90640 / Pd(dist) stub
│   ├── vision.py           YOLOv8-nano / Pd(dist) stub
│   ├── acoustic.py         PyAudio FFT VAD / Pd(dist) stub
│   └── ir.py               RPi.GPIO / 3 m-range stub
├── jetson/                 Jetson — navigation bridge
│   ├── ghost_bridge.py     cuVSLAM odom → GHOST Teleport + GHOST wp → Nav2
│   ├── nav2_driver.py      clean Nav2 BasicNavigator wrapper
│   ├── start.sh            launch script (core + bridge + ROS2 nav)
│   └── requirements.txt
├── pi/                     Pi — operator comms
│   ├── operator_link.py    LoRa (Meshtastic) ↔ GHOST status relay
│   └── requirements.txt
├── dashboard/
│   └── index.html          single-file live dashboard (no build step)
└── docs/
    └── adr/
        └── 0001-layered-architecture.md
```

---

## Hardware target

| Component | Role |
|-----------|------|
| Jetson Orin Nano | Runs `ghost-core`, YOLOv8 inference |
| Raspberry Pi 5 | GPIO sensors — IMU, thermal, IR, mic |
| MLX90640 | 32×24 thermal array, I2C |
| MPU-6050 | 6-DOF IMU, I2C |
| USB microphone | Acoustic VAD |
| Pi Camera + AI HAT | YOLOv8-nano person detection |

The Pi runs `fusion.py --hardware`; it connects to the Jetson's WebSocket over Ethernet or USB-C gadget mode.

---

## ROS2 integration

`ghost-core` is I/O-free and stateless by design — it plugs into any navigation stack via a thin bridge node.

`ghost_bridge.py` publishes two topics:

| Topic | Type | Content |
|-------|------|---------|
| `/ghost/contact` | `std_msgs/String` | JSON ContactDeclaration when GHOST declares a find |
| `/ghost/status` | `std_msgs/String` | JSON snapshot every ~250 ms (entropy, peak_p, sweeps, elevation, hazards) |

Swap `--odom /visual_slam/tracking/odometry` → `--odom /odom` to use wheel odometry instead of cuVSLAM.

---

## Branch status

This is the `ghost` working branch off `Team-20/main`.
Open a PR to merge when ready for team review.
