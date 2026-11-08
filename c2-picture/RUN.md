# Running the C2 operations picture (v0.2)

Requires a Java JDK 17 or newer. No external libraries.

```
cd c2-picture
./run.sh            # demo: simulated drones write reports into ./inbox
./run.sh --no-sim   # real reports only: drop drone report JSON files into ./inbox
```

## What you see

- Isometric 5 x 5 mile board (162 x 162 squares, 50 m each), synthetic terrain with trees and textures.
- Heat map: probability the subject is in each square, with a contour at 50% of the peak and a ring on the peak.
- Dashed cyan = planned routes (HOLO), one per UAV. Faint solid cyan = recent flown tracks.
- Red pulsing area = ENEMY AO. Red dotted loop = predicted patrol route.
- Translucent / dashed / "?" = HOLO (predicted). Filled = SOLID (confirmed).
- Loop: INIT plans 3 routes -> EXECUTE (UAVs shown HOLO along routes) -> UAVs land, reports arrive in `./inbox` -> belief updates, contacts and hostile sightings turn objects SOLID -> REGROUP -> replan.

## Keys

SPACE pause · +/- sim speed · H halt · R resume · P replan (in REGROUP) · arrows pan · Z/X zoom (4 levels)

## Files in and out

| Folder | File | What |
|---|---|---|
| `inbox/` | `*.json` | Drone reports (format: `docs/sample_drone_report.json`); moved to `inbox/processed/` once read |
| `out/` | `mission_order_W#-X.json` | Route, rules, intent, return deadline for each sortie |
| `out/` | `travel_path.json` | First UAV's current planned route as a waypoint list |
| `out/` | `map_state.json` | Situation map for the victim device (rescue_app v0.5 format) |

Headless snapshots: `java -Xmx2g -Djava.awt.headless=true -cp build c2.app.Snapshot 400 snapshots`
