# Running the C2 operations picture (v0.1)

Requires a Java JDK 17 or newer. No external libraries.

```
cd c2-picture
./run.sh            # demo: simulated drone writes reports into ./inbox
./run.sh --no-sim   # real reports only: drop drone report JSON files into ./inbox
```

Manual build: `javac -d build $(find src -name "*.java") && java -cp build c2.app.Main --assets=assets`

## What you see

- Isometric 5 x 5 mile board (162 x 162 squares, 50 m each), synthetic terrain until `terrain.json` prep exists.
- Red overlay = probability the subject is in each square (a prediction, so translucent).
- Dashed cyan = planned route (HOLO). Faint solid cyan = last flown tracks.
- Objects: blue rectangle TOC, cyan drone, white subject, orange hazards, red hostile. Translucent/dashed = HOLO, filled = SOLID.
- Loop: INIT plans a route -> EXECUTE (drone shown HOLO along its route) -> drone lands, report arrives in `./inbox` -> picked up next tick -> belief updates, contact turns the subject SOLID, urgent casualty alerts -> REGROUP -> replan.
- Confirmed moving objects fade back to HOLO after 15 mission minutes.

## Keys

SPACE pause · +/- sim speed · H halt · R resume · P replan (in REGROUP) · arrows pan · Z/X zoom

## Files in and out

| Folder | File | What |
|---|---|---|
| `inbox/` | `*.json` | Drone reports (format: `docs/sample_drone_report.json`); moved to `inbox/processed/` once read |
| `out/` | `mission_order_S#.json` | Route, rules, intent, return deadline for each sortie |
| `out/` | `travel_path.json` | Current planned route as a waypoint list (rows/cols + meters east/north + ETA) |
| `out/` | `map_state.json` | Situation map for the victim device (rescue_app v0.5 format) |

Headless snapshots: `java -Djava.awt.headless=true -cp build c2.app.Snapshot 400 snapshots`

## Not yet in v0.1

Real terrain import, flat north-up inset, GHOST engine adapter, listener CSV adapter, multiple drones.
