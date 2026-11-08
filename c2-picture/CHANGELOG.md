# C2 operations picture: iteration log

## v0.2
- **3 UAVs** fly each wave in parallel. The planner spreads them out: each drone skips targets the earlier drones already took. Launches are staggered one minute apart.
- **Larger enemy presence:** an ENEMY AO zone (pulsing red area), six hostiles (command post, observation post, squad, vehicle, possible sniper, a patrol moving along a predicted loop). The planner routes around enemy areas and keeps stand-off from known hostiles.
- **Hostiles confirmed by reports:** a drone that passes within stand-off range reports hostiles it saw; the nearest predicted hostile snaps HOLO -> SOLID.
- **Terrain:** deeper, satellite-like colors, hillshading, per-square color variation, trees on forest (conifers on heavy forest, broadleaf on light), speckle on rock, ripples on water, grass tufts. Seam fix removes the dark striping.
- **Heat map:** stronger color ramp (amber -> red -> magenta -> white at the peak), a clean contour at 50% of the peak, and a pulsing ring on the most likely square.
- **4th zoom level** (40 px tiles).
- **Dashboard redesign:** gradient glass cards, large clock, phase card, stat tiles (belief peak, entropy, sorties flown, contacts), per-UAV sortie cards with progress bars and status chips, two-column tracked-object list, alert cards with color bars, modern fonts (Helvetica Neue / SF Mono when available).
- Sorties that stay overdue 15 minutes are written off as LOST so the mission can continue.

## v0.1
- First end-to-end build: clock, digital twin, observer bridge to Swing, State-pattern mission loop, drone report inbox adapter, simulated drone, mission order / travel path / map state writers, local search engine, isometric board with HOLO/SOLID styles.
