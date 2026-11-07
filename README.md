![Brainstorm whiteboard — CH-1 draft](brainstorm_1.png)
*Brainstorm whiteboard, CH-1 (draft).*


NOTE: Check point 1, 60 minutes, limits we expect to run in to, ASK outlves to either keep going or reduce scope to key product we've already identified 

# Operation Lost and Found — Code Metal Forge 2026

**SITREP — CH-1** | Sat 07 Nov 2026 | BASE X, Boston
**Status:** DRAFT — built from the brainstorm whiteboard; nothing below is a locked decision.

![Brainstorm whiteboard — CH-1 draft](brainstorm_1.png)
*Brainstorm whiteboard, CH-1 (draft).*

---

## 1. Situation

Search-and-rescue challenge: find a missing person when GPS, connectivity, and critical infrastructure are unavailable.

Working assumptions from the board:

- **GPS: DENIED.** The vehicle cannot rely on satellite position. It must estimate where it is from onboard sensors.
- **Comms: DENIED / degraded.** The vehicle may not get commands mid-mission. It needs to carry its plan and act on it without a link.
- **Limits:** weight, power, cost, connectivity.
- **Key bottleneck:** sensor fusion vs. power. More sensors and more onboard processing find the person faster but drain the battery faster.

## 2. Mission — Task 1

1. **Search algorithm premise** — rapidly generate a movement plan (where the drone goes, in what order).
2. **Hardware check** — confirm what is physically available to us.
3. **Software + driver/controller stack** — define what has to exist between "the plan" and "the motors spin" for autonomous movement.

## 3. Concept of Operations (draft)

### Mission loop

```
INFO  →  INIT  →  EXECUTE  →  REGROUP  →  back to INFO
```

| Phase   | What happens |
|---------|--------------|
| INFO    | Gather what we know: find-the-guy, hazards / life signs, map / conditions |
| INIT    | Build the search plan from that info |
| EXECUTE | Fly the plan, sense, detect |
| REGROUP | Pull results back together, update what we know, replan |

### Search algorithm premise — Bayesian search

**The problem it solves:** we don't know where the person is, but we don't know *nothing* either. Some places are more likely than others. We want to spend limited battery on the most likely places first.

**How it works, in plain terms:**

1. Lay a grid over the search area. Each square holds a number: the chance the person is in that square. All squares together add to 100%.
2. Set the starting numbers from what we know:
   - **LKL** (last known location) — squares near it start higher.
   - **Known hazards** — squares the person likely couldn't reach, or the drone can't enter, are adjusted.
   - **Terrain / orientation** — paths, slopes, direction of travel.
   - **Risk** — weighting for danger to the person or the vehicle.
3. Generate the **initial search path**: order the squares so the drone covers the most probability for the least flight time and energy.
4. While flying: when the drone searches a square and does **not** find the person, that square's number goes down and every other square's number goes up slightly (because the total stays 100%). A miss is still information.
5. On REGROUP, replan from the updated grid.

This is established search theory (it is the basis of how the U.S. Coast Guard plans maritime searches), so it is defensible to judges and has known math behind it.

### Component split (from the board)

| # | Component | Owner |
|---|-----------|-------|
| 1 | Drones that can move through positions | Eric |
| 2 | Algorithm that creates the travel path | Jeff |
| 3 | | Sebastian — supervise, refine, fill in, develop UML and Arch-Sys design |

The handoff between 1 and 2 is the critical interface: **Jeff's algorithm outputs an ordered list of waypoints; Eric's drone layer consumes that list and flies it.** Agreeing on that format early lets both build in parallel.

### Processes the system must perform

Maneuver → Process → Sense → Act / Initiate → Communicate → Consolidate

### Sensing options

- **Media:** CV (camera), IR, acoustic, RF, thermal
- **Onboard data:** IMU, accelerometer, prior knowledge (+ entries not yet transcribed)

## 4. Software + Driver/Controller Stack (draft)

What has to exist, top to bottom. Each layer only talks to the layer directly below it.

| Layer | Job | Runs on | Candidate options (pending hardware check) |
|-------|-----|---------|--------------------------------------------|
| 1. Mission planner | Holds the probability grid, picks next squares | Laptop or companion computer | Python (our code) |
| 2. Path planner | Turns chosen squares into a flyable waypoint sequence, avoids hazards | Laptop or companion computer | Python (our code) |
| 3. Position estimation (GPS-denied) | Answers "where am I?" without GPS | Companion computer + flight controller | Optical flow sensor, visual-inertial odometry (camera + IMU), dead reckoning from IMU |
| 4. Perception | Answers "is that the person?" | Companion computer (Jetson if available) | Pretrained person detector on camera / thermal feed |
| 5. Vehicle interface | Sends "go to this point" to the flight controller, reads back state | Companion computer | MAVLink via MAVSDK or pymavlink (if PX4/ArduPilot); vendor SDK otherwise |
| 6. Flight controller firmware | Keeps the drone stable, turns "go to point" into motor commands | Flight controller board | PX4, ArduPilot, or vendor firmware |
| 7. Motor drivers (ESCs) | Turn commands into motor speed | ESCs | Comes with the airframe |

**Note on layer 3:** without GPS, a "go to waypoint X" command only works if the flight controller has some other position source. This is likely the hardest integration item and directly drives the sensor fusion vs. power trade.

## 5. Hardware Check

Status: **NOT YET VERIFIED ON SITE.** Code Metal's previous hackathon supplied microcontrollers (ESP32, Raspberry Pi Pico), Raspberry Pi, Jetson Orin Nano, SDRs, sensors, and wired cameras. Treat that as a reference only until confirmed for this event.

| Item | Available? | Qty | Role in our stack | Notes |
|------|-----------|-----|-------------------|-------|
| Drone airframe / platform | ☐ | | Vehicle | Model determines firmware + SDK |
| Flight controller | ☐ | | Layer 6 | PX4/ArduPilot-compatible? |
| Companion computer (Pi / Jetson) | ☐ | | Layers 1–5 | Jetson needed for onboard CV? |
| Camera (RGB) | ☐ | | Perception, VIO | |
| Thermal / IR sensor | ☐ | | Perception | |
| Optical flow / rangefinder | ☐ | | GPS-denied positioning | |
| IMU | ☐ | | Positioning | Usually on the flight controller |
| SDR / RF | ☐ | | RF detection, comms | |
| Microcontrollers (ESP32, Pico) | ☐ | | Sensor nodes, relays | |
| Batteries + charger | ☐ | | Power | Flight time per pack? |
| Telemetry radio | ☐ | | Comms when available | |

## 6. Open Items

- Drone platform and firmware — decides layers 5–6.
- GPS-denied position source — decides whether waypoint flying works at all.
- Waypoint list format between path algorithm and drone layer.
- Whiteboard entries not yet transcribed: first DATA item, item below "Acc", "LTT" and the red item after "Risk" in the Bayesian inputs.

---

*Next SITREP: CH-2*
