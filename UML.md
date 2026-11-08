# Operation Lost and Found — UML Class Design (draft)

Built from the brainstorm: denied environment, motherships on corridors, dumb sensor nodes on tethers, nature-as-sensor anomaly detection, rescuers as calibration, Bayesian heat map, digital twin for the response team.

Diagrams render on GitHub. Notation:

| Arrow | Meaning | Read it as |
|---|---|---|
| `<\|--` | Inheritance | "is a kind of" |
| `..\|>` | Implements an interface | "can play the role of" |
| `*--` | Composition | "owns, and it dies with me" |
| `o--` | Aggregation | "holds, but it can exist without me" |
| `-->` | Association | "uses / knows about" |
| `..>` | Dependency | "reads or produces, briefly" |

`+` public, `-` private, `#` protected (visible to subclasses).

---

## 1. Overview — who talks to whom

Members left out so the shape is visible. Sections 2–6 fill in each part.

```mermaid
classDiagram
    direction LR
    Mission --> Environment : searches
    Mission --> Mothership : deploys
    Mothership "1" *-- "1" CentralStation : hosts
    Mothership "1" o-- "0..*" SensorNode : carries and releases
    SensorNode "1" *-- "1..*" Sensor : carries
    SensorNode "1" --> "1" DataLink : streams over
    DataLink --> CentralStation : delivers SensorReading
    CentralStation --> ProbabilityGrid : updates
    CentralStation --> Library : stores and reads
    CentralStation --> DecisionEngine : feeds situation
    DecisionEngine --> PathPlanner : may replan
    PathPlanner --> ProbabilityGrid : reads
    Environment "1" *-- "1" ProbabilityGrid : holds
    DigitalTwin --> Environment : shows
    DigitalTwin --> CentralStation : subscribes to
    DigitalTwin --> ResponseTeam : alerts
```

---

## 2. Environment — the world being searched

```mermaid
classDiagram
    class Environment {
        -GridCell[][][] cells
        -double cellSizeMeters
        -DeniedZone deniedZone
        -List~Corridor~ corridors
        -List~Hazard~ hazards
        -LastKnownLocation lastKnown
        -ProbabilityGrid probabilityGrid
        +cellAt(Position spot) GridCell
        +neighborsOf(GridCell cell) List~GridCell~
        +isDenied(Position spot) boolean
    }
    class GridCell {
        -int column
        -int row
        -int layer
        -TerrainType terrain
        -boolean searched
        +center() Position
        +travelCost() double
    }
    class Position {
        +double xMeters
        +double yMeters
        +double zMeters
        +distanceTo(Position other) double
    }
    class TerrainType {
        <<enumeration>>
        OPEN
        FOREST
        ROAD
        TRAIL
        WATER
        STRUCTURE
    }
    class DeniedZone {
        -List~Position~ boundary
        +contains(Position spot) boolean
    }
    class Corridor {
        -CorridorType type
        -List~Position~ route
        +nearestPoint(Position spot) Position
    }
    class CorridorType {
        <<enumeration>>
        POWER_LINE_CLEARING
        ROAD
        PATH
        WATERWAY
    }
    class Hazard {
        -HazardType type
        -Position location
        -double severity
        +pathWeight() double
    }
    class HazardType {
        <<enumeration>>
        CLIFF
        RAVINE
        STEEP_SLOPE
        OBSTACLE
        WATER
    }
    class LastKnownLocation {
        -Position spot
        -Instant time
        -double headingDegrees
    }

    Environment "1" *-- "many" GridCell
    Environment "1" *-- "0..1" DeniedZone
    Environment "1" *-- "0..*" Corridor
    Environment "1" *-- "0..*" Hazard
    Environment "1" *-- "1" LastKnownLocation
    GridCell --> TerrainType
    Corridor --> CorridorType
    Hazard --> HazardType
    Hazard --> Position
    LastKnownLocation --> Position
```

---

## 3. Platforms, nodes, sensors, links — the physical things

`BayStation` is modeled as a **role** (interface) the `Mothership` plays, not a separate object. That matches "the mothership has three responsibilities."

```mermaid
classDiagram
    class Platform {
        <<abstract>>
        #String platformId
        #Position position
        #double batteryPercent
        +position() Position
        +isAlive() boolean
    }
    class Mothership {
        -Corridor assignedCorridor
        -List~SensorNode~ bay
        -CentralStation station
        +transitToZone(Corridor route) void
        +returnHome() void
        +consolidate() void
    }
    class BayStation {
        <<interface>>
        +releaseNode(SensorNode node, Position target) void
        +recoverNode(SensorNode node) void
        +nodesAvailable() int
    }
    class AirMothership
    class GroundMothership

    class SensorNode {
        <<abstract>>
        #String nodeId
        #List~Sensor~ sensors
        #DataLink link
        +sampleAll() List~SensorReading~
        +sendReadings() void
    }
    class PollenDrone {
        -boolean landed
        +flyTo(Position target) void
        +land() void
    }
    class ListenerNode {
        -Position fixedSpot
    }

    class Sensor {
        <<abstract>>
        #String sensorId
        #double maxRangeMeters
        +read() SensorReading
    }
    class AcousticSensor {
        -int sampleRateHz
        -double[] bandEdgesHz
    }
    class ThermalSensor {
        -int pixelColumns
        -int pixelRows
    }
    class IRSensor
    class VisionSensor
    class VibrationSensor
    class RadioSensor {
        -int nearbyDeviceCount
    }

    class DataLink {
        <<abstract>>
        +send(SensorReading reading) boolean
        +isUp() boolean
    }
    class FiberTether {
        -double lengthMeters
        -double lightLevel
        +isSnagged() boolean
    }
    class WiFiLink
    class StoreAndForwardLog {
        -String logFilePath
        +flush() void
    }

    Platform <|-- Mothership
    Platform <|-- SensorNode
    Mothership <|-- AirMothership
    Mothership <|-- GroundMothership
    Mothership ..|> BayStation
    SensorNode <|-- PollenDrone
    SensorNode <|-- ListenerNode
    Sensor <|-- AcousticSensor
    Sensor <|-- ThermalSensor
    Sensor <|-- IRSensor
    Sensor <|-- VisionSensor
    Sensor <|-- VibrationSensor
    Sensor <|-- RadioSensor
    DataLink <|-- FiberTether
    DataLink <|-- WiFiLink
    DataLink <|-- StoreAndForwardLog
    SensorNode "1" *-- "1..*" Sensor
    SensorNode "1" --> "1" DataLink
    Mothership "1" o-- "0..*" SensorNode
```

**Real-world mapping this weekend:** a Pi Zero listener = `ListenerNode` + `AcousticSensor` + `StoreAndForwardLog` (logs to SD, collected later). A fiber-tethered micro drone = `PollenDrone` + sensors + `FiberTether`.

---

## 4. Data and evidence — what flows

The key contract: **every `Evidence` answers one question** — *"how likely would I see this if the person were in this cell?"* That single method is what lets any source feed the Bayesian map.

```mermaid
classDiagram
    class SensorReading {
        +String nodeId
        +String sensorId
        +Instant time
        +Position nodePosition
        +Map~String,Double~ values
    }
    class Baseline {
        -String nodeId
        -Map~String,RunningStats~ normalByChannel
        -int samplesSeen
        +update(SensorReading reading) void
        +isTrustworthy() boolean
        +deviation(SensorReading reading) double
    }
    class RunningStats {
        -double mean
        -double spread
        -int count
        +add(double value) void
    }
    class Anomaly {
        +String nodeId
        +Instant time
        +Position where
        +double strength
        +String channel
    }
    class KnownActor {
        -String actorId
        -ActorType type
        -Track track
        +positionAt(Instant time) Position
    }
    class ActorType {
        <<enumeration>>
        FRIENDLY
        ENEMY
        OTHER_HUMAN
        VEHICLE
    }
    class Track {
        -List~TimedPosition~ points
        +add(TimedPosition point) void
        +nearest(Instant time) TimedPosition
    }
    class TimedPosition {
        +Position spot
        +Instant time
    }
    class HumanSignature {
        -Map~String,Double~ typicalResponseByChannel
        -int labeledExamples
        +learnFrom(Anomaly anomaly, KnownActor cause) void
        +matchScore(Anomaly anomaly) double
    }
    class Cue {
        +Anomaly source
        +double signatureMatch
        +Position estimatedSpot
    }

    class Evidence {
        <<abstract>>
        #Instant time
        +likelihood(GridCell cell) double
    }
    class SearchResult {
        -Position searcherSpot
        -boolean found
        -double detectionChance
    }
    class CueEvidence {
        -Cue cue
    }
    class SightingReport {
        -Position reportedSpot
        -double reporterReliability
    }

    class Library {
        -Map~String,Baseline~ baselinesByPlace
        -List~HumanSignature~ signatures
        +saveDeployment(String areaName) void
        +loadFor(String areaName) void
    }

    Baseline "1" *-- "1..*" RunningStats
    KnownActor --> ActorType
    KnownActor "1" *-- "1" Track
    Track "1" *-- "0..*" TimedPosition
    Cue --> Anomaly
    Evidence <|-- SearchResult
    Evidence <|-- CueEvidence
    Evidence <|-- SightingReport
    CueEvidence --> Cue
    Library o-- Baseline
    Library o-- HumanSignature
```

---

## 5. Processing — the central station's brain

Dumb nodes, smart center: every computation lives here.

```mermaid
classDiagram
    class CentralStation {
        -Map~String,Baseline~ baselines
        -List~KnownActor~ knownActors
        -BaselineBuilder baselineBuilder
        -AnomalyDetector anomalyDetector
        -KnownActorFilter knownActorFilter
        -SignatureMatcher signatureMatcher
        -CorrelationEngine correlationEngine
        -BayesianUpdater bayesianUpdater
        +receive(SensorReading reading) void
        +snapshot() SituationSnapshot
    }
    class BaselineBuilder {
        +update(Baseline baseline, SensorReading reading) void
    }
    class AnomalyDetector {
        -double deviationThreshold
        +check(SensorReading reading, Baseline baseline) Optional~Anomaly~
    }
    class KnownActorFilter {
        -double explainRadiusMeters
        +explainedBy(Anomaly anomaly, List~KnownActor~ actors) Optional~KnownActor~
    }
    class SignatureMatcher {
        -HumanSignature signature
        +train(Anomaly anomaly, KnownActor cause) void
        +toCue(Anomaly anomaly) Cue
    }
    class CorrelationEngine {
        -List~Anomaly~ recent
        +add(Anomaly anomaly) void
        +movingDisturbances() List~Track~
    }
    class ProbabilityGrid {
        -double[][] chanceByCell
        +chanceAt(GridCell cell) double
        +hottestCells(int count) List~GridCell~
        +entropy() double
        +normalize() void
    }
    class BayesianUpdater {
        +apply(Evidence evidence, ProbabilityGrid grid) void
        +spreadOverTime(ProbabilityGrid grid, MotionModel walker, Duration elapsed) void
    }
    class MotionModel {
        <<interface>>
        +moveChance(GridCell from, GridCell to, Duration elapsed) double
    }
    class LostPersonMotion {
        -double walkingSpeed
        -boolean prefersDownhill
        -boolean prefersTrails
    }
    class SituationSnapshot {
        +ProbabilityGrid grid
        +List~Cue~ recentCues
        +List~Hazard~ hazards
        +List~Platform~ assets
    }
    class Trigger {
        <<interface>>
        +isMet(SituationSnapshot situation) boolean
        +fire(DecisionEngine engine) void
    }
    class CueNearCorridorTrigger
    class NodeLostTrigger
    class ConvergenceTrigger
    class DecisionEngine {
        -List~Trigger~ armedTriggers
        +evaluate(SituationSnapshot situation) void
    }
    class PathPlanner {
        +nextWaypoint(ProbabilityGrid grid, Environment world, Platform searcher) GridCell
    }

    CentralStation *-- BaselineBuilder
    CentralStation *-- AnomalyDetector
    CentralStation *-- KnownActorFilter
    CentralStation *-- SignatureMatcher
    CentralStation *-- CorrelationEngine
    CentralStation *-- BayesianUpdater
    CentralStation ..> SituationSnapshot : produces
    BayesianUpdater ..> ProbabilityGrid : updates
    BayesianUpdater --> MotionModel
    MotionModel <|.. LostPersonMotion
    Trigger <|.. CueNearCorridorTrigger
    Trigger <|.. NodeLostTrigger
    Trigger <|.. ConvergenceTrigger
    DecisionEngine o-- Trigger
    DecisionEngine ..> SituationSnapshot : reads
    DecisionEngine --> PathPlanner
```

**Flow of one reading:**
`SensorReading` → `BaselineBuilder` (learn normal) → `AnomalyDetector` (odd?) → `KnownActorFilter` (explained by us?) — if yes → `SignatureMatcher.train` (calibration); if no → `SignatureMatcher.toCue` → `CueEvidence` → `BayesianUpdater.apply` → `ProbabilityGrid` → `DecisionEngine` checks triggers.

---

## 6. Command picture and mission — the humans and the loop

```mermaid
classDiagram
    class DigitalTwin {
        -Environment world
        -List~MapLayer~ layers
        +refresh(SituationSnapshot situation) void
        +render() void
    }
    class MapLayer {
        <<abstract>>
        #boolean visible
        +draw(SituationSnapshot situation) void
    }
    class TerrainLayer
    class HeatMapLayer
    class AssetLayer
    class TetherLayer
    class CueLayer
    class KnownActorLayer
    class Alert {
        +Cue cause
        +Position where
        +AlertLevel level
        +Instant time
    }
    class AlertLevel {
        <<enumeration>>
        INFO
        INVESTIGATE
        CONTACT
    }
    class ResponseTeam {
        -String callSign
        -List~KnownActor~ members
        +receive(Alert alert) void
    }
    class Mission {
        -MissionPhase phase
        -Environment world
        -List~Mothership~ motherships
        -CentralStation station
        -DigitalTwin twin
        +advancePhase() void
    }
    class MissionPhase {
        <<enumeration>>
        INFO
        INIT
        EXECUTE
        REGROUP
    }

    DigitalTwin o-- MapLayer
    MapLayer <|-- TerrainLayer
    MapLayer <|-- HeatMapLayer
    MapLayer <|-- AssetLayer
    MapLayer <|-- TetherLayer
    MapLayer <|-- CueLayer
    MapLayer <|-- KnownActorLayer
    DigitalTwin ..> Alert : raises
    Alert --> AlertLevel
    ResponseTeam ..> Alert : receives
    Mission --> MissionPhase
    Mission --> DigitalTwin
```

---

## Design notes and open questions

1. **`Evidence.likelihood(cell)` is the central contract.** Any new source (a drone sweep, a listener cue, a human report) plugs into the map by implementing it. The hard one is `CueEvidence`, whose number comes from the `HumanSignature` calibration.
2. **Rescuers are double-use data.** `KnownActorFilter` both removes our own disturbance and hands labeled examples to `SignatureMatcher.train`.
3. **`BayStation` is a role, not a thing.** Change it to a class only if bay stations can exist apart from a mothership.
4. **Repeated readings must not count as fresh evidence.** `BayesianUpdater` needs a rule for correlated evidence (same sensor, same spot, short interval). The GHOST stub demo converged ~10 m off the subject partly for this reason.
5. **Triggers, not schedules.** `DecisionEngine` holds armed `Trigger`s; the environment's state decides when they fire (core idea 2).

### Mapping to Jeff's GHOST code

| This design | GHOST |
|---|---|
| `ProbabilityGrid` | `belief_map.rs` |
| `Evidence` / `likelihood` | `sensor_model.rs` (Pd / Pfa likelihood ratios) |
| `BayesianUpdater` | update + normalize inside the core |
| `PathPlanner` | `path_planner.rs` |
| `Hazard` | `hazard.rs` |
| `DigitalTwin` | `dashboard/index.html` |
| `Sensor` subclasses | `sensors/*.py` |
| Not yet in GHOST | `Baseline`, `AnomalyDetector`, `KnownActorFilter`, `HumanSignature`, `CorrelationEngine`, `MotionModel`, `DecisionEngine`, `ListenerNode` |
