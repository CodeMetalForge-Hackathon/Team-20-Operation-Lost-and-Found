package c2.mission;

import c2.export.MapStateWriter;
import c2.export.MissionOrderWriter;
import c2.export.PathExporter;
import c2.ingest.InboundQueue;
import c2.ingest.ReportIngest;
import c2.ingest.SimulatedDroneFeed;
import c2.input.Command;
import c2.input.CommandSink;
import c2.model.Alert;
import c2.model.AlertLevel;
import c2.model.BeliefGrid;
import c2.model.DroneReport;
import c2.model.GridPos;
import c2.model.Knowledge;
import c2.model.MapObject;
import c2.model.ObjectKind;
import c2.model.OperationsPicture;
import c2.model.PatrolRoute;
import c2.model.SortieView;
import c2.model.Source;
import c2.model.TravelPath;
import c2.model.Waypoint;
import c2.observe.PictureEventType;
import c2.search.SearchEngine;
import c2.time.TickListener;

import java.time.Duration;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

/**
 * State pattern context and clock listener. Each tick: apply what arrived, let time act on the
 * picture (belief spreads, patrols move, confirmations age), then let the current phase run.
 */
public class MissionController implements TickListener, CommandSink {
    private static final Duration SOLID_LIFETIME = Duration.ofMinutes(15);
    private static final DateTimeFormatter CLOCK = DateTimeFormatter.ofPattern("HH:mm").withZone(ZoneOffset.UTC);

    private final OperationsPicture picture;
    private final SearchEngine engine;
    private final InboundQueue queue;
    private final SimulatedDroneFeed simulatedDrone;     // null when using real reports only
    private final MissionOrderWriter orderWriter;
    private final PathExporter pathExporter;
    private final MapStateWriter mapStateWriter;
    private final String subjectId, tocId;
    private final List<String> droneIds;
    private final String intent;

    private MissionPhase phase;
    private final List<Sortie> activeSorties = new ArrayList<>();
    private int waveCount = 0;

    public MissionController(OperationsPicture picture, SearchEngine engine, InboundQueue queue,
                             SimulatedDroneFeed simulatedDrone, MissionOrderWriter orderWriter,
                             PathExporter pathExporter, MapStateWriter mapStateWriter,
                             String subjectId, List<String> droneIds, String tocId, String intent) {
        this.picture = picture;
        this.engine = engine;
        this.queue = queue;
        this.simulatedDrone = simulatedDrone;
        this.orderWriter = orderWriter;
        this.pathExporter = pathExporter;
        this.mapStateWriter = mapStateWriter;
        this.subjectId = subjectId;
        this.droneIds = droneIds;
        this.tocId = tocId;
        this.intent = intent;
    }

    /** Begin the mission loop. */
    public void begin() {
        picture.update(() -> transitionTo(new InitPhase()));
    }

    @Override
    public void onTick(Instant now, Duration elapsed) {
        picture.update(() -> {
            picture.setNow(now);
            for (Object item : queue.drain()) {
                if (item instanceof DroneReport report) ingest(report);
            }
            engine.advance(picture, elapsed);
            if (simulatedDrone != null) simulatedDrone.wander(elapsed.toMillis() / 60000.0);
            movePatrols(now);
            ageConfirmations(now, elapsed);
            MissionPhase next = phase.onTick(this, now, elapsed);
            if (next != phase) transitionTo(next);
            picture.changed(PictureEventType.TICK, null);
        });
    }

    @Override
    public void submit(Command command) {
        picture.update(() -> {
            MissionPhase next = phase.onCommand(this, command);
            if (next != phase) transitionTo(next);
        });
    }

    // ---- called by phases -------------------------------------------------

    /** One wave: every drone gets its own route; later drones avoid earlier drones' targets. */
    void launchWave() {
        waveCount++;
        activeSorties.clear();
        GridPos launchPoint = picture.object(tocId).pos();
        List<GridPos> takenTargets = new ArrayList<>();
        for (int index = 0; index < droneIds.size(); index++) {
            String sortieId = "W" + waveCount + "-" + (char) ('A' + index);
            Instant depart = picture.now().plusSeconds(index * 60L);          // staggered one minute apart
            TravelPath route = engine.planRoute(picture, sortieId, launchPoint, depart, takenTargets);
            for (Waypoint waypoint : route.waypoints()) takenTargets.add(waypoint.pos());
            Sortie sortie = new Sortie(sortieId, droneIds.get(index), route);
            activeSorties.add(sortie);
            picture.putPlannedPath(route);
            orderWriter.write(sortieId, route, route.end().plus(Sortie.OVERDUE_GRACE), intent);
            if (index == 0) pathExporter.export(route);
        }
        mapStateWriter.write(picture, subjectId);
        picture.raise(new Alert(picture.now(), AlertLevel.INFO, "Wave " + waveCount + ": "
                + droneIds.size() + " UAVs launched"));
        publishSorties();
    }

    void predictDroneAlongRoute(Sortie sortie, Instant now) {
        MapObject drone = picture.object(sortie.droneId());
        drone.moveTo(sortie.route().positionAt(now));
        if (now.isAfter(sortie.route().start())) drone.revertToHolo();
    }

    void sortieLanded(Sortie sortie, Instant now) {
        sortie.markLanded();
        MapObject drone = picture.object(sortie.droneId());
        drone.confirm(picture.object(tocId).pos(), now, Source.ANCHOR);
        picture.changed(PictureEventType.OBJECT_CONFIRMED, sortie.droneId());
        if (simulatedDrone != null) simulatedDrone.sortieLanded(sortie.id(), sortie.route(), now, hostilePositions());
    }

    void raiseOverdue(Sortie sortie) {
        picture.raise(new Alert(picture.now(), AlertLevel.OVERDUE, sortie.id() + " OVERDUE, no report"));
    }

    void writeOffLost(Sortie sortie) {
        picture.removePlannedPath(sortie.id());
        picture.raise(new Alert(picture.now(), AlertLevel.OVERDUE, sortie.id() + " presumed LOST"));
    }

    void publishSorties() {
        List<SortieView> views = new ArrayList<>();
        for (Sortie sortie : activeSorties) {
            String status = sortie.isReportReceived() ? "REPORTED" : sortie.isLost() ? "LOST"
                    : sortie.isLanded() ? "LANDED" : "OUT";
            views.add(new SortieView(sortie.id(), picture.object(sortie.droneId()).label(),
                    sortie.route().start(), sortie.expectedReturn(), status));
        }
        picture.setSorties(views);
    }

    List<Sortie> activeSorties() { return activeSorties; }
    OperationsPicture picture() { return picture; }

    // ---- internals ----------------------------------------------------------

    private void ingest(DroneReport report) {
        ReportIngest.apply(picture, engine, report, subjectId);
        for (Sortie sortie : activeSorties) {
            if (sortie.id().equals(report.sortieId())) {
                sortie.markReportReceived();
                picture.addFlownTrack(sortie.route().asFlown());
                picture.removePlannedPath(sortie.id());
            }
        }
        mapStateWriter.write(picture, subjectId);
        publishSorties();
    }

    private List<GridPos> hostilePositions() {
        List<GridPos> positions = new ArrayList<>();
        for (MapObject object : picture.objects()) if (object.kind() == ObjectKind.HOSTILE) positions.add(object.pos());
        return positions;
    }

    /** Hostiles with a predicted patrol are moved around their loop as time passes. */
    private void movePatrols(Instant now) {
        for (PatrolRoute patrol : picture.patrols()) {
            MapObject hostile = picture.object(patrol.objectId());
            if (hostile != null && hostile.knowledge() == Knowledge.HOLO) {
                hostile.moveTo(patrol.positionAt(picture.start(), now));
            }
        }
    }

    /** SOLID moving objects fade to HOLO after SOLID_LIFETIME; HOLO subject follows the belief peak. */
    private void ageConfirmations(Instant now, Duration elapsed) {
        double minutes = elapsed.toMillis() / 60000.0;
        for (MapObject object : picture.objects()) {
            if (!object.kind().moves || object.kind() == ObjectKind.DRONE) continue;
            if (object.knowledge() == Knowledge.SOLID) {
                Duration age = object.ageAt(now);
                if (age != null && age.compareTo(SOLID_LIFETIME) > 0) {
                    object.revertToHolo();
                    picture.changed(PictureEventType.OBJECT_AGED, object.id());
                }
            } else {
                object.growUncertainty(minutes * 0.15, object.kind() == ObjectKind.SUBJECT ? 30 : 10);
            }
        }
        MapObject subject = picture.object(subjectId);
        if (subject != null && subject.knowledge() == Knowledge.HOLO) subject.moveTo(beliefPeak());
    }

    private GridPos beliefPeak() {
        BeliefGrid belief = picture.belief();
        int bestRow = 0, bestCol = 0;
        double best = -1;
        for (int row = 0; row < belief.rows(); row++) {
            for (int col = 0; col < belief.cols(); col++) {
                if (belief.at(row, col) > best) { best = belief.at(row, col); bestRow = row; bestCol = col; }
            }
        }
        return new GridPos(bestRow, bestCol);
    }

    private void transitionTo(MissionPhase next) {
        phase = next;
        picture.setPhaseName(next.name());
        next.enter(this);
    }
}
