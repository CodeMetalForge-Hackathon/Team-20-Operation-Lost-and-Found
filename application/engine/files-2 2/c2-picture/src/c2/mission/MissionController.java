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
import c2.model.OperationsPicture;
import c2.model.Source;
import c2.model.TravelPath;
import c2.observe.PictureEventType;
import c2.search.SearchEngine;
import c2.time.TickListener;

import java.time.Duration;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;

/**
 * State pattern context and clock listener. Each tick: apply what arrived, let time act on the
 * picture (belief spreads, confirmations age), then let the current phase run.
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
    private final String subjectId, droneId, tocId;
    private final String intent;

    private MissionPhase phase;
    private Sortie currentSortie;
    private int sortieCount = 0;

    public MissionController(OperationsPicture picture, SearchEngine engine, InboundQueue queue,
                             SimulatedDroneFeed simulatedDrone, MissionOrderWriter orderWriter,
                             PathExporter pathExporter, MapStateWriter mapStateWriter,
                             String subjectId, String droneId, String tocId, String intent) {
        this.picture = picture;
        this.engine = engine;
        this.queue = queue;
        this.simulatedDrone = simulatedDrone;
        this.orderWriter = orderWriter;
        this.pathExporter = pathExporter;
        this.mapStateWriter = mapStateWriter;
        this.subjectId = subjectId;
        this.droneId = droneId;
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

    void launchNewSortie() {
        sortieCount++;
        String sortieId = "S" + sortieCount;
        GridPos launchPoint = picture.object(tocId).pos();
        TravelPath route = engine.planRoute(picture, sortieId, launchPoint, picture.now());
        currentSortie = new Sortie(sortieId, route);
        picture.setPlannedPath(route);
        orderWriter.write(sortieId, route, route.end().plus(Sortie.OVERDUE_GRACE), intent);
        pathExporter.export(route);
        mapStateWriter.write(picture, subjectId);
        picture.raise(new Alert(picture.now(), AlertLevel.INFO, sortieId + " launched: "
                + (route.waypoints().size() - 2) + " targets, back ~" + CLOCK.format(route.end())));
    }

    void predictDroneAlongRoute(Instant now) {
        MapObject drone = picture.object(droneId);
        drone.moveTo(currentSortie.route().positionAt(now));
        drone.revertToHolo();
        picture.setSortieStatus(currentSortie.id() + " out, due " + CLOCK.format(currentSortie.expectedReturn()));
    }

    void sortieLanded(Instant now) {
        currentSortie.markLanded();
        MapObject drone = picture.object(droneId);
        drone.confirm(picture.object(tocId).pos(), now, Source.ANCHOR);
        picture.changed(PictureEventType.OBJECT_CONFIRMED, droneId);
        picture.setSortieStatus(currentSortie.id() + " landed, awaiting report");
        if (simulatedDrone != null) simulatedDrone.sortieLanded(currentSortie.id(), currentSortie.route(), now);
    }

    void raiseOverdue(Sortie sortie) {
        picture.raise(new Alert(picture.now(), AlertLevel.OVERDUE, sortie.id() + " OVERDUE, no report"));
    }

    Sortie currentSortie() { return currentSortie; }
    OperationsPicture picture() { return picture; }

    // ---- internals ----------------------------------------------------------

    private void ingest(DroneReport report) {
        ReportIngest.apply(picture, engine, report, subjectId);
        if (currentSortie != null && currentSortie.id().equals(report.sortieId())) {
            currentSortie.markReportReceived();
            picture.addFlownTrack(currentSortie.route().asFlown());
            picture.setPlannedPath(null);
            picture.setSortieStatus(currentSortie.id() + " report ingested");
        }
        mapStateWriter.write(picture, subjectId);
    }

    /** SOLID moving objects fade to HOLO after SOLID_LIFETIME; HOLO subject follows the belief peak. */
    private void ageConfirmations(Instant now, Duration elapsed) {
        double minutes = elapsed.toMillis() / 60000.0;
        for (MapObject object : picture.objects()) {
            if (!object.kind().moves || object.id().equals(droneId)) continue;
            if (object.knowledge() == Knowledge.SOLID) {
                Duration age = object.ageAt(now);
                if (age != null && age.compareTo(SOLID_LIFETIME) > 0) {
                    object.revertToHolo();
                    picture.changed(PictureEventType.OBJECT_AGED, object.id());
                }
            } else {
                object.growUncertainty(minutes * 0.15, 30);
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
