package c2.search;

import c2.model.BeliefGrid;
import c2.model.DroneReport;
import c2.model.GridPos;
import c2.model.MapObject;
import c2.model.ObjectKind;
import c2.model.OperationsPicture;
import c2.model.TerrainGrid;
import c2.model.TravelPath;
import c2.model.Waypoint;
import c2.model.ZoneType;
import c2.observe.PictureEventType;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/** Java search engine: belief updates, spreading over time, greedy information-gain routes. */
public class LocalSearchEngine implements SearchEngine {
    private static final double SWEEP_COVERAGE = 0.35;          // one pass sees ~35% of a 50 m square
    private static final double MOVING_FRACTION_PER_MINUTE = 0.015;
    private static final int TARGETS_PER_ROUTE = 6;
    private static final double MIN_TARGET_SPACING = 6;
    private static final double DRONE_CELLS_PER_SECOND = 0.2;  // 10 m/s over 50 m squares
    private static final double HOSTILE_STANDOFF_CELLS = 6;

    @Override
    public void applyReport(OperationsPicture picture, DroneReport report) {
        BeliefGrid belief = picture.belief();
        TerrainGrid terrain = picture.terrain();
        if (report.sighting() != null) {
            belief.applySighting(report.sighting(), 2.0);
        } else {
            belief.applyMiss(report.sweptCells(), terrain, SWEEP_COVERAGE);
        }
        picture.changed(PictureEventType.BELIEF_UPDATED, "report " + report.sortieId());
    }

    @Override
    public void advance(OperationsPicture picture, Duration elapsed) {
        double minutes = elapsed.toMillis() / 60000.0;
        double fraction = Math.min(0.5, minutes * MOVING_FRACTION_PER_MINUTE);
        if (fraction <= 0) return;
        picture.belief().spread(picture.terrain(), fraction);
        picture.changed(PictureEventType.BELIEF_UPDATED, "spread");
    }

    @Override
    public TravelPath planRoute(OperationsPicture picture, String pathId, GridPos launchPoint, Instant departAt,
                                Collection<GridPos> avoidNear) {
        BeliefGrid belief = picture.belief();
        TerrainGrid terrain = picture.terrain();
        List<GridPos> hostiles = picture.objects().stream()
                .filter(object -> object.kind() == ObjectKind.HOSTILE).map(MapObject::pos).toList();

        // 1. Pick high-value, well-spaced targets: value = chance x how well the sensor sees there.
        List<GridPos> targets = new ArrayList<>();
        List<double[]> scored = new ArrayList<>();
        for (int row = 0; row < belief.rows(); row++) {
            for (int col = 0; col < belief.cols(); col++) {
                GridPos cell = new GridPos(row, col);
                double value = belief.at(row, col) * terrain.at(row, col).cover().sensorFactor * risk(picture, cell, hostiles);
                if (value > 0) scored.add(new double[]{value, row, col});
            }
        }
        scored.sort((a, b) -> Double.compare(b[0], a[0]));
        for (double[] candidate : scored) {
            GridPos pos = new GridPos((int) candidate[1], (int) candidate[2]);
            boolean farEnough = targets.stream().allMatch(chosen -> chosen.distanceTo(pos) >= MIN_TARGET_SPACING)
                    && avoidNear.stream().allMatch(taken -> taken.distanceTo(pos) >= MIN_TARGET_SPACING);
            if (farEnough) targets.add(pos);
            if (targets.size() == TARGETS_PER_ROUTE) break;
        }

        // 2. Order them nearest-first from the launch point, then return home.
        List<GridPos> ordered = new ArrayList<>();
        GridPos current = launchPoint;
        List<GridPos> remaining = new ArrayList<>(targets);
        while (!remaining.isEmpty()) {
            GridPos from = current;
            GridPos nearest = remaining.stream()
                    .min((a, b) -> Double.compare(from.distanceTo(a), from.distanceTo(b))).get();
            ordered.add(nearest);
            remaining.remove(nearest);
            current = nearest;
        }

        // 3. Times along the way.
        List<Waypoint> waypoints = new ArrayList<>();
        Instant time = departAt;
        GridPos previous = launchPoint;
        waypoints.add(new Waypoint(launchPoint, time));
        ordered.add(launchPoint);
        for (GridPos pos : ordered) {
            long seconds = (long) (previous.distanceTo(pos) / DRONE_CELLS_PER_SECOND);
            time = time.plusSeconds(seconds);
            waypoints.add(new Waypoint(pos, time));
            previous = pos;
        }
        return new TravelPath(pathId, waypoints, c2.model.Knowledge.HOLO);
    }

    /** Lower value inside enemy areas and near known hostiles, so drones aren't routed into them. */
    private static double risk(OperationsPicture picture, GridPos cell, List<GridPos> hostiles) {
        double factor = picture.insideZone(cell, ZoneType.ENEMY_AO) ? 0.15 : 1.0;
        for (GridPos hostile : hostiles) {
            if (hostile.distanceTo(cell) < HOSTILE_STANDOFF_CELLS) factor *= 0.3;
        }
        return factor;
    }
}
