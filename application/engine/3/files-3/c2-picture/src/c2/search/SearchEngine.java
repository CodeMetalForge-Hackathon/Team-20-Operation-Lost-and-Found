package c2.search;

import c2.model.DroneReport;
import c2.model.GridPos;
import c2.model.OperationsPicture;
import c2.model.TravelPath;

import java.time.Duration;
import java.time.Instant;
import java.util.Collection;

/**
 * Facade: the one door into the search math. LocalSearchEngine implements it in Java;
 * an adapter to Jeff's GHOST core can implement the same interface.
 */
public interface SearchEngine {
    void applyReport(OperationsPicture picture, DroneReport report);
    void advance(OperationsPicture picture, Duration elapsed);

    /** Plan one drone's route. Targets near avoidNear (other drones' targets) are skipped, so routes spread out. */
    TravelPath planRoute(OperationsPicture picture, String pathId, GridPos launchPoint, Instant departAt,
                         Collection<GridPos> avoidNear);
}
