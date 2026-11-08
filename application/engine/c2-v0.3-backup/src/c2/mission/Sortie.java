package c2.mission;

import c2.model.TravelPath;

import java.time.Duration;
import java.time.Instant;

/** One drone trip: which drone, its route, when it should be back, and whether its report has arrived. */
public class Sortie {
    public static final Duration OVERDUE_GRACE = Duration.ofMinutes(5);
    public static final Duration LOST_AFTER = Duration.ofMinutes(15);

    private final String id;
    private final String droneId;
    private final TravelPath route;
    private boolean landed;
    private boolean reportReceived;
    private boolean overdueRaised;
    private boolean lost;

    public Sortie(String id, String droneId, TravelPath route) {
        this.id = id;
        this.droneId = droneId;
        this.route = route;
    }

    public String id() { return id; }
    public String droneId() { return droneId; }
    public TravelPath route() { return route; }
    public Instant expectedReturn() { return route.end(); }
    public boolean isLanded() { return landed; }
    public boolean isReportReceived() { return reportReceived; }
    public boolean isLost() { return lost; }
    public boolean isFinished() { return reportReceived || lost; }
    public void markLanded() { landed = true; }
    public void markReportReceived() { reportReceived = true; }

    /** True exactly once, when the sortie passes its return time plus grace with no report. */
    public boolean becameOverdue(Instant now) {
        if (reportReceived || overdueRaised) return false;
        if (now.isAfter(expectedReturn().plus(OVERDUE_GRACE))) {
            overdueRaised = true;
            return true;
        }
        return false;
    }

    /** True exactly once, when an overdue sortie is written off so the mission can continue. */
    public boolean becameLost(Instant now) {
        if (reportReceived || lost) return false;
        if (now.isAfter(expectedReturn().plus(LOST_AFTER))) {
            lost = true;
            return true;
        }
        return false;
    }
}
