package c2.mission;

import c2.model.TravelPath;

import java.time.Duration;
import java.time.Instant;

/** One drone trip: its route, when it should be back, and whether its report has arrived. */
public class Sortie {
    public static final Duration OVERDUE_GRACE = Duration.ofMinutes(5);

    private final String id;
    private final TravelPath route;
    private boolean landed;
    private boolean reportReceived;
    private boolean overdueRaised;

    public Sortie(String id, TravelPath route) {
        this.id = id;
        this.route = route;
    }

    public String id() { return id; }
    public TravelPath route() { return route; }
    public Instant expectedReturn() { return route.end(); }
    public boolean isLanded() { return landed; }
    public boolean isReportReceived() { return reportReceived; }
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
}
