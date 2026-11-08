package c2.model;

import c2.observe.PictureEvent;
import c2.observe.PictureEventType;
import c2.observe.PictureObserver;

import java.time.Instant;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * The digital twin (AirCom's GameBoard role). Holds terrain, belief, objects, paths, alerts.
 * Observer subject: changes are collected during an update and sent to observers after it.
 *
 * Threading: writers call update(...); readers (views) wrap reads in synchronized (picture).
 */
public class OperationsPicture {
    private static final int ALERTS_KEPT = 8;

    private final GeoFrame frame;
    private final TerrainGrid terrain;
    private final BeliefGrid belief;
    private final Map<String, MapObject> objects = new LinkedHashMap<>();
    private final List<TravelPath> flownTracks = new ArrayList<>();
    private final Deque<Alert> alerts = new ArrayDeque<>();
    private final List<PictureObserver> observers = new CopyOnWriteArrayList<>();
    private final List<PictureEvent> pendingEvents = new ArrayList<>();

    private TravelPath plannedPath;
    private Instant now;
    private String phaseName = "-";
    private String sortieStatus = "-";

    public OperationsPicture(GeoFrame frame, TerrainGrid terrain, Instant start) {
        this.frame = frame;
        this.terrain = terrain;
        this.belief = new BeliefGrid(frame.rows(), frame.cols());
        this.now = start;
    }

    public void addObserver(PictureObserver observer) { observers.add(observer); }

    /** Run a block of changes under the lock, then notify observers of everything that changed. */
    public void update(Runnable changes) {
        List<PictureEvent> toSend;
        synchronized (this) {
            changes.run();
            toSend = new ArrayList<>(pendingEvents);
            pendingEvents.clear();
        }
        for (PictureEvent event : toSend) {
            for (PictureObserver observer : observers) observer.onPictureChanged(event);
        }
    }

    /** Record a change; sent when the current update finishes. */
    public void changed(PictureEventType type, String detail) { pendingEvents.add(new PictureEvent(type, detail)); }

    public void add(MapObject object) { objects.put(object.id(), object); }
    public MapObject object(String id) { return objects.get(id); }
    public Collection<MapObject> objects() { return objects.values(); }

    public void setPlannedPath(TravelPath path) {
        plannedPath = path;
        changed(PictureEventType.PATH_PLANNED, path == null ? "cleared" : path.id());
    }
    public void addFlownTrack(TravelPath track) { flownTracks.add(track); }

    public void raise(Alert alert) {
        alerts.addFirst(alert);
        while (alerts.size() > ALERTS_KEPT) alerts.removeLast();
        changed(PictureEventType.ALERT_RAISED, alert.text());
    }

    public void setNow(Instant time) { now = time; }
    public void setPhaseName(String name) {
        phaseName = name;
        changed(PictureEventType.PHASE_CHANGED, name);
    }
    public void setSortieStatus(String status) { sortieStatus = status; }

    public GeoFrame frame() { return frame; }
    public TerrainGrid terrain() { return terrain; }
    public BeliefGrid belief() { return belief; }
    public TravelPath plannedPath() { return plannedPath; }
    public List<TravelPath> flownTracks() { return flownTracks; }
    public Collection<Alert> alerts() { return alerts; }
    public Instant now() { return now; }
    public String phaseName() { return phaseName; }
    public String sortieStatus() { return sortieStatus; }
}
