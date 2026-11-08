package c2.model;

import java.time.Duration;
import java.time.Instant;

/** Anything on the board. Identity is the id, which stays the same across mission orders and reports. */
public class MapObject {
    private final String id;
    private final ObjectKind kind;
    private final String label;
    private GridPos pos;
    private Knowledge knowledge;
    private Source source;
    private Instant confirmedAt;
    private double uncertaintyCells;

    public MapObject(String id, ObjectKind kind, String label, GridPos pos,
                     Knowledge knowledge, Source source, Instant confirmedAt, double uncertaintyCells) {
        this.id = id;
        this.kind = kind;
        this.label = label;
        this.pos = pos;
        this.knowledge = knowledge;
        this.source = source;
        this.confirmedAt = confirmedAt;
        this.uncertaintyCells = uncertaintyCells;
    }

    /** A report or anchor confirms this object: becomes SOLID at the reported position. */
    public void confirm(GridPos newPos, Instant when, Source by) {
        pos = newPos;
        knowledge = Knowledge.SOLID;
        source = by;
        confirmedAt = when;
        uncertaintyCells = 1;
    }

    /** Confirmation too old: back to a prediction. */
    public void revertToHolo() {
        knowledge = Knowledge.HOLO;
        source = Source.PREDICTED;
    }

    public void growUncertainty(double cells, double max) {
        uncertaintyCells = Math.min(max, uncertaintyCells + cells);
    }

    public Duration ageAt(Instant now) {
        return confirmedAt == null ? null : Duration.between(confirmedAt, now);
    }

    public void moveTo(GridPos newPos) { pos = newPos; }

    public String id() { return id; }
    public ObjectKind kind() { return kind; }
    public String label() { return label; }
    public GridPos pos() { return pos; }
    public Knowledge knowledge() { return knowledge; }
    public Source source() { return source; }
    public Instant confirmedAt() { return confirmedAt; }
    public double uncertaintyCells() { return uncertaintyCells; }
}
