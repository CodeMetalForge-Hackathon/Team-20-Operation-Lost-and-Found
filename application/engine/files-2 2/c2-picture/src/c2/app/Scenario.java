package c2.app;

import c2.model.GeoFrame;
import c2.model.GridPos;
import c2.model.Knowledge;
import c2.model.MapObject;
import c2.model.ObjectKind;
import c2.model.OperationsPicture;
import c2.model.Source;
import c2.model.TerrainGrid;

import java.time.Instant;

/** The starting picture for the demo: 5 x 5 mile AO, 50 m squares, TOC, last known location, known threats. */
public final class Scenario {
    public static final String SUBJECT = "subject-1", DRONE = "drone-1", TOC = "toc";
    public static final GeoFrame FRAME = new GeoFrame(162, 162, 50);
    public static final GridPos LAST_KNOWN = new GridPos(70, 82);
    public static final GridPos TRUE_SUBJECT = new GridPos(86, 98);        // hidden; only the simulated drone knows
    public static final Instant START = Instant.parse("2026-11-08T13:00:00Z");

    private Scenario() { }

    public static OperationsPicture build() {
        TerrainGrid terrain = TerrainGrid.synthetic(FRAME);
        OperationsPicture picture = new OperationsPicture(FRAME, terrain, START);
        GridPos tocPos = new GridPos(132, 34);

        picture.add(new MapObject(TOC, ObjectKind.TOC, "TOC", tocPos, Knowledge.SOLID, Source.ANCHOR, START, 0));
        picture.add(new MapObject(DRONE, ObjectKind.DRONE, "DRONE", tocPos, Knowledge.SOLID, Source.ANCHOR, START, 0));
        picture.add(new MapObject(SUBJECT, ObjectKind.SUBJECT, "SUBJECT", LAST_KNOWN, Knowledge.HOLO, Source.PREDICTED, null, 12));
        picture.add(new MapObject("hazard-cliff", ObjectKind.HAZARD, "CLIFF", new GridPos(62, 101), Knowledge.SOLID, Source.ANCHOR, START, 0));
        picture.add(new MapObject("hazard-fire", ObjectKind.HAZARD, "FIRE", new GridPos(40, 58), Knowledge.SOLID, Source.REPORT, START, 0));
        picture.add(new MapObject("hostile-1", ObjectKind.HOSTILE, "ENEMY", new GridPos(34, 128), Knowledge.HOLO, Source.PREDICTED, null, 7));

        picture.belief().seedGaussian(LAST_KNOWN, 14, terrain);
        return picture;
    }
}
