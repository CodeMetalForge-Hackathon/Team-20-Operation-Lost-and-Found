package c2.app;

import c2.model.GeoFrame;
import c2.model.GridPos;
import c2.model.Knowledge;
import c2.model.MapObject;
import c2.model.ObjectKind;
import c2.model.OperationsPicture;
import c2.model.PatrolRoute;
import c2.model.Source;
import c2.model.TerrainGrid;
import c2.model.Zone;
import c2.model.ZoneType;

import java.time.Duration;
import java.time.Instant;
import java.util.List;

/** The starting picture: 5 x 5 mile AO, 50 m squares, TOC, 3 UAVs, last known location, enemy presence. */
public final class Scenario {
    public static final String SUBJECT = "subject-1", TOC = "toc";
    public static final List<String> DRONES = List.of("uav-1", "uav-2");
    public static final GeoFrame FRAME = new GeoFrame(162, 162, 50);
    public static final GridPos LAST_KNOWN = new GridPos(70, 82);
    public static final GridPos TRUE_SUBJECT = new GridPos(90, 99);        // hidden; only the simulated drone knows
    public static final Instant START = Instant.parse("2026-11-08T13:00:00Z");

    private Scenario() { }

    public static OperationsPicture build() {
        TerrainGrid terrain = TerrainGrid.synthetic(FRAME);
        OperationsPicture picture = new OperationsPicture(FRAME, terrain, START);
        GridPos tocPos = new GridPos(132, 34);

        picture.add(new MapObject(TOC, ObjectKind.TOC, "TOC", tocPos, Knowledge.SOLID, Source.ANCHOR, START, 0));
        for (int index = 0; index < DRONES.size(); index++) {
            picture.add(new MapObject(DRONES.get(index), ObjectKind.DRONE, "UAV-" + (index + 1), tocPos,
                    Knowledge.SOLID, Source.ANCHOR, START, 0));
        }
        picture.add(new MapObject(SUBJECT, ObjectKind.SUBJECT, "SUBJECT", LAST_KNOWN, Knowledge.HOLO, Source.PREDICTED, null, 12));

        // Known hazards
        picture.add(new MapObject("hazard-cliff", ObjectKind.HAZARD, "CLIFF", new GridPos(62, 101), Knowledge.SOLID, Source.ANCHOR, START, 0));
        picture.add(new MapObject("hazard-fire", ObjectKind.HAZARD, "FIRE", new GridPos(40, 58), Knowledge.SOLID, Source.REPORT, START, 0));
        picture.add(new MapObject("hazard-ford", ObjectKind.HAZARD, "FAST WATER", new GridPos(104, 43), Knowledge.SOLID, Source.ANCHOR, START, 0));

        // Enemy area of operations (north-east) and the forces in and around it
        picture.addZone(new Zone("ENEMY AO", ZoneType.ENEMY_AO, List.of(
                new GridPos(10, 104), new GridPos(12, 152), new GridPos(58, 156),
                new GridPos(66, 128), new GridPos(48, 108)), Knowledge.SOLID));
        picture.add(new MapObject("hostile-cp", ObjectKind.HOSTILE, "ENEMY CP", new GridPos(24, 140), Knowledge.SOLID, Source.REPORT, START, 1));
        picture.add(new MapObject("hostile-op", ObjectKind.HOSTILE, "ENEMY OP", new GridPos(46, 118), Knowledge.HOLO, Source.PREDICTED, null, 5));
        picture.add(new MapObject("hostile-sqd", ObjectKind.HOSTILE, "ENEMY SQD", new GridPos(38, 132), Knowledge.HOLO, Source.PREDICTED, null, 7));
        picture.add(new MapObject("hostile-veh", ObjectKind.HOSTILE, "ENEMY VEH", new GridPos(54, 146), Knowledge.HOLO, Source.PREDICTED, null, 6));
        picture.add(new MapObject("hostile-snp", ObjectKind.HOSTILE, "SNIPER?", new GridPos(64, 112), Knowledge.HOLO, Source.PREDICTED, null, 4));
        picture.add(new MapObject("hostile-ptl", ObjectKind.HOSTILE, "PATROL", new GridPos(60, 120), Knowledge.HOLO, Source.PREDICTED, null, 5));
        picture.addPatrol(new PatrolRoute("hostile-ptl", List.of(
                new GridPos(60, 120), new GridPos(72, 110), new GridPos(78, 124), new GridPos(66, 136)),
                Duration.ofMinutes(50)));

        picture.belief().seedGaussian(LAST_KNOWN, 14, terrain);
        return picture;
    }
}
