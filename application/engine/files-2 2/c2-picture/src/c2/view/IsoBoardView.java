package c2.view;

import c2.model.BeliefGrid;
import c2.model.Knowledge;
import c2.model.MapObject;
import c2.model.ObjectKind;
import c2.model.OperationsPicture;
import c2.model.TerrainGrid;
import c2.model.TravelPath;
import c2.model.Waypoint;
import c2.observe.PictureEvent;
import c2.observe.PictureObserver;

import javax.swing.JPanel;
import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Composite;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;

/** The isometric board. Observer of the picture; draws terrain (cached), belief, paths, objects. */
public class IsoBoardView extends JPanel implements PictureObserver {
    private static final KnowledgeStyle HOLO = new HoloStyle();
    private static final KnowledgeStyle SOLID = new SolidStyle();
    private static final Color BACKGROUND = new Color(12, 16, 20);

    private final OperationsPicture picture;
    private final TerrainImageCache terrainCache;
    private final Camera camera;
    private final SpriteLibrary sprites;
    private final long startNanos = System.nanoTime();
    private volatile double lastFrameMillis;

    public IsoBoardView(OperationsPicture picture, TerrainImageCache terrainCache, Camera camera, SpriteLibrary sprites) {
        this.picture = picture;
        this.terrainCache = terrainCache;
        this.camera = camera;
        this.sprites = sprites;
        setPreferredSize(new Dimension(1100, 760));
        setBackground(BACKGROUND);
        setFocusable(true);
    }

    @Override
    public void onPictureChanged(PictureEvent event) { repaint(); }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        long started = System.nanoTime();
        paintBoard((Graphics2D) graphics, getWidth(), getHeight());
        lastFrameMillis = (System.nanoTime() - started) / 1e6;
        graphics.setColor(new Color(150, 160, 170));
        graphics.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 11));
        graphics.drawString(String.format("frame %.1f ms   zoom %d   arrows pan  Z/X zoom", lastFrameMillis,
                camera.zoomIndex()), 10, getHeight() - 10);
    }

    /** All drawing, separate from Swing so it can also render off-screen (snapshots, tests). */
    public void paintBoard(Graphics2D pen, int viewWidth, int viewHeight) {
        pen.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        pen.setColor(BACKGROUND);
        pen.fillRect(0, 0, viewWidth, viewHeight);

        int tileWidth = camera.tileWidth();
        IsoProjection iso = terrainCache.projection(tileWidth);
        BufferedImage terrainImage = terrainCache.image(tileWidth);
        TerrainGrid terrain = picture.terrain();
        double centerHeight = terrain.at((int) camera.centerRow(), (int) camera.centerCol()).heightMeters();
        int offsetX = (int) (viewWidth / 2.0 - iso.x(camera.centerRow(), camera.centerCol()));
        int offsetY = (int) (viewHeight / 2.0 - iso.centerY(camera.centerRow(), camera.centerCol(), centerHeight));
        double clockSeconds = (System.nanoTime() - startNanos) / 1e9;

        pen.drawImage(terrainImage, offsetX, offsetY, null);
        pen.translate(offsetX, offsetY);
        synchronized (picture) {
            drawBelief(pen, iso, terrain);
            var tracks = picture.flownTracks();                                  // only the last two flown tracks, to keep it readable
            for (int index = Math.max(0, tracks.size() - 2); index < tracks.size(); index++) {
                drawPath(pen, iso, terrain, tracks.get(index), clockSeconds);
            }
            if (picture.plannedPath() != null) drawPath(pen, iso, terrain, picture.plannedPath(), clockSeconds);
            for (MapObject object : picture.objects()) drawObject(pen, iso, terrain, object, clockSeconds);
        }
        pen.translate(-offsetX, -offsetY);
        drawLegend(pen);
    }

    /** Probability as a translucent red overlay (it's a prediction, so holo-style). */
    private void drawBelief(Graphics2D pen, IsoProjection iso, TerrainGrid terrain) {
        BeliefGrid belief = picture.belief();
        double max = belief.max();
        if (max <= 0) return;
        int halfW = iso.tileWidth / 2, halfH = iso.tileHeight / 2;
        for (int row = 0; row < belief.rows(); row++) {
            for (int col = 0; col < belief.cols(); col++) {
                double strength = belief.at(row, col) / max;
                if (strength < 0.06) continue;
                int alpha = (int) Math.min(190, 30 + 170 * strength);
                pen.setColor(new Color(255, (int) (190 * (1 - strength)), 40, alpha));
                int x = (int) iso.x(row, col), y = (int) iso.y(row, col, terrain.at(row, col).heightMeters());
                pen.fillPolygon(new Polygon(new int[]{x, x + halfW, x, x - halfW}, new int[]{y, y + halfH, y + 2 * halfH, y + halfH}, 4));
            }
        }
    }

    private void drawPath(Graphics2D pen, IsoProjection iso, TerrainGrid terrain, TravelPath path, double clockSeconds) {
        Path2D.Double line = new Path2D.Double();
        boolean first = true;
        for (Waypoint waypoint : path.waypoints()) {
            double h = terrain.at(waypoint.pos()).heightMeters() + 40;       // drawn a little above ground
            double x = iso.x(waypoint.pos().row(), waypoint.pos().col());
            double y = iso.centerY(waypoint.pos().row(), waypoint.pos().col(), h);
            if (first) { line.moveTo(x, y); first = false; } else line.lineTo(x, y);
        }
        if (path.knowledge() == Knowledge.HOLO) {
            float phase = (float) (clockSeconds * 12 % 10);
            pen.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f, new float[]{6f, 4f}, 10f - phase));
            pen.setColor(new Color(60, 230, 230, 200));
        } else {
            pen.setStroke(new BasicStroke(1.5f));
            pen.setColor(new Color(60, 230, 230, 70));
        }
        pen.draw(line);
        if (path.knowledge() == Knowledge.HOLO) {
            for (Waypoint waypoint : path.waypoints()) {
                double h = terrain.at(waypoint.pos()).heightMeters() + 40;
                double x = iso.x(waypoint.pos().row(), waypoint.pos().col());
                double y = iso.centerY(waypoint.pos().row(), waypoint.pos().col(), h);
                HOLO.paint(pen, new Ellipse2D.Double(x - 3, y - 3, 6, 6), new Color(60, 230, 230), clockSeconds);
            }
        }
    }

    private void drawObject(Graphics2D pen, IsoProjection iso, TerrainGrid terrain, MapObject object, double clockSeconds) {
        double h = terrain.at(object.pos()).heightMeters();
        double x = iso.x(object.pos().row(), object.pos().col());
        double groundY = iso.centerY(object.pos().row(), object.pos().col(), h);
        boolean holo = object.knowledge() == Knowledge.HOLO;
        KnowledgeStyle style = holo ? HOLO : SOLID;
        Color color = SymbolPainter.colorFor(object.kind());

        if (holo && object.uncertaintyCells() > 1) {                        // prediction ring, flattened to the iso ground
            double ringW = object.uncertaintyCells() * iso.tileWidth, ringH = object.uncertaintyCells() * iso.tileHeight;
            pen.setStroke(new BasicStroke(1.2f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[]{5f, 5f}, 0f));
            pen.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 120));
            pen.draw(new Ellipse2D.Double(x - ringW / 2, groundY - ringH / 2, ringW, ringH));
        }

        double size = 14;
        double symbolY = groundY - 10;                                       // stands slightly above the ground
        pen.setColor(new Color(0, 0, 0, 90));
        pen.drawLine((int) x, (int) groundY, (int) x, (int) symbolY);       // stalk to the ground point

        BufferedImage droneSprite = sprites.drone();
        if (object.kind() == ObjectKind.DRONE && droneSprite != null) {
            double spriteW = 30, spriteH = spriteW * droneSprite.getHeight() / droneSprite.getWidth();
            Composite before = pen.getComposite();
            if (holo) pen.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER,
                    (float) (0.45 + 0.25 * Math.sin(clockSeconds * 3))));
            pen.drawImage(droneSprite, (int) (x - spriteW / 2), (int) (symbolY - spriteH / 2), (int) spriteW, (int) spriteH, null);
            pen.setComposite(before);
        } else {
            style.paint(pen, SymbolPainter.shapeFor(object.kind(), x, symbolY, size), color, clockSeconds);
        }

        pen.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 10));
        pen.setColor(holo ? new Color(color.getRed(), color.getGreen(), color.getBlue(), 170) : color);
        pen.drawString(object.label() + (holo ? " (est)" : ""), (int) x + 10, (int) symbolY - 6);
    }

    private void drawLegend(Graphics2D pen) {
        pen.setColor(new Color(0, 0, 0, 150));
        pen.fillRoundRect(8, 8, 300, 22, 8, 8);
        pen.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 11));
        pen.setColor(new Color(220, 225, 230));
        pen.drawString("SOLID = confirmed   dashed/translucent = HOLO (predicted)", 16, 23);
    }
}
