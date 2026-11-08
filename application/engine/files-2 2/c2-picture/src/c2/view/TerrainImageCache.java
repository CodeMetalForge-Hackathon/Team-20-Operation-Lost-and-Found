package c2.view;

import c2.model.TerrainCell;
import c2.model.TerrainGrid;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.util.HashMap;
import java.util.Map;

/** Terrain never changes, so it's drawn once per zoom level into an image and reused every frame. */
public class TerrainImageCache {
    private final TerrainGrid terrain;
    private final int rows, cols;
    private final Map<Integer, BufferedImage> images = new HashMap<>();
    private final Map<Integer, IsoProjection> projections = new HashMap<>();

    public TerrainImageCache(TerrainGrid terrain, int rows, int cols) {
        this.terrain = terrain;
        this.rows = rows;
        this.cols = cols;
    }

    public synchronized IsoProjection projection(int tileWidth) {
        return projections.computeIfAbsent(tileWidth, width -> new IsoProjection(width, rows, cols, terrain.maxHeight()));
    }

    public synchronized BufferedImage image(int tileWidth) {
        return images.computeIfAbsent(tileWidth, this::render);
    }

    private BufferedImage render(int tileWidth) {
        IsoProjection iso = projection(tileWidth);
        BufferedImage image = new BufferedImage(iso.boardWidth(), iso.boardHeight(), BufferedImage.TYPE_INT_ARGB);
        Graphics2D pen = image.createGraphics();
        pen.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_OFF);
        pen.setStroke(new BasicStroke(1f));

        // Back to front (painter's order): squares with a smaller row+col are farther away.
        for (int depth = 0; depth <= rows + cols - 2; depth++) {
            for (int row = Math.max(0, depth - cols + 1); row <= Math.min(rows - 1, depth); row++) {
                int col = depth - row;
                drawTile(pen, iso, row, col);
            }
        }
        pen.dispose();
        return image;
    }

    private void drawTile(Graphics2D pen, IsoProjection iso, int row, int col) {
        TerrainCell cell = terrain.at(row, col);
        double h = cell.heightMeters();
        int topX = (int) iso.x(row, col), topY = (int) iso.y(row, col, h);
        int halfW = iso.tileWidth / 2, halfH = iso.tileHeight / 2;

        // Side walls down to the lower of the two front neighbours, so height reads as depth.
        double frontLeft = row + 1 < rows ? terrain.at(row + 1, col).heightMeters() : 0;
        double frontRight = col + 1 < cols ? terrain.at(row, col + 1).heightMeters() : 0;
        Color base = cell.cover().color;
        int leftDrop = (int) Math.max(0, (h - frontLeft) * iso.pixelsPerMeter);
        int rightDrop = (int) Math.max(0, (h - frontRight) * iso.pixelsPerMeter);
        if (leftDrop > 0) {
            pen.setColor(shade(base, 0.62));
            pen.fillPolygon(new int[]{topX - halfW, topX, topX, topX - halfW},
                    new int[]{topY + halfH, topY + 2 * halfH, topY + 2 * halfH + leftDrop, topY + halfH + leftDrop}, 4);
        }
        if (rightDrop > 0) {
            pen.setColor(shade(base, 0.78));
            pen.fillPolygon(new int[]{topX, topX + halfW, topX + halfW, topX},
                    new int[]{topY + 2 * halfH, topY + halfH, topY + halfH + rightDrop, topY + 2 * halfH + rightDrop}, 4);
        }

        // Top face, lit from the north-west: brighter when higher than its front neighbours.
        double light = 1.0 + Math.max(-0.25, Math.min(0.25, ((h - frontLeft) + (h - frontRight)) * 0.012));
        Polygon top = new Polygon(new int[]{topX, topX + halfW, topX, topX - halfW},
                new int[]{topY, topY + halfH, topY + 2 * halfH, topY + halfH}, 4);
        pen.setColor(shade(base, light));
        pen.fillPolygon(top);
        if (cell.cliff()) {
            pen.setColor(new Color(60, 40, 30));
            pen.drawPolygon(top);
        }
    }

    private static Color shade(Color color, double factor) {
        return new Color(clamp(color.getRed() * factor), clamp(color.getGreen() * factor), clamp(color.getBlue() * factor));
    }

    private static int clamp(double value) { return (int) Math.max(0, Math.min(255, value)); }
}
