package c2.app;

import c2.view.IsoBoardView;
import c2.view.SitrepPanel;

import javax.imageio.ImageIO;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.nio.file.Path;

/**
 * Headless run: steps the clock by hand and saves PNG snapshots of the board + SITREP.
 *   java -Djava.awt.headless=true -cp out c2.app.Snapshot 400 snapshots
 */
public class Snapshot {
    public static void main(String[] args) throws Exception {
        int ticks = args.length > 0 ? Integer.parseInt(args[0]) : 400;
        Path folder = Path.of(args.length > 1 ? args[1] : "snapshots");
        java.nio.file.Files.createDirectories(folder);
        C2App app = new C2App(Path.of("inbox"), Path.of("out"), Path.of("assets"), true);
        app.controller.begin();
        IsoBoardView board = new IsoBoardView(app.picture, app.terrainCache, app.camera, app.sprites);
        SitrepPanel sitrep = new SitrepPanel(app.picture, app.clock);
        int every = Math.max(1, ticks / 4);
        for (int tick = 1; tick <= ticks; tick++) {
            app.clock.tick();
            app.reportAdapter.scanOnce(app.queue);
            if (tick % every == 0) save(board, sitrep, folder.resolve("tick_" + tick + ".png"));
        }
    }

    private static void save(IsoBoardView board, SitrepPanel sitrep, Path file) throws Exception {
        BufferedImage image = new BufferedImage(1100 + 340, 760, BufferedImage.TYPE_INT_RGB);
        Graphics2D pen = image.createGraphics();
        board.paintBoard(pen, 1100, 760);
        pen.translate(1100, 0);
        pen.setColor(new java.awt.Color(14, 20, 16));
        pen.fillRect(0, 0, 340, 760);
        sitrep.paintPanel(pen);
        pen.dispose();
        ImageIO.write(image, "png", file.toFile());
        System.out.println("saved " + file);
    }
}
