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
        if (args.length > 2) for (int step = 0; step < Integer.parseInt(args[2]); step++) {
            app.camera.submit(c2.input.Command.of(c2.input.CommandType.ZOOM_IN));
        }
        int every = Math.max(1, ticks / 4);
        for (int tick = 1; tick <= ticks; tick++) {
            app.clock.tick();
            app.reportAdapter.scanOnce(app.queue);
            if (tick % every == 0) save(board, sitrep, folder.resolve("tick_" + tick + ".png"));
        }
    }

    private static void save(IsoBoardView board, SitrepPanel sitrep, Path file) throws Exception {
        BufferedImage image = new BufferedImage(1100 + SitrepPanel.WIDTH, 780, BufferedImage.TYPE_INT_RGB);
        Graphics2D pen = image.createGraphics();
        board.paintBoard(pen, 1100, 780);
        pen.translate(1100, 0);
        sitrep.paintPanel(pen, SitrepPanel.WIDTH, 780);
        pen.dispose();
        ImageIO.write(image, "png", file.toFile());
        System.out.println("saved " + file);
    }
}
