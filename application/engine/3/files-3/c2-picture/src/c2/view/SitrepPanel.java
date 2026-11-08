package c2.view;

import c2.app.Version;
import c2.model.Alert;
import c2.model.Knowledge;
import c2.model.MapObject;
import c2.model.OperationsPicture;
import c2.model.SortieView;
import c2.observe.PictureEvent;
import c2.observe.PictureObserver;
import c2.time.SimClock;

import javax.swing.JPanel;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GradientPaint;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.RenderingHints;
import java.time.Duration;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** SITREP panel: dark glass cards, status chips, stat tiles, sortie progress bars, alert cards. */
public class SitrepPanel extends JPanel implements PictureObserver {
    public static final int WIDTH = 370;
    private static final DateTimeFormatter CLOCK = DateTimeFormatter.ofPattern("HH:mm:ss").withZone(ZoneOffset.UTC);
    private static final DateTimeFormatter SHORT = DateTimeFormatter.ofPattern("HH:mm").withZone(ZoneOffset.UTC);

    private static final Color BG_TOP = new Color(14, 22, 36), BG_BOTTOM = new Color(7, 11, 19);
    private static final Color CARD = new Color(255, 255, 255, 14), CARD_EDGE = new Color(120, 170, 220, 45);
    private static final Color TEXT = new Color(228, 236, 245), MUTED = new Color(130, 148, 170);
    private static final Color ACCENT = new Color(60, 220, 230), AMBER = new Color(255, 196, 70);
    private static final Color DANGER = new Color(255, 82, 82), BLUE = new Color(90, 150, 255);

    private static final String SANS = pickFont("Helvetica Neue", "SF Pro Display", "Inter", "Segoe UI", Font.SANS_SERIF);
    private static final String MONO = pickFont("SF Mono", "Menlo", "JetBrains Mono", "Consolas", Font.MONOSPACED);

    private final OperationsPicture picture;
    private final SimClock clock;

    public SitrepPanel(OperationsPicture picture, SimClock clock) {
        this.picture = picture;
        this.clock = clock;
        setPreferredSize(new Dimension(WIDTH, 780));
    }

    private static String pickFont(String... families) {
        List<String> installed = Arrays.asList(GraphicsEnvironment.getLocalGraphicsEnvironment().getAvailableFontFamilyNames());
        for (String family : families) if (installed.contains(family)) return family;
        return families[families.length - 1];
    }

    @Override
    public void onPictureChanged(PictureEvent event) { repaint(); }

    @Override
    protected void paintComponent(Graphics graphics) { paintPanel((Graphics2D) graphics, getWidth(), getHeight()); }

    public void paintPanel(Graphics2D pen, int width, int height) {
        pen.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        pen.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        pen.setPaint(new GradientPaint(0, 0, BG_TOP, 0, height, BG_BOTTOM));
        pen.fillRect(0, 0, width, height);
        pen.setColor(new Color(60, 220, 230, 60));
        pen.fillRect(0, 0, 1, height);                                       // thin accent edge against the map

        int left = 16, inner = width - 32, y = 18;
        synchronized (picture) {
            // Header
            pen.setFont(new Font(SANS, Font.BOLD, 13));
            pen.setColor(MUTED);
            pen.drawString("C2  //  OPERATIONS PICTURE", left, y + 10);
            chip(pen, Version.VERSION, width - 16 - 44, y - 2, 44, ACCENT);
            y += 34;

            // Clock
            pen.setFont(new Font(MONO, Font.BOLD, 30));
            pen.setColor(TEXT);
            pen.drawString(CLOCK.format(picture.now()) + "Z", left, y + 24);
            y += 34;
            chip(pen, String.format("x%.0f SIM", clock.speed()), left, y, 74, BLUE);
            if (clock.isPaused()) chip(pen, "PAUSED", left + 82, y, 64, AMBER);
            y += 30;

            // Phase
            card(pen, left, y, inner, 44);
            label(pen, "PHASE", left + 12, y + 16);
            Color phaseColor = phaseColor(picture.phaseName());
            pen.setColor(phaseColor);
            pen.fillOval(left + 12, y + 24, 9, 9);
            pen.setFont(new Font(SANS, Font.BOLD, 15));
            pen.drawString(picture.phaseName(), left + 28, y + 34);
            y += 54;

            // Stat tiles
            int tileW = (inner - 8) / 2;
            tile(pen, left, y, tileW, "BELIEF PEAK", String.format("%.1f%%", picture.belief().max() * 100), AMBER);
            tile(pen, left + tileW + 8, y, tileW, "ENTROPY", String.format("%.2f", picture.belief().entropy()), TEXT);
            y += 58;
            tile(pen, left, y, tileW, "SORTIES FLOWN", String.valueOf(picture.sortiesFlown()), ACCENT);
            tile(pen, left + tileW + 8, y, tileW, "CONTACTS", String.valueOf(picture.contacts()),
                    picture.contacts() > 0 ? DANGER : TEXT);
            y += 66;

            // Sorties
            label(pen, "SORTIES", left, y);
            y += 8;
            for (SortieView sortie : picture.sorties()) {
                card(pen, left, y, inner, 34);
                pen.setFont(new Font(SANS, Font.BOLD, 12));
                pen.setColor(TEXT);
                pen.drawString(sortie.droneLabel() + "  " + sortie.sortieId(), left + 10, y + 15);
                pen.setFont(new Font(MONO, Font.PLAIN, 10));
                pen.setColor(MUTED);
                pen.drawString("due " + SHORT.format(sortie.due()), left + 150, y + 15);
                chip(pen, sortie.status(), left + inner - 76, y + 4, 68, statusColor(sortie.status()));
                double total = Math.max(1, Duration.between(sortie.launched(), sortie.due()).toSeconds());
                double done = Math.max(0, Math.min(1, Duration.between(sortie.launched(), picture.now()).toSeconds() / total));
                if (!sortie.status().equals("OUT")) done = 1;
                pen.setColor(new Color(255, 255, 255, 25));
                pen.fillRoundRect(left + 10, y + 23, inner - 20, 4, 4, 4);
                pen.setColor(statusColor(sortie.status()));
                pen.fillRoundRect(left + 10, y + 23, (int) ((inner - 20) * done), 4, 4, 4);
                y += 40;
            }
            y += 8;

            // Objects (two columns)
            label(pen, "TRACKED OBJECTS", left, y);
            y += 8;
            List<MapObject> objects = new ArrayList<>(picture.objects());
            int columnW = (inner - 8) / 2, rowsUsed = (objects.size() + 1) / 2;
            card(pen, left, y, inner, rowsUsed * 18 + 10);
            for (int index = 0; index < objects.size(); index++) {
                MapObject object = objects.get(index);
                int cx = left + 8 + (index % 2) * (columnW + 8), cy = y + 18 + (index / 2) * 18;
                Color kindColor = SymbolPainter.colorFor(object.kind());
                boolean solid = object.knowledge() == Knowledge.SOLID;
                pen.setColor(kindColor);
                if (solid) pen.fillOval(cx, cy - 8, 8, 8);
                else { pen.setStroke(new BasicStroke(1.3f)); pen.drawOval(cx, cy - 8, 8, 8); }
                pen.setFont(new Font(SANS, solid ? Font.BOLD : Font.PLAIN, 11));
                pen.setColor(solid ? TEXT : MUTED);
                String name = object.label().length() > 11 ? object.label().substring(0, 11) : object.label();
                pen.drawString(name, cx + 13, cy);
                pen.setFont(new Font(MONO, Font.PLAIN, 9));
                pen.setColor(solid ? ACCENT : MUTED);
                pen.drawString(solid ? "SOLID" : "HOLO", cx + columnW - 34, cy);
            }
            y += rowsUsed * 18 + 22;

            // Alerts
            label(pen, "ALERTS", left, y);
            y += 8;
            int footer = height - 54;
            for (Alert alert : picture.alerts()) {
                if (y + 30 > footer) break;
                Color levelColor = switch (alert.level()) {
                    case URGENT, OVERDUE -> DANGER;
                    case CONTACT -> AMBER;
                    default -> MUTED;
                };
                card(pen, left, y, inner, 26);
                pen.setColor(levelColor);
                pen.fillRoundRect(left, y, 3, 26, 3, 3);
                pen.setFont(new Font(MONO, Font.PLAIN, 10));
                pen.setColor(MUTED);
                pen.drawString(SHORT.format(alert.time()), left + 10, y + 17);
                pen.setFont(new Font(SANS, alert.level().ordinal() > 0 ? Font.BOLD : Font.PLAIN, 11));
                pen.setColor(alert.level().ordinal() > 0 ? levelColor : TEXT);
                String text = alert.text().length() > 40 ? alert.text().substring(0, 39) + "…" : alert.text();
                pen.drawString(text, left + 48, y + 17);
                y += 30;
            }
        }

        // Footer keys
        pen.setColor(new Color(255, 255, 255, 18));
        pen.fillRect(0, height - 46, width, 1);
        pen.setFont(new Font(MONO, Font.PLAIN, 10));
        pen.setColor(MUTED);
        pen.drawString("SPACE pause   +/- speed   H halt   R resume", left, height - 28);
        pen.drawString("P replan   arrows pan   Z/X zoom", left, height - 13);
    }

    private static void card(Graphics2D pen, int x, int y, int width, int height) {
        pen.setColor(CARD);
        pen.fillRoundRect(x, y, width, height, 10, 10);
        pen.setColor(CARD_EDGE);
        pen.setStroke(new BasicStroke(1f));
        pen.drawRoundRect(x, y, width, height, 10, 10);
    }

    private static void tile(Graphics2D pen, int x, int y, int width, String name, String value, Color valueColor) {
        card(pen, x, y, width, 50);
        label(pen, name, x + 10, y + 16);
        pen.setFont(new Font(MONO, Font.BOLD, 20));
        pen.setColor(valueColor);
        pen.drawString(value, x + 10, y + 40);
    }

    private static void label(Graphics2D pen, String text, int x, int y) {
        pen.setFont(new Font(SANS, Font.BOLD, 10));
        pen.setColor(MUTED);
        pen.drawString(text, x, y);
    }

    private static void chip(Graphics2D pen, String text, int x, int y, int width, Color color) {
        pen.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 40));
        pen.fillRoundRect(x, y, width, 18, 18, 18);
        pen.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 170));
        pen.setStroke(new BasicStroke(1f));
        pen.drawRoundRect(x, y, width, 18, 18, 18);
        pen.setFont(new Font(SANS, Font.BOLD, 10));
        pen.setColor(color);
        int textWidth = pen.getFontMetrics().stringWidth(text);
        pen.drawString(text, x + (width - textWidth) / 2, y + 13);
    }

    private static Color phaseColor(String phase) {
        return switch (phase) {
            case "EXECUTE" -> ACCENT;
            case "REGROUP" -> AMBER;
            case "HALTED" -> DANGER;
            default -> BLUE;
        };
    }

    private static Color statusColor(String status) {
        return switch (status) {
            case "REPORTED" -> new Color(90, 220, 130);
            case "LANDED" -> AMBER;
            case "LOST" -> DANGER;
            default -> ACCENT;
        };
    }
}
