package c2.view;

import c2.model.Alert;
import c2.model.MapObject;
import c2.model.OperationsPicture;
import c2.observe.PictureEvent;
import c2.observe.PictureObserver;
import c2.time.SimClock;

import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.time.Duration;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;

/** Right-hand status panel: clock, phase, sortie, belief, objects with knowledge state, alerts, keys. */
public class SitrepPanel extends JPanel implements PictureObserver {
    private static final DateTimeFormatter CLOCK = DateTimeFormatter.ofPattern("HH:mm:ss 'Z'").withZone(ZoneOffset.UTC);
    private static final Color TEXT = new Color(200, 230, 205), DIM = new Color(120, 140, 125);

    private final OperationsPicture picture;
    private final SimClock clock;

    public SitrepPanel(OperationsPicture picture, SimClock clock) {
        this.picture = picture;
        this.clock = clock;
        setPreferredSize(new Dimension(340, 760));
        setBackground(new Color(14, 20, 16));
    }

    @Override
    public void onPictureChanged(PictureEvent event) { repaint(); }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        paintPanel((Graphics2D) graphics);
    }

    public void paintPanel(Graphics2D pen) {
        pen.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        int x = 14, y = 26;
        pen.setFont(new Font(Font.MONOSPACED, Font.BOLD, 15));
        pen.setColor(TEXT);
        pen.drawString("C2 OPERATIONS PICTURE", x, y);
        pen.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));

        synchronized (picture) {
            y += 24;
            pen.drawString("TIME   " + CLOCK.format(picture.now()), x, y);
            y += 16;
            pen.setColor(DIM);
            pen.drawString(String.format("SPEED  x%.0f %s", clock.speed(), clock.isPaused() ? "[PAUSED]" : ""), x, y);
            y += 22;
            pen.setColor(TEXT);
            pen.drawString("PHASE  " + picture.phaseName(), x, y);
            y += 16;
            pen.drawString("SORTIE " + picture.sortieStatus(), x, y);
            y += 16;
            pen.setColor(DIM);
            pen.drawString(String.format("BELIEF peak %.2f%%  entropy %.2f", picture.belief().max() * 100,
                    picture.belief().entropy()), x, y);

            y += 28;
            pen.setColor(TEXT);
            pen.drawString("OBJECTS", x, y);
            for (MapObject object : picture.objects()) {
                y += 16;
                Duration age = object.ageAt(picture.now());
                String ageText = age == null ? "never" : (age.toMinutes() + "m ago");
                pen.setColor(SymbolPainter.colorFor(object.kind()));
                pen.drawString(String.format("%-9s %-5s %s", object.label(), object.knowledge(), ageText), x + 8, y);
            }

            y += 28;
            pen.setColor(TEXT);
            pen.drawString("ALERTS", x, y);
            for (Alert alert : picture.alerts()) {
                y += 16;
                pen.setColor(switch (alert.level()) {
                    case URGENT, OVERDUE -> new Color(255, 90, 80);
                    case CONTACT -> new Color(255, 220, 90);
                    default -> DIM;
                });
                String text = DateTimeFormatter.ofPattern("HH:mm").withZone(ZoneOffset.UTC).format(alert.time())
                        + " " + alert.text();
                pen.drawString(text.length() > 44 ? text.substring(0, 44) : text, x + 8, y);
            }
        }

        int keysY = getHeight() > 0 ? getHeight() - 70 : 690;
        pen.setColor(DIM);
        pen.drawString("SPACE pause   +/- speed", x, keysY);
        pen.drawString("H halt   R resume   P replan", x, keysY + 16);
        pen.drawString("arrows pan    Z/X zoom", x, keysY + 32);
    }
}
