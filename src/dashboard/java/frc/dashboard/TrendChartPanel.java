package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.text.DecimalFormat;

import javax.swing.JPanel;

/**
 * Lightweight Swing panel that paints a scrolling line chart from a
 * {@link TrendDataStore}.  Renders a title, Y-axis labels, an optional
 * threshold line, and the data series.  No third-party charting library
 * required; all rendering is done with Java2D for minimal overhead during
 * the 100 ms dashboard refresh cycle.
 */
public final class TrendChartPanel extends JPanel {

    private static final Color BG = new Color(21, 31, 46);
    private static final Color GRID = new Color(55, 79, 107, 80);
    private static final Color AXIS_TEXT = new Color(166, 184, 206);
    private static final Color TITLE_COLOR = new Color(243, 247, 252);
    private static final Font TITLE_FONT = new Font("Segoe UI", Font.BOLD, 13);
    private static final Font AXIS_FONT = new Font("Segoe UI", Font.PLAIN, 11);
    private static final DecimalFormat AXIS_FMT = new DecimalFormat("0.0");
    private static final int INSET_LEFT = 48;
    private static final int INSET_RIGHT = 8;
    private static final int INSET_TOP = 22;
    private static final int INSET_BOTTOM = 18;

    private final String title;
    private final Color lineColor;
    private final double thresholdValue;
    private final Color thresholdColor;
    private TrendDataStore store;
    private double fixedMinY = Double.NaN;
    private double fixedMaxY = Double.NaN;

    public TrendChartPanel(String title, Color lineColor) {
        this(title, lineColor, Double.NaN, null);
    }

    public TrendChartPanel(String title, Color lineColor, double thresholdValue, Color thresholdColor) {
        this.title = title;
        this.lineColor = lineColor;
        this.thresholdValue = thresholdValue;
        this.thresholdColor = thresholdColor;
        setBackground(BG);
        setPreferredSize(new Dimension(360, 160));
    }

    public void setStore(TrendDataStore store) {
        this.store = store;
    }

    public void setFixedYRange(double min, double max) {
        this.fixedMinY = min;
        this.fixedMaxY = max;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g.create();
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int w = getWidth();
        int h = getHeight();
        int plotW = w - INSET_LEFT - INSET_RIGHT;
        int plotH = h - INSET_TOP - INSET_BOTTOM;

        // Title
        g2.setColor(TITLE_COLOR);
        g2.setFont(TITLE_FONT);
        g2.drawString(title, INSET_LEFT, 15);

        if (store == null || store.size() < 2 || plotW < 4 || plotH < 4) {
            g2.setColor(AXIS_TEXT);
            g2.setFont(AXIS_FONT);
            g2.drawString("Waiting for data...", INSET_LEFT + 4, INSET_TOP + plotH / 2);
            g2.dispose();
            return;
        }

        // Compute Y range
        double minY, maxY;
        if (Double.isFinite(fixedMinY) && Double.isFinite(fixedMaxY)) {
            minY = fixedMinY;
            maxY = fixedMaxY;
        } else {
            minY = Double.MAX_VALUE;
            maxY = -Double.MAX_VALUE;
            for (int i = 0; i < store.size(); i++) {
                double v = store.value(i);
                if (Double.isFinite(v)) {
                    minY = Math.min(minY, v);
                    maxY = Math.max(maxY, v);
                }
            }
            if (minY == Double.MAX_VALUE) {
                minY = 0;
                maxY = 1;
            }
            double margin = (maxY - minY) * 0.1;
            if (margin < 0.01) margin = 0.5;
            minY -= margin;
            maxY += margin;
        }

        double tMin = store.timestamp(0);
        double tMax = store.timestamp(store.size() - 1);
        if (tMax <= tMin) {
            tMax = tMin + 1;
        }

        // Grid lines (3 horizontal)
        g2.setColor(GRID);
        g2.setStroke(new BasicStroke(1));
        g2.setFont(AXIS_FONT);
        FontMetrics fm = g2.getFontMetrics();
        for (int i = 0; i <= 3; i++) {
            double frac = i / 3.0;
            int yPx = INSET_TOP + (int) (plotH * (1.0 - frac));
            g2.setColor(GRID);
            g2.drawLine(INSET_LEFT, yPx, INSET_LEFT + plotW, yPx);
            double yVal = minY + frac * (maxY - minY);
            g2.setColor(AXIS_TEXT);
            String label = AXIS_FMT.format(yVal);
            g2.drawString(label, INSET_LEFT - fm.stringWidth(label) - 4, yPx + fm.getAscent() / 2);
        }

        // Threshold line
        if (Double.isFinite(thresholdValue) && thresholdColor != null
                && thresholdValue >= minY && thresholdValue <= maxY) {
            int tY = INSET_TOP + (int) (plotH * (1.0 - (thresholdValue - minY) / (maxY - minY)));
            g2.setColor(thresholdColor);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                    10.0f, new float[]{6, 4}, 0));
            g2.drawLine(INSET_LEFT, tY, INSET_LEFT + plotW, tY);
        }

        // Data line
        g2.setColor(lineColor);
        g2.setStroke(new BasicStroke(1.5f));
        int prevX = -1, prevY = -1;
        for (int i = 0; i < store.size(); i++) {
            double v = store.value(i);
            if (!Double.isFinite(v)) continue;
            double t = store.timestamp(i);
            int px = INSET_LEFT + (int) (plotW * (t - tMin) / (tMax - tMin));
            int py = INSET_TOP + (int) (plotH * (1.0 - (v - minY) / (maxY - minY)));
            py = Math.max(INSET_TOP, Math.min(INSET_TOP + plotH, py));
            if (prevX >= 0) {
                g2.drawLine(prevX, prevY, px, py);
            }
            prevX = px;
            prevY = py;
        }

        // Time axis labels
        g2.setColor(AXIS_TEXT);
        g2.setFont(AXIS_FONT);
        double span = tMax - tMin;
        String startLabel = AXIS_FMT.format(span) + "s ago";
        g2.drawString(startLabel, INSET_LEFT, INSET_TOP + plotH + 14);
        g2.drawString("now", INSET_LEFT + plotW - fm.stringWidth("now"), INSET_TOP + plotH + 14);

        g2.dispose();
    }
}
