package frc.dashboard;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

class VisionStreamPanel extends JPanel {

    private static final Color BG = new Color(6, 10, 18);
    private static final Color FG = new Color(236, 242, 248);

    private volatile BufferedImage frame;
    private volatile String statusText = "Stream idle";
    private volatile String streamUrl = "";
    private volatile boolean enabled = true;
    private Thread workerThread;

    VisionStreamPanel() {
        setPreferredSize(new Dimension(960, 540));
        setBackground(BG);
    }

    synchronized void setStreamUrl(String nextStreamUrl) {
        if (nextStreamUrl == null) {
            nextStreamUrl = "";
        }
        if (nextStreamUrl.equals(streamUrl)) {
            return;
        }
        streamUrl = nextStreamUrl;
        restartWorker();
    }

    synchronized void setStreamingEnabled(boolean shouldEnable) {
        if (enabled == shouldEnable) {
            return;
        }
        enabled = shouldEnable;
        restartWorker();
    }

    synchronized void shutdown() {
        stopWorker();
    }

    String getStatusText() {
        return statusText;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g.create();
        g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g2.setColor(BG);
        g2.fillRect(0, 0, getWidth(), getHeight());

        BufferedImage currentFrame = frame;
        if (currentFrame == null) {
            g2.setColor(FG);
            g2.drawString(statusText, 20, 30);
            g2.dispose();
            return;
        }

        double panelAspect = (double) getWidth() / Math.max(1, getHeight());
        double frameAspect = (double) currentFrame.getWidth() / Math.max(1, currentFrame.getHeight());
        int drawWidth = getWidth();
        int drawHeight = getHeight();
        if (frameAspect > panelAspect) {
            drawHeight = (int) Math.round(drawWidth / frameAspect);
        } else {
            drawWidth = (int) Math.round(drawHeight * frameAspect);
        }
        int x = (getWidth() - drawWidth) / 2;
        int y = (getHeight() - drawHeight) / 2;
        g2.drawImage(currentFrame, x, y, drawWidth, drawHeight, null);
        g2.setColor(FG);
        g2.drawString(statusText, 16, getHeight() - 16);
        g2.dispose();
    }

    private synchronized void restartWorker() {
        stopWorker();
        if (!enabled || streamUrl.isBlank()) {
            frame = null;
            statusText = enabled ? "No stream URL selected" : "Stream paused";
            repaintOnEdt();
            return;
        }
        workerThread = new Thread(this::runWorker, "VisionStreamPanel");
        workerThread.setDaemon(true);
        workerThread.start();
    }

    private synchronized void stopWorker() {
        if (workerThread != null) {
            workerThread.interrupt();
            workerThread = null;
        }
    }

    private void runWorker() {
        int consecutiveErrors = 0;
        while (!Thread.currentThread().isInterrupted()) {
            try {
                statusText = "Connecting to " + streamUrl;
                repaintOnEdt();
                streamFrames(streamUrl);
                consecutiveErrors = 0;
            } catch (IOException ex) {
                consecutiveErrors++;
                statusText = "STREAM ERROR (" + consecutiveErrors + "x): " + ex.getMessage()
                        + " — check robot IP / camera";
                frame = null;
                repaintOnEdt();
                // Back off more aggressively after repeated failures
                sleepQuietly(Math.min(1000L * consecutiveErrors, 5000L));
            }
        }
    }

    private void streamFrames(String urlString) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(urlString).openConnection();
        connection.setConnectTimeout(2000);
        connection.setReadTimeout(5000);
        connection.setUseCaches(false);
        connection.setRequestProperty("Connection", "Keep-Alive");

        try (BufferedInputStream input = new BufferedInputStream(connection.getInputStream())) {
            ByteArrayOutputStream jpegBytes = new ByteArrayOutputStream(64 * 1024);
            boolean collecting = false;
            int previous = -1;
            while (!Thread.currentThread().isInterrupted()) {
                int current = input.read();
                if (current < 0) {
                    throw new IOException("Stream closed");
                }
                if (!collecting) {
                    if (previous == 0xFF && current == 0xD8) {
                        jpegBytes.reset();
                        jpegBytes.write(0xFF);
                        jpegBytes.write(0xD8);
                        collecting = true;
                    }
                } else {
                    jpegBytes.write(current);
                    if (previous == 0xFF && current == 0xD9) {
                        BufferedImage decoded = ImageIO.read(new ByteArrayInputStream(jpegBytes.toByteArray()));
                        if (decoded != null) {
                            frame = decoded;
                            statusText = decoded.getWidth() + "x" + decoded.getHeight() + "  " + urlString;
                            repaintOnEdt();
                        }
                        collecting = false;
                    }
                }
                previous = current;
            }
        } finally {
            connection.disconnect();
        }
    }

    private void repaintOnEdt() {
        SwingUtilities.invokeLater(this::repaint);
    }

    private static void sleepQuietly(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
