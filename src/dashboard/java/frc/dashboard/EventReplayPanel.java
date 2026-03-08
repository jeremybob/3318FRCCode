package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.io.IOException;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.SwingConstants;
import javax.swing.Timer;
import javax.swing.filechooser.FileNameExtensionFilter;

/**
 * Self-contained panel that lets the operator load a match CSV produced by
 * {@link MatchCsvLogger} and scrub through the data.  The panel owns its own
 * {@link Timer} that ticks at 100 ms when "playing" to advance the cursor
 * through the rows.
 *
 * <p>Only the small subset of fields captured in the CSV is displayed.
 * Trend charts for battery voltage, shooter speed, and heading are embedded.
 */
public final class EventReplayPanel extends JPanel {

    private static final Color BG = new Color(11, 18, 28);
    private static final Color CARD = new Color(21, 31, 46);
    private static final Color BORDER_C = new Color(55, 79, 107);
    private static final Color TEXT = new Color(243, 247, 252);
    private static final Color MUTED = new Color(166, 184, 206);
    private static final Color OK = new Color(46, 170, 88);
    private static final Color BAD = new Color(202, 65, 65);
    private static final Color WARN = new Color(224, 157, 55);
    private static final Color INFO = new Color(58, 118, 179);

    private static final Font TITLE_FONT = new Font("Segoe UI", Font.BOLD, 14);
    private static final Font METRIC_FONT = new Font("Segoe UI", Font.PLAIN, 16);
    private static final Font ACTION_FONT = new Font("Segoe UI", Font.BOLD, 14);
    private static final DecimalFormat ONE_D = new DecimalFormat("0.0");

    // Replay state
    private List<MatchCsvLogger.CsvRow> rows;
    private int cursor;
    private boolean playing;
    private final Timer playTimer;

    // Controls
    private final JButton loadButton = styledButton("Load CSV");
    private final JButton playPauseButton = styledButton("Play");
    private final JButton rewindButton = styledButton("|<");
    private final JSlider scrubSlider = new JSlider(0, 0, 0);

    // Status labels
    private final JLabel fileLabel = metricLabel("File: (none)");
    private final JLabel rowCountLabel = metricLabel("Rows: 0");
    private final JLabel cursorLabel = metricLabel("Position: 0 / 0");
    private final JLabel timestampLabel = metricLabel("Timestamp: --");

    // Data labels (current row)
    private final JLabel batteryLabel = metricLabel("Battery: --");
    private final JLabel modeLabel = metricLabel("Mode: --");
    private final JLabel matchTimeLabel = metricLabel("Match time: --");
    private final JLabel shooterLabel = metricLabel("Shooter: L -- / R -- RPS");
    private final JLabel readyLabel = metricLabel("Ready: --");
    private final JLabel alignLabel = metricLabel("Align: --");
    private final JLabel poseLabel = metricLabel("Pose: --");
    private final JLabel canLabel = metricLabel("CAN: --");

    // Mini trend charts populated from replay data
    private final TrendChartPanel batteryChart;
    private final TrendChartPanel shooterChart;
    private final TrendChartPanel headingChart;
    private final TrendDataStore batteryStore = new TrendDataStore(3000);
    private final TrendDataStore shooterStore = new TrendDataStore(3000);
    private final TrendDataStore headingStore = new TrendDataStore(3000);

    public EventReplayPanel() {
        setLayout(new BorderLayout(8, 8));
        setBackground(BG);
        setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

        batteryChart = new TrendChartPanel("Battery (V)", OK, 11.5, WARN);
        batteryChart.setStore(batteryStore);
        batteryChart.setFixedYRange(10.0, 14.0);

        shooterChart = new TrendChartPanel("Shooter Left (RPS)", INFO);
        shooterChart.setStore(shooterStore);

        headingChart = new TrendChartPanel("Heading (deg)", new Color(140, 100, 255));
        headingChart.setStore(headingStore);

        playTimer = new Timer(100, e -> advanceCursor());
        playTimer.setRepeats(true);

        add(buildControlPanel(), BorderLayout.NORTH);
        add(buildCenterPanel(), BorderLayout.CENTER);
        add(buildStatusPanel(), BorderLayout.SOUTH);

        loadButton.addActionListener(e -> loadFile());
        playPauseButton.addActionListener(e -> togglePlay());
        rewindButton.addActionListener(e -> rewind());
        scrubSlider.addChangeListener(e -> {
            if (!scrubSlider.getValueIsAdjusting()) return;
            cursor = scrubSlider.getValue();
            displayCurrentRow();
        });
    }

    // ------------------------------------------------------------------
    // Layout builders
    // ------------------------------------------------------------------

    private JPanel buildControlPanel() {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.X_AXIS));
        panel.setBackground(CARD);
        panel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(BORDER_C, 1),
                BorderFactory.createEmptyBorder(8, 10, 8, 10)));
        panel.add(loadButton);
        panel.add(Box.createHorizontalStrut(8));
        panel.add(rewindButton);
        panel.add(Box.createHorizontalStrut(8));
        panel.add(playPauseButton);
        panel.add(Box.createHorizontalStrut(12));
        scrubSlider.setBackground(CARD);
        scrubSlider.setForeground(TEXT);
        panel.add(scrubSlider);
        return panel;
    }

    private JPanel buildCenterPanel() {
        JPanel panel = new JPanel(new BorderLayout(8, 8));
        panel.setBackground(BG);

        // Left side: current row data
        JPanel dataPanel = new JPanel();
        dataPanel.setLayout(new BoxLayout(dataPanel, BoxLayout.Y_AXIS));
        dataPanel.setBackground(CARD);
        dataPanel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(BORDER_C, 1),
                BorderFactory.createEmptyBorder(10, 10, 10, 10)));
        JLabel title = new JLabel("Current Row");
        title.setForeground(MUTED);
        title.setFont(TITLE_FONT);
        dataPanel.add(title);
        dataPanel.add(Box.createVerticalStrut(6));
        dataPanel.add(timestampLabel);
        dataPanel.add(batteryLabel);
        dataPanel.add(modeLabel);
        dataPanel.add(matchTimeLabel);
        dataPanel.add(shooterLabel);
        dataPanel.add(readyLabel);
        dataPanel.add(alignLabel);
        dataPanel.add(poseLabel);
        dataPanel.add(canLabel);
        dataPanel.setPreferredSize(new Dimension(360, 300));

        // Right side: charts
        JPanel chartPanel = new JPanel(new GridLayout(3, 1, 4, 4));
        chartPanel.setBackground(BG);
        chartPanel.add(wrapChart(batteryChart));
        chartPanel.add(wrapChart(shooterChart));
        chartPanel.add(wrapChart(headingChart));

        panel.add(dataPanel, BorderLayout.WEST);
        panel.add(chartPanel, BorderLayout.CENTER);
        return panel;
    }

    private JPanel buildStatusPanel() {
        JPanel panel = new JPanel(new GridLayout(1, 3, 10, 0));
        panel.setBackground(BG);
        panel.add(wrapStatusCard(fileLabel));
        panel.add(wrapStatusCard(rowCountLabel));
        panel.add(wrapStatusCard(cursorLabel));
        return panel;
    }

    // ------------------------------------------------------------------
    // Replay logic
    // ------------------------------------------------------------------

    private void loadFile() {
        JFileChooser chooser = new JFileChooser(".");
        chooser.setDialogTitle("Open match CSV");
        chooser.setFileFilter(new FileNameExtensionFilter("CSV files", "csv"));
        if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) return;
        Path path = chooser.getSelectedFile().toPath();
        try {
            rows = MatchCsvLogger.loadCsv(path);
            cursor = 0;
            playing = false;
            playTimer.stop();
            playPauseButton.setText("Play");

            scrubSlider.setMinimum(0);
            scrubSlider.setMaximum(Math.max(0, rows.size() - 1));
            scrubSlider.setValue(0);

            fileLabel.setText("File: " + path.getFileName());
            rowCountLabel.setText("Rows: " + rows.size());

            populateChartStores();
            displayCurrentRow();
        } catch (IOException ex) {
            fileLabel.setText("File: ERROR - " + ex.getMessage());
        }
    }

    // Package-private for testing
    void loadRows(List<MatchCsvLogger.CsvRow> rows, String fileName) {
        this.rows = rows;
        cursor = 0;
        playing = false;
        playTimer.stop();
        playPauseButton.setText("Play");

        scrubSlider.setMinimum(0);
        scrubSlider.setMaximum(Math.max(0, rows.size() - 1));
        scrubSlider.setValue(0);

        fileLabel.setText("File: " + fileName);
        rowCountLabel.setText("Rows: " + rows.size());

        populateChartStores();
        displayCurrentRow();
    }

    private void togglePlay() {
        if (rows == null || rows.isEmpty()) return;
        playing = !playing;
        if (playing) {
            playPauseButton.setText("Pause");
            playTimer.start();
        } else {
            playPauseButton.setText("Play");
            playTimer.stop();
        }
    }

    private void rewind() {
        cursor = 0;
        scrubSlider.setValue(0);
        displayCurrentRow();
    }

    private void advanceCursor() {
        if (rows == null || rows.isEmpty()) return;
        if (cursor < rows.size() - 1) {
            cursor++;
            scrubSlider.setValue(cursor);
            displayCurrentRow();
        } else {
            playing = false;
            playTimer.stop();
            playPauseButton.setText("Play");
        }
    }

    private void populateChartStores() {
        batteryStore.clear();
        shooterStore.clear();
        headingStore.clear();
        if (rows == null) return;
        for (MatchCsvLogger.CsvRow row : rows) {
            batteryStore.add(row.timestampSec(), row.batteryVoltage());
            shooterStore.add(row.timestampSec(), row.shooterLeftRps());
            headingStore.add(row.timestampSec(), row.headingDeg());
        }
    }

    private void displayCurrentRow() {
        if (rows == null || rows.isEmpty()) {
            cursorLabel.setText("Position: 0 / 0");
            return;
        }
        MatchCsvLogger.CsvRow row = rows.get(cursor);
        cursorLabel.setText("Position: " + (cursor + 1) + " / " + rows.size());
        timestampLabel.setText("Timestamp: " + ONE_D.format(row.timestampSec()) + "s");
        batteryLabel.setText("Battery: " + ONE_D.format(row.batteryVoltage()) + "V");
        batteryLabel.setForeground(row.batteryVoltage() < 11.5 ? WARN : TEXT);
        modeLabel.setText("Mode: " + row.mode() + (row.enabled() ? " EN" : " DIS"));
        matchTimeLabel.setText("Match time: " + ONE_D.format(row.matchTimeSec()) + "s");
        shooterLabel.setText("Shooter: L " + ONE_D.format(row.shooterLeftRps())
                + " / R " + ONE_D.format(row.shooterRightRps()) + " RPS");
        readyLabel.setText("Ready: " + (row.readyToScore() ? "YES" : "NO")
                + " (" + (row.readyReason().isEmpty() ? "--" : row.readyReason()) + ")");
        readyLabel.setForeground(row.readyToScore() ? OK : BAD);
        alignLabel.setText("Align: " + row.alignState()
                + " yaw=" + ONE_D.format(row.alignYawDeg())
                + " pitch=" + ONE_D.format(row.alignPitchDeg()));
        poseLabel.setText("Pose: (" + ONE_D.format(row.poseXm()) + ", "
                + ONE_D.format(row.poseYm()) + ") hdg " + ONE_D.format(row.headingDeg()));
        canLabel.setText("CAN: " + ONE_D.format(row.canBusUtilization() * 100.0) + "% util"
                + (row.brownoutAlert() ? " BROWNOUT" : ""));
        canLabel.setForeground(row.brownoutAlert() ? BAD : TEXT);

        // Repaint charts (they auto-read from stores)
        batteryChart.repaint();
        shooterChart.repaint();
        headingChart.repaint();
    }

    // ------------------------------------------------------------------
    // Widget helpers
    // ------------------------------------------------------------------

    private static JButton styledButton(String text) {
        JButton button = new JButton(text);
        button.setFocusPainted(false);
        button.setFont(ACTION_FONT);
        button.setBackground(new Color(48, 92, 142));
        button.setForeground(TEXT);
        button.setBorder(BorderFactory.createEmptyBorder(6, 12, 6, 12));
        return button;
    }

    private static JLabel metricLabel(String text) {
        JLabel label = new JLabel(text);
        label.setFont(METRIC_FONT);
        label.setForeground(TEXT);
        return label;
    }

    private static JPanel wrapChart(TrendChartPanel chart) {
        JPanel panel = new JPanel(new BorderLayout());
        panel.setBackground(CARD);
        panel.setBorder(BorderFactory.createLineBorder(BORDER_C, 1));
        panel.add(chart, BorderLayout.CENTER);
        return panel;
    }

    private static JPanel wrapStatusCard(JLabel label) {
        JPanel panel = new JPanel(new BorderLayout());
        panel.setBackground(CARD);
        panel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(BORDER_C, 1),
                BorderFactory.createEmptyBorder(6, 8, 6, 8)));
        panel.add(label, BorderLayout.CENTER);
        return panel;
    }
}
