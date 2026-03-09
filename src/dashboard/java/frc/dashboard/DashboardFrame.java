package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.RenderingHints;
import java.io.IOException;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayDeque;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.DefaultListCellRenderer;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JList;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.ScrollPaneConstants;
import javax.swing.JToggleButton;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;
import javax.swing.SwingConstants;
import javax.swing.Timer;
import javax.swing.UIManager;

import frc.robot.Constants;

public class DashboardFrame extends JFrame {

    private static final Color BG = new Color(11, 18, 28);
    private static final Color CARD = new Color(21, 31, 46);
    private static final Color CARD_ALT = new Color(14, 22, 34);
    private static final Color BORDER = new Color(55, 79, 107);
    private static final Color TEXT = new Color(243, 247, 252);
    private static final Color MUTED = new Color(166, 184, 206);
    private static final Color OK = new Color(46, 170, 88);
    private static final Color WARN = new Color(224, 157, 55);
    private static final Color BAD = new Color(202, 65, 65);
    private static final Color PENDING = new Color(150, 108, 49);
    private static final Color INFO = new Color(58, 118, 179);
    private static final Color BUTTON_ACTIVE = new Color(48, 92, 142);
    private static final Color BUTTON_DISABLED = new Color(61, 73, 89);

    private static final Font HEADER_FONT = new Font("Segoe UI", Font.BOLD, 17);
    private static final Font CARD_TITLE_FONT = new Font("Segoe UI", Font.BOLD, 14);
    private static final Font METRIC_FONT = new Font("Segoe UI", Font.PLAIN, 18);
    private static final Font ACTION_FONT = new Font("Segoe UI", Font.BOLD, 15);
    private static final Font READY_FONT = new Font("Segoe UI", Font.BOLD, 40);
    private static final Font MONO_FONT = new Font(Font.MONOSPACED, Font.PLAIN, 13);

    private static final DecimalFormat ONE_DECIMAL = new DecimalFormat("0.0");
    private static final DecimalFormat ZERO_DECIMAL = new DecimalFormat("0");
    private static final DecimalFormat ROTATION_DECIMAL = new DecimalFormat("0.000000");
    private static final DateTimeFormatter LOG_TIME_FORMAT = DateTimeFormatter.ofPattern("HH:mm:ss");
    private static final int MAX_EVENT_LINES = 70;
    private static final int MAX_CONTROL_EVENT_LINES = 140;

    // Thermal thresholds (TalonFX processor temp, Celsius)
    private static final double TEMP_WARN_C = 60.0;
    private static final double TEMP_CRITICAL_C = 80.0;

    private final DashboardNtClient client;

    // Header labels
    private final JLabel connectionLabel = new JLabel("Disconnected");
    private final JLabel modeLabel = new JLabel("Mode: UNKNOWN");
    private final JLabel allianceLabel = new JLabel("Alliance: UNKNOWN");
    private final JLabel matchTimeLabel = new JLabel("Match: --.-");
    private final JLabel phaseLabel = new JLabel("Phase: UNKNOWN");
    private final JLabel freshnessLabel = new JLabel("Telemetry: --");
    private final JLabel batteryLabel = new JLabel("Battery: --");
    private final JLabel matchInfoLabel = new JLabel("Match: --");

    // Driver tab: shot readiness
    private final JLabel readyLabel = new JLabel("NOT READY", SwingConstants.CENTER);
    private final JLabel readyReasonLabel = new JLabel("Reason: --");
    private final JLabel nextActionLabel = new JLabel("Action: --");
    private final JLabel hubActivityLabel = new JLabel("HUB: --");

    // Driver tab: auto selector & execution
    private final JLabel autoSelectorLabel = new JLabel("Auto: --");
    private final JLabel autoSourceLabel = new JLabel("Source: --");
    private final JLabel autoExecutionLabel = new JLabel("Status: Idle");
    private final JLabel autoChooserStatusLabel = new JLabel("Chooser: --");
    private JComboBox<String> autoChooserCombo;
    private JButton autoApplyButton;

    // Driver tab: pre-match checklist
    private final JLabel preMatchBatteryLabel = createChecklistLabel("Battery");
    private final JLabel preMatchCanLabel = createChecklistLabel("CAN Bus");
    private final JLabel preMatchImuLabel = createChecklistLabel("IMU data");
    private final JLabel preMatchCameraLabel = createChecklistLabel("Camera");
    private final JLabel preMatchIntakeLabel = createChecklistLabel("Intake homed");

    // Driver tab: shot checklist
    private final JLabel intakeChecklistLabel = createChecklistLabel("Intake homed");
    private final JLabel shooterChecklistLabel = createChecklistLabel("Shooter at speed");
    private final JLabel targetChecklistLabel = createChecklistLabel("Vision target");
    private final JLabel geometryChecklistLabel = createChecklistLabel("Shot geometry");
    private final JLabel yawChecklistLabel = createChecklistLabel("Yaw aligned");

    // Driver tab: align pipeline
    private final JLabel alignPhaseLabel = new JLabel("Align phase: IDLE");
    private final JLabel yawLabel = new JLabel("Yaw: --");
    private final JLabel pitchLabel = new JLabel("Pitch: --");
    private final JProgressBar yawBar = new JProgressBar(-30, 30);
    private final JLabel visionLabel = new JLabel("Vision: --");
    private final JLabel abortLabel = new JLabel("Last abort: --");
    private final FieldPanel fieldPanel = new FieldPanel();

    // Operator tab: subsystem metrics
    private final JLabel shooterLabel = new JLabel("Left -- / Right -- RPS");
    private final JLabel shooterAtSpeedLabel = new JLabel("At speed: NO");
    private final JLabel intakeLabel = new JLabel("Homed: NO  Limit: NO  Tilt: -- deg");
    private final JLabel conveyorLabel = new JLabel("Feeder -- A  Hopper -- A");
    // --- CLIMBER DISABLED ---
    private final JLabel climberLabel = new JLabel("DISABLED — no hardware");
    private final JLabel operatorVisionLabel = new JLabel("Target: NO  Feasible: NO");
    private final JLabel operatorPhaseLabel = new JLabel("Align phase: IDLE");
    private final JLabel operatorReadyLabel = new JLabel("NOT READY");
    private final JLabel operatorReadyReasonLabel = new JLabel("Reason: --");

    // Operator tab: swerve module angles
    private final JLabel swerveAnglesLabel = new JLabel("FL -- FR -- BL -- BR --");

    // Operator tab: system health (CAN + camera)
    private final JLabel canHealthLabel = new JLabel("CAN: --");
    private final JLabel cameraStatusLabel = new JLabel("Camera: --");
    private final JLabel cameraDebugLabel = new JLabel("Debug: --");
    private final JLabel cameraErrorLabel = new JLabel("Error: --");
    private final JLabel pigeonLabel = new JLabel("Pigeon: Y -- P -- R --");

    // Operator tab: motor temperatures
    private final JLabel driveTempLabel = new JLabel("Drive: FL -- FR -- BL -- BR --");
    private final JLabel shooterTempLabel = new JLabel("Shooter: L -- R --");

    // Command ack
    private final JLabel ackLabel = new JLabel("Ack: --");
    private final JTextArea eventLogArea = new JTextArea();
    private final ArrayDeque<String> eventLogLines = new ArrayDeque<>();

    // Bring-up / pit tabs
    private final JTextArea bringUpArea = new JTextArea();
    private final JTextArea pitRawArea = new JTextArea();
    private final VisionStreamPanel visionStreamPanel = new VisionStreamPanel();
    private final JLabel visionStreamStatusLabel = new JLabel("Stream: --");
    private final JLabel visionStreamSourceLabel = new JLabel("Source: --");
    private final JLabel visionStreamCameraLabel = new JLabel("Camera: --");
    private final JLabel visionStreamErrorLabel = new JLabel("Error: --");
    private JComboBox<String> visionStreamModeCombo;
    private JToggleButton visionStreamEnabledToggle;
    // Controls tab
    private final JLabel driverButtonsLabel = new JLabel("Driver buttons: --");
    private final JLabel operatorButtonsLabel = new JLabel("Operator buttons: --");
    private final JLabel controlLastEventLabel = new JLabel("Last event: --");
    private final JTextArea controlLogArea = new JTextArea();
    private final ArrayDeque<String> controlEventLines = new ArrayDeque<>();
    // Swerve tools tab
    private final JLabel swerveValidationStatusLabel = new JLabel("Validation: IDLE");
    private final JLabel swerveValidationOutputsLabel = new JLabel("Outputs: drive -- steer --");
    private final JLabel swerveValidationDeltasLabel = new JLabel("Deltas: angle -- cancoder --");
    private final JLabel swerveValidationLiveLabel = new JLabel("Selected: angle -- pos -- abs --");
    private final JLabel swerveCalibrationLabel = new JLabel("Calibration: raw -- offset --");
    private final JLabel swerveValidationHintLabel =
            new JLabel("Use steer test for steer/CANcoder signs, drive test for wheel spin.");
    private JComboBox<String> swerveModuleCombo;
    private JButton swerveSteerPositiveButton;
    private JButton swerveSteerNegativeButton;
    private JButton swerveDrivePositiveButton;
    private JButton swerveDriveNegativeButton;
    private JButton swerveStopValidationButton;
    private JButton swerveCalibrateButton;

    // Buttons
    private JButton zeroHeadingButton;
    private JButton stopDriveButton;
    private JButton alignShootButton;
    private JButton fallbackShootButton;
    private JButton intakeHomeButton;
    private JButton level1ClimbButton;

    // Trend charts (Trends tab)
    private static final int TREND_CAPACITY = 1800; // 3 minutes at 10 Hz
    private final TrendDataStore batteryTrend = new TrendDataStore(TREND_CAPACITY);
    private final TrendDataStore loopTimingTrend = new TrendDataStore(TREND_CAPACITY);
    private final TrendDataStore shooterSpinUpTrend = new TrendDataStore(TREND_CAPACITY);
    private final TrendChartPanel batteryChart =
            new TrendChartPanel("Battery Voltage (V)", OK, 11.5, WARN);
    private final TrendChartPanel loopTimingChart =
            new TrendChartPanel("Loop Period (ms)", INFO, 25.0, BAD);
    private final TrendChartPanel shooterSpinUpChart =
            new TrendChartPanel("Shooter Left RPS", new Color(140, 100, 255));

    // Event replay
    private final EventReplayPanel replayPanel = new EventReplayPanel();

    // CSV match logger
    private MatchCsvLogger csvLogger;
    private boolean csvLoggingEnabled;

    // State tracking
    private double lastRobotTimestampSec = Double.NaN;
    private long lastTimestampSeenNanos = System.nanoTime();
    private long lastAckSeqLogged = 0;
    private boolean connectionInitialized = false;
    private boolean lastConnected = false;
    private boolean readyInitialized = false;
    private boolean lastReady = false;
    private boolean controlEventsInitialized = false;
    private long lastControlEventSeqLogged = 0;
    private boolean autoSelectionEventsInitialized = false;
    private String lastSelectedAutoName = "";
    private String lastSelectedAutoSource = "";
    private boolean updatingAutoChooserModel = false;
    private String pendingAutoSelectionName;
    private String lastRobotSelectedAutoNameForChooser;

    public DashboardFrame(DashboardNtClient client) {
        super("3318 Competition Dashboard");
        this.client = client;

        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setMinimumSize(new Dimension(1440, 860));
        getContentPane().setBackground(BG);
        setLayout(new BorderLayout(10, 10));

        styleLogAreas();
        styleYawBar();

        add(buildHeader(), BorderLayout.NORTH);
        add(buildTabs(), BorderLayout.CENTER);

        Timer timer = new Timer(100, e -> refresh());
        timer.start();
    }

    // =========================================================================
    // HEADER — 8 cells across the top
    // =========================================================================
    private JPanel buildHeader() {
        JPanel header = new JPanel(new GridLayout(1, 8, 6, 6));
        header.setBorder(BorderFactory.createEmptyBorder(10, 10, 0, 10));
        header.setBackground(BG);

        styleHeaderLabel(connectionLabel);
        styleHeaderLabel(modeLabel);
        styleHeaderLabel(allianceLabel);
        styleHeaderLabel(matchTimeLabel);
        styleHeaderLabel(phaseLabel);
        styleHeaderLabel(batteryLabel);
        styleHeaderLabel(matchInfoLabel);
        styleHeaderLabel(freshnessLabel);

        header.add(connectionLabel);
        header.add(modeLabel);
        header.add(allianceLabel);
        header.add(matchTimeLabel);
        header.add(phaseLabel);
        header.add(batteryLabel);
        header.add(matchInfoLabel);
        header.add(freshnessLabel);
        return header;
    }

    // =========================================================================
    // TABS
    // =========================================================================
    private JTabbedPane buildTabs() {
        JTabbedPane tabs = new JTabbedPane();
        tabs.setBackground(CARD);
        tabs.setForeground(TEXT);
        tabs.addTab("Driver", buildDriverTab());
        tabs.addTab("Operator", buildOperatorTab());
        tabs.addTab("Controls", buildControlsTab());
        tabs.addTab("Swerve Tools", buildSwerveToolsTab());
        tabs.addTab("Vision", buildVisionTab());
        tabs.addTab("Trends", buildTrendsTab());
        tabs.addTab("Replay", replayPanel);
        tabs.addTab("Bring-up", buildBringUpTab());
        tabs.addTab("Pit", buildPitTab());
        return tabs;
    }

    // =========================================================================
    // DRIVER TAB
    // =========================================================================
    private JPanel buildDriverTab() {
        JPanel root = new JPanel(new BorderLayout(10, 10));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        fieldPanel.setPreferredSize(new Dimension(810, 620));
        root.add(fieldPanel, BorderLayout.CENTER);

        readyLabel.setFont(READY_FONT);
        readyLabel.setOpaque(true);
        readyLabel.setBackground(BAD);
        readyLabel.setForeground(Color.WHITE);
        readyLabel.setBorder(BorderFactory.createEmptyBorder(18, 8, 18, 8));

        styleMetricLabel(readyReasonLabel);
        styleMetricLabel(nextActionLabel);
        styleMetricLabel(hubActivityLabel);
        styleMetricLabel(autoSelectorLabel);
        styleMetricLabel(autoSourceLabel);
        styleMetricLabel(autoExecutionLabel);
        styleCompactLabel(autoChooserStatusLabel);
        styleCompactLabel(alignPhaseLabel);
        styleCompactLabel(yawLabel);
        styleCompactLabel(pitchLabel);
        styleCompactLabel(visionLabel);
        styleCompactLabel(abortLabel);
        styleMetricLabel(ackLabel);

        JPanel side = new JPanel();
        side.setLayout(new BoxLayout(side, BoxLayout.Y_AXIS));
        side.setBackground(BG);

        addSideCard(side, buildAutoSelectionCard());
        addSideCard(side, buildPreMatchChecklistCard());
        addSideCard(side, wrapLabelCard("Shot Readiness", readyLabel, readyReasonLabel, nextActionLabel, hubActivityLabel));
        addSideCard(side, buildChecklistCard());
        addSideCard(side, buildAlignCard());
        addSideCard(side, wrapLabelCard("Command Ack", ackLabel));
        addSideCard(side, buildDriverActionCard());
        addSideCard(side, buildOperatorActionCard());

        JScrollPane sideScroll = new JScrollPane(side);
        sideScroll.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
        sideScroll.setBorder(BorderFactory.createEmptyBorder());
        sideScroll.getViewport().setBackground(BG);
        sideScroll.setPreferredSize(new Dimension(430, 620));
        root.add(sideScroll, BorderLayout.EAST);
        return root;
    }

    // =========================================================================
    // OPERATOR TAB — 3x3 grid + event feed
    // =========================================================================
    private JPanel buildOperatorTab() {
        JPanel root = new JPanel(new BorderLayout(10, 10));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        styleMetricLabel(shooterLabel);
        styleMetricLabel(shooterAtSpeedLabel);
        styleMetricLabel(intakeLabel);
        styleMetricLabel(conveyorLabel);
        styleMetricLabel(climberLabel);
        styleMetricLabel(operatorVisionLabel);
        styleMetricLabel(operatorPhaseLabel);
        styleMetricLabel(operatorReadyLabel);
        styleMetricLabel(operatorReadyReasonLabel);
        styleMetricLabel(swerveAnglesLabel);
        styleMetricLabel(canHealthLabel);
        styleMetricLabel(cameraStatusLabel);
        styleCompactLabel(cameraDebugLabel);
        styleCompactLabel(cameraErrorLabel);
        styleMetricLabel(pigeonLabel);
        styleMetricLabel(driveTempLabel);
        styleMetricLabel(shooterTempLabel);

        JPanel top = new JPanel(new GridLayout(3, 3, 10, 10));
        top.setBackground(BG);
        // Row 1: subsystem status
        top.add(wrapLabelCard("Shooter", shooterLabel, shooterAtSpeedLabel));
        top.add(wrapLabelCard("Intake", intakeLabel));
        top.add(wrapLabelCard("Conveyor", conveyorLabel));
        // Row 2: more subsystems
        top.add(wrapLabelCard("Climber", climberLabel));
        top.add(wrapLabelCard("Vision", operatorVisionLabel, operatorPhaseLabel));
        top.add(wrapLabelCard("Readiness", operatorReadyLabel, operatorReadyReasonLabel));
        // Row 3: system health
        top.add(wrapLabelCard("Swerve Modules", swerveAnglesLabel));
        top.add(wrapLabelCard("System Health", canHealthLabel, cameraStatusLabel, cameraDebugLabel, cameraErrorLabel, pigeonLabel));
        top.add(wrapLabelCard("Motor Temps", driveTempLabel, shooterTempLabel));
        root.add(top, BorderLayout.CENTER);

        JScrollPane eventScroll = new JScrollPane(eventLogArea);
        eventScroll.setBorder(BorderFactory.createLineBorder(BORDER, 1));
        eventScroll.getViewport().setBackground(CARD_ALT);
        eventScroll.setPreferredSize(new Dimension(100, 160));
        root.add(wrapCard("Event Feed", eventScroll), BorderLayout.SOUTH);
        return root;
    }

    // =========================================================================
    // CONTROLS TAB
    // =========================================================================
    private JPanel buildControlsTab() {
        JPanel root = new JPanel(new BorderLayout(10, 10));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        styleMetricLabel(driverButtonsLabel);
        styleMetricLabel(operatorButtonsLabel);
        styleMetricLabel(controlLastEventLabel);

        JPanel top = new JPanel(new GridLayout(1, 3, 10, 10));
        top.setBackground(BG);
        top.add(wrapLabelCard("Driver Buttons", driverButtonsLabel));
        top.add(wrapLabelCard("Operator Buttons", operatorButtonsLabel));
        top.add(wrapLabelCard("Last Trigger", controlLastEventLabel));
        root.add(top, BorderLayout.NORTH);

        JScrollPane controlLogScroll = new JScrollPane(controlLogArea);
        controlLogScroll.setBorder(BorderFactory.createLineBorder(BORDER, 1));
        controlLogScroll.getViewport().setBackground(CARD_ALT);
        root.add(wrapCard("Control Trigger Feed", controlLogScroll), BorderLayout.CENTER);
        return root;
    }

    // =========================================================================
    // VISION TAB
    // =========================================================================
    private JPanel buildVisionTab() {
        JPanel root = new JPanel(new BorderLayout(10, 10));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        styleMetricLabel(visionStreamStatusLabel);
        styleMetricLabel(visionStreamSourceLabel);
        styleMetricLabel(visionStreamCameraLabel);
        styleMetricLabel(visionStreamErrorLabel);

        JPanel controls = new JPanel();
        controls.setLayout(new BoxLayout(controls, BoxLayout.X_AXIS));
        controls.setBackground(CARD);

        JLabel sourceLabel = infoLabel("View");
        visionStreamModeCombo = new JComboBox<>(new String[] {"Overlay", "Raw"});
        visionStreamModeCombo.setFont(ACTION_FONT);
        visionStreamModeCombo.setBackground(CARD_ALT);
        visionStreamModeCombo.setForeground(TEXT);
        visionStreamModeCombo.setFocusable(false);
        visionStreamModeCombo.setRenderer(createDarkComboRenderer());
        visionStreamModeCombo.addActionListener(e -> updateVisionStreamSelection());

        visionStreamEnabledToggle = new JToggleButton("Live");
        visionStreamEnabledToggle.setSelected(true);
        visionStreamEnabledToggle.setFocusPainted(false);
        visionStreamEnabledToggle.setFont(ACTION_FONT);
        visionStreamEnabledToggle.setBackground(BUTTON_ACTIVE);
        visionStreamEnabledToggle.setForeground(TEXT);
        visionStreamEnabledToggle.addActionListener(e -> {
            visionStreamEnabledToggle.setText(visionStreamEnabledToggle.isSelected() ? "Live" : "Paused");
            visionStreamPanel.setStreamingEnabled(visionStreamEnabledToggle.isSelected());
            updateVisionStreamSelection();
        });

        controls.add(sourceLabel);
        controls.add(Box.createHorizontalStrut(8));
        controls.add(visionStreamModeCombo);
        controls.add(Box.createHorizontalStrut(10));
        controls.add(visionStreamEnabledToggle);
        controls.add(Box.createHorizontalGlue());
        controls.add(infoLabel("Overlay uses roboRIO AprilTag detections"));

        root.add(wrapCard("Vision Stream Controls", controls), BorderLayout.NORTH);
        root.add(wrapCard("Live Camera", visionStreamPanel), BorderLayout.CENTER);
        root.add(wrapLabelCard(
                "Vision Stream Status",
                visionStreamStatusLabel,
                visionStreamSourceLabel,
                visionStreamCameraLabel,
                visionStreamErrorLabel), BorderLayout.SOUTH);

        updateVisionStreamSelection();
        return root;
    }

    // =========================================================================
    // SWERVE TOOLS TAB
    // =========================================================================
    private JPanel buildSwerveToolsTab() {
        JPanel root = new JPanel(new BorderLayout(10, 10));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        styleMetricLabel(swerveValidationStatusLabel);
        styleMetricLabel(swerveValidationOutputsLabel);
        styleMetricLabel(swerveValidationDeltasLabel);
        styleMetricLabel(swerveValidationLiveLabel);
        styleMetricLabel(swerveCalibrationLabel);
        styleMetricLabel(swerveValidationHintLabel);

        root.add(buildSwerveToolsControlsCard(), BorderLayout.NORTH);

        JPanel center = new JPanel(new GridLayout(2, 2, 10, 10));
        center.setBackground(BG);
        center.add(wrapLabelCard("Validation Status",
                swerveValidationStatusLabel,
                swerveValidationOutputsLabel,
                swerveValidationDeltasLabel));
        center.add(wrapLabelCard("Selected Module Live",
                swerveValidationLiveLabel,
                swerveCalibrationLabel));
        center.add(wrapLabelCard("How To Use",
                swerveValidationHintLabel,
                infoLabel("1. Put robot on blocks and choose one module."),
                infoLabel("2. Steer +/- checks steer invert and CANcoder direction."),
                infoLabel("3. Drive +/- checks drive invert.")));
        center.add(wrapLabelCard("Notes",
                infoLabel("Angle delta should match physical module motion."),
                infoLabel("CANcoder delta should have the same sign as steer motion."),
                infoLabel("Run calibration with wheels straight before updating offsets.")));
        root.add(center, BorderLayout.CENTER);
        return root;
    }

    // =========================================================================
    // TRENDS TAB
    // =========================================================================
    private JPanel buildTrendsTab() {
        JPanel root = new JPanel(new BorderLayout(8, 8));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        batteryChart.setStore(batteryTrend);
        batteryChart.setFixedYRange(10.0, 14.0);
        loopTimingChart.setStore(loopTimingTrend);
        loopTimingChart.setFixedYRange(0.0, 40.0);
        shooterSpinUpChart.setStore(shooterSpinUpTrend);

        JPanel chartsGrid = new JPanel(new GridLayout(3, 1, 8, 8));
        chartsGrid.setBackground(BG);
        chartsGrid.add(wrapCard("Battery Voltage", batteryChart));
        chartsGrid.add(wrapCard("Loop Timing", loopTimingChart));
        chartsGrid.add(wrapCard("Shooter Spin-Up", shooterSpinUpChart));
        root.add(chartsGrid, BorderLayout.CENTER);

        JPanel csvPanel = new JPanel();
        csvPanel.setLayout(new BoxLayout(csvPanel, BoxLayout.X_AXIS));
        csvPanel.setBackground(CARD);
        csvPanel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(BORDER, 1),
                BorderFactory.createEmptyBorder(6, 10, 6, 10)));
        JButton csvToggle = new JButton("Start CSV Log");
        csvToggle.setFocusPainted(false);
        csvToggle.setFont(ACTION_FONT);
        csvToggle.setBackground(BUTTON_ACTIVE);
        csvToggle.setForeground(TEXT);
        csvToggle.setBorder(BorderFactory.createEmptyBorder(6, 12, 6, 12));
        JLabel csvStatusLabel = new JLabel("CSV: idle");
        csvStatusLabel.setForeground(MUTED);
        csvStatusLabel.setFont(new Font("Segoe UI", Font.PLAIN, 14));
        csvToggle.addActionListener(e -> {
            if (csvLoggingEnabled) {
                stopCsvLogger();
                csvToggle.setText("Start CSV Log");
                csvStatusLabel.setText("CSV: stopped");
            } else {
                startCsvLogger();
                csvToggle.setText("Stop CSV Log");
                if (csvLogger != null) {
                    csvStatusLabel.setText("CSV: " + csvLogger.getPath().getFileName());
                }
            }
        });
        csvPanel.add(csvToggle);
        csvPanel.add(Box.createHorizontalStrut(12));
        csvPanel.add(csvStatusLabel);
        root.add(csvPanel, BorderLayout.SOUTH);

        return root;
    }

    private void startCsvLogger() {
        try {
            String filename = "match_" + DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")
                    .format(LocalDateTime.now()) + ".csv";
            csvLogger = new MatchCsvLogger(Path.of(filename));
            csvLoggingEnabled = true;
        } catch (IOException ex) {
            System.err.println("Failed to start CSV logger: " + ex.getMessage());
            csvLoggingEnabled = false;
        }
    }

    private void stopCsvLogger() {
        csvLoggingEnabled = false;
        if (csvLogger != null) {
            try {
                csvLogger.close();
            } catch (IOException ex) {
                System.err.println("Failed to close CSV logger: " + ex.getMessage());
            }
            csvLogger = null;
        }
    }

    // =========================================================================
    // BRING-UP TAB
    // =========================================================================
    private JPanel buildBringUpTab() {
        JPanel root = new JPanel(new BorderLayout(8, 8));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        JScrollPane bringUpScroll = new JScrollPane(bringUpArea);
        bringUpScroll.setBorder(BorderFactory.createLineBorder(BORDER, 1));
        bringUpScroll.getViewport().setBackground(CARD_ALT);
        root.add(bringUpScroll, BorderLayout.CENTER);
        return root;
    }

    // =========================================================================
    // PIT TAB
    // =========================================================================
    private JPanel buildPitTab() {
        JPanel root = new JPanel(new BorderLayout(8, 8));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        JScrollPane pitScroll = new JScrollPane(pitRawArea);
        pitScroll.setBorder(BorderFactory.createLineBorder(BORDER, 1));
        pitScroll.getViewport().setBackground(CARD_ALT);
        root.add(pitScroll, BorderLayout.CENTER);
        return root;
    }

    // =========================================================================
    // CARD BUILDERS
    // =========================================================================
    private JPanel buildPreMatchChecklistCard() {
        JPanel panel = new JPanel(new GridLayout(5, 1, 4, 4));
        panel.setBackground(CARD);
        panel.add(preMatchBatteryLabel);
        panel.add(preMatchCanLabel);
        panel.add(preMatchImuLabel);
        panel.add(preMatchCameraLabel);
        panel.add(preMatchIntakeLabel);
        return wrapCard("Pre-Match Checklist", panel);
    }

    private JPanel buildChecklistCard() {
        JPanel panel = new JPanel(new GridLayout(5, 1, 4, 4));
        panel.setBackground(CARD);
        panel.add(intakeChecklistLabel);
        panel.add(shooterChecklistLabel);
        panel.add(targetChecklistLabel);
        panel.add(geometryChecklistLabel);
        panel.add(yawChecklistLabel);
        return wrapCard("Shot Checklist", panel);
    }

    private JPanel buildAlignCard() {
        JPanel panel = new JPanel(new GridLayout(6, 1, 4, 4));
        panel.setBackground(CARD);
        panel.add(alignPhaseLabel);
        panel.add(yawLabel);
        panel.add(yawBar);
        panel.add(pitchLabel);
        panel.add(visionLabel);
        panel.add(abortLabel);
        return wrapCard("Align Pipeline", panel);
    }

    private JPanel buildAutoSelectionCard() {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.setBackground(CARD);

        autoChooserCombo = new JComboBox<>();
        autoChooserCombo.setFont(ACTION_FONT);
        autoChooserCombo.setBackground(CARD_ALT);
        autoChooserCombo.setForeground(TEXT);
        autoChooserCombo.setFocusable(false);
        autoChooserCombo.setRenderer(createDarkComboRenderer());
        autoChooserCombo.addActionListener(e -> {
            if (updatingAutoChooserModel) {
                return;
            }
            Object selectedItem = autoChooserCombo.getSelectedItem();
            pendingAutoSelectionName = selectedItem == null ? null : selectedItem.toString();
        });

        autoApplyButton = new JButton("Apply");
        autoApplyButton.setFocusPainted(false);
        autoApplyButton.setFont(ACTION_FONT);
        autoApplyButton.setBackground(BUTTON_ACTIVE);
        autoApplyButton.setForeground(TEXT);
        autoApplyButton.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
        autoApplyButton.addActionListener(e -> {
            if (pendingAutoSelectionName == null || pendingAutoSelectionName.isBlank()) {
                return;
            }
            client.sendAutoSelection(pendingAutoSelectionName);
        });

        JPanel controls = new JPanel(new GridLayout(1, 2, 8, 0));
        controls.setBackground(CARD);
        controls.add(autoChooserCombo);
        controls.add(autoApplyButton);

        panel.add(autoSelectorLabel);
        panel.add(Box.createVerticalStrut(4));
        panel.add(autoSourceLabel);
        panel.add(Box.createVerticalStrut(4));
        panel.add(autoExecutionLabel);
        panel.add(Box.createVerticalStrut(4));
        panel.add(autoChooserStatusLabel);
        panel.add(Box.createVerticalStrut(10));
        panel.add(controls);

        return wrapCard("Auto Selection", panel);
    }

    private JPanel buildDriverActionCard() {
        JPanel panel = new JPanel(new GridLayout(2, 2, 8, 8));
        panel.setBackground(CARD);

        zeroHeadingButton = createCommandButton("Zero Heading", DashboardNtClient.DashboardCommand.ZERO_HEADING);
        stopDriveButton = createCommandButton("Stop Drive", DashboardNtClient.DashboardCommand.STOP_DRIVE);
        alignShootButton = createCommandButton("Align + Shoot", DashboardNtClient.DashboardCommand.ALIGN_SHOOT);
        fallbackShootButton = createCommandButton("Fallback Shot", DashboardNtClient.DashboardCommand.FALLBACK_SHOOT);

        panel.add(zeroHeadingButton);
        panel.add(stopDriveButton);
        panel.add(alignShootButton);
        panel.add(fallbackShootButton);

        return wrapCard("Driver Actions", panel);
    }

    private JPanel buildOperatorActionCard() {
        JPanel panel = new JPanel(new GridLayout(1, 2, 8, 8));
        panel.setBackground(CARD);

        intakeHomeButton = createCommandButton("Intake Home", DashboardNtClient.DashboardCommand.INTAKE_HOME);
        level1ClimbButton = createCommandButton("Level 1 Climb", DashboardNtClient.DashboardCommand.LEVEL1_CLIMB);

        panel.add(intakeHomeButton);
        panel.add(level1ClimbButton);
        return wrapCard("Operator Actions", panel);
    }

    private JPanel buildSwerveToolsControlsCard() {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.setBackground(CARD);

        swerveModuleCombo = new JComboBox<>(new String[] {"FL", "FR", "BL", "BR"});
        swerveModuleCombo.setFont(ACTION_FONT);
        swerveModuleCombo.setBackground(CARD_ALT);
        swerveModuleCombo.setForeground(TEXT);
        swerveModuleCombo.setFocusable(false);
        swerveModuleCombo.setRenderer(createDarkComboRenderer());

        swerveCalibrateButton = createCommandButton(
                "Read Calibration",
                DashboardNtClient.DashboardCommand.CALIBRATE_CANCODERS);
        swerveStopValidationButton = createCommandButton(
                "Stop Validation",
                DashboardNtClient.DashboardCommand.STOP_SWERVE_VALIDATION);

        JPanel topRow = new JPanel(new GridLayout(1, 3, 8, 0));
        topRow.setBackground(CARD);
        topRow.add(swerveModuleCombo);
        topRow.add(swerveCalibrateButton);
        topRow.add(swerveStopValidationButton);

        swerveSteerPositiveButton = createSwerveValidationButton("Steer +", "STEER_POSITIVE");
        swerveSteerNegativeButton = createSwerveValidationButton("Steer -", "STEER_NEGATIVE");
        swerveDrivePositiveButton = createSwerveValidationButton("Drive +", "DRIVE_FORWARD");
        swerveDriveNegativeButton = createSwerveValidationButton("Drive -", "DRIVE_REVERSE");

        JPanel actionGrid = new JPanel(new GridLayout(1, 4, 8, 8));
        actionGrid.setBackground(CARD);
        actionGrid.add(swerveSteerPositiveButton);
        actionGrid.add(swerveSteerNegativeButton);
        actionGrid.add(swerveDrivePositiveButton);
        actionGrid.add(swerveDriveNegativeButton);

        panel.add(topRow);
        panel.add(Box.createVerticalStrut(10));
        panel.add(actionGrid);
        return wrapCard("Swerve Validation Controls", panel);
    }

    private JButton createCommandButton(String text, DashboardNtClient.DashboardCommand command) {
        JButton button = new JButton(text);
        button.setFocusPainted(false);
        button.setFont(ACTION_FONT);
        button.setBackground(BUTTON_ACTIVE);
        button.setForeground(TEXT);
        button.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
        button.addActionListener(e -> client.sendCommand(command));
        return button;
    }

    private JButton createSwerveValidationButton(String text, String modeToken) {
        JButton button = new JButton(text);
        button.setFocusPainted(false);
        button.setFont(ACTION_FONT);
        button.setBackground(BUTTON_ACTIVE);
        button.setForeground(TEXT);
        button.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
        button.addActionListener(e -> client.sendSwerveValidation(selectedSwerveModuleToken(), modeToken));
        return button;
    }

    private JLabel infoLabel(String text) {
        JLabel label = new JLabel(text);
        styleCompactLabel(label);
        return label;
    }

    private DefaultListCellRenderer createDarkComboRenderer() {
        return new DefaultListCellRenderer() {
            @Override
            public Component getListCellRendererComponent(
                    JList<?> list, Object value, int index, boolean isSelected, boolean cellHasFocus) {
                super.getListCellRendererComponent(list, value, index, isSelected, cellHasFocus);
                if (isSelected) {
                    setBackground(BUTTON_ACTIVE);
                    setForeground(TEXT);
                } else {
                    setBackground(CARD_ALT);
                    setForeground(TEXT);
                }
                return this;
            }
        };
    }

    // =========================================================================
    // CARD WRAPPERS
    // =========================================================================
    private JPanel wrapLabelCard(String title, JLabel... labels) {
        JPanel panel = new JPanel(new GridBagLayout());
        panel.setBackground(CARD);
        panel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(BORDER, 1),
                BorderFactory.createEmptyBorder(10, 10, 10, 10)));

        GridBagConstraints gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.insets = new Insets(0, 0, 6, 0);

        JLabel titleLabel = new JLabel(title);
        titleLabel.setForeground(MUTED);
        titleLabel.setFont(CARD_TITLE_FONT);
        panel.add(titleLabel, gbc);

        for (JLabel label : labels) {
            gbc.gridy++;
            panel.add(label, gbc);
        }
        return panel;
    }

    private JPanel wrapCard(String title, JComponent content) {
        JPanel panel = new JPanel(new BorderLayout(6, 6));
        panel.setBackground(CARD);
        panel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(BORDER, 1),
                BorderFactory.createEmptyBorder(10, 10, 10, 10)));

        JLabel titleLabel = new JLabel(title);
        titleLabel.setForeground(MUTED);
        titleLabel.setFont(CARD_TITLE_FONT);
        panel.add(titleLabel, BorderLayout.NORTH);
        panel.add(content, BorderLayout.CENTER);
        return panel;
    }

    private void addSideCard(JPanel container, JPanel card) {
        card.setAlignmentX(JComponent.LEFT_ALIGNMENT);
        card.setMaximumSize(new Dimension(Integer.MAX_VALUE, card.getPreferredSize().height));
        container.add(card);
        container.add(Box.createVerticalStrut(8));
    }

    // =========================================================================
    // STYLING
    // =========================================================================
    private void styleHeaderLabel(JLabel label) {
        label.setOpaque(true);
        label.setBackground(CARD);
        label.setForeground(TEXT);
        label.setFont(HEADER_FONT);
        label.setHorizontalAlignment(SwingConstants.CENTER);
        label.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
    }

    private void styleMetricLabel(JLabel label) {
        label.setForeground(TEXT);
        label.setFont(METRIC_FONT);
    }

    private void styleCompactLabel(JLabel label) {
        label.setForeground(TEXT);
        label.setFont(new Font("Segoe UI", Font.PLAIN, 15));
    }

    private void styleYawBar() {
        yawBar.setValue(0);
        yawBar.setStringPainted(true);
        yawBar.setString("Yaw error: --");
        yawBar.setBackground(CARD_ALT);
        yawBar.setForeground(INFO);
        yawBar.setBorder(BorderFactory.createLineBorder(BORDER, 1));
    }

    private void styleLogAreas() {
        eventLogArea.setEditable(false);
        eventLogArea.setFont(MONO_FONT);
        eventLogArea.setBackground(CARD_ALT);
        eventLogArea.setForeground(TEXT);
        eventLogArea.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

        controlLogArea.setEditable(false);
        controlLogArea.setFont(MONO_FONT);
        controlLogArea.setBackground(CARD_ALT);
        controlLogArea.setForeground(TEXT);
        controlLogArea.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

        bringUpArea.setEditable(false);
        bringUpArea.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 14));
        bringUpArea.setBackground(CARD_ALT);
        bringUpArea.setForeground(TEXT);
        bringUpArea.setLineWrap(true);
        bringUpArea.setWrapStyleWord(true);
        bringUpArea.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

        pitRawArea.setEditable(false);
        pitRawArea.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 14));
        pitRawArea.setBackground(CARD_ALT);
        pitRawArea.setForeground(TEXT);
        pitRawArea.setLineWrap(true);
        pitRawArea.setWrapStyleWord(true);
        pitRawArea.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
    }

    // =========================================================================
    // REFRESH — called every 100ms
    // =========================================================================
    private void refresh() {
        DashboardData data = client.read();
        long nowNanos = System.nanoTime();

        // Feed trend stores
        double ts = data.robotTimestampSec();
        if (data.connected() && Double.isFinite(ts) && ts > 0.0) {
            batteryTrend.add(ts, data.batteryVoltage());
            shooterSpinUpTrend.add(ts, data.shooterLeftRps());
            // Loop period estimated from successive robot timestamps
            if (Double.isFinite(lastRobotTimestampSec) && ts > lastRobotTimestampSec) {
                double periodMs = (ts - lastRobotTimestampSec) * 1000.0;
                loopTimingTrend.add(ts, periodMs);
            }
        }
        batteryChart.repaint();
        loopTimingChart.repaint();
        shooterSpinUpChart.repaint();

        // CSV logging
        if (csvLoggingEnabled && csvLogger != null && data.connected()) {
            try {
                csvLogger.log(data);
            } catch (IOException ex) {
                System.err.println("CSV log error: " + ex.getMessage());
                stopCsvLogger();
            }
        }

        // Header
        connectionLabel.setText(data.connected() ? "Connected" : "Disconnected");
        connectionLabel.setBackground(data.connected() ? OK : BAD);

        modeLabel.setText("Mode: " + data.mode() + (data.enabled() ? " EN" : " DIS"));
        allianceLabel.setText("Alliance: " + data.alliance());

        updateMatchState(data);
        updateTelemetryFreshness(data, nowNanos);
        updateBatteryLabel(data);
        updateMatchInfoLabel(data);

        // Auto selector & execution (Driver tab)
        updateAutoSelectionCard(data);

        // Pre-match checklist
        updateChecklist(preMatchBatteryLabel, "Battery",
                data.batteryVoltage() >= 12.0);
        updateChecklist(preMatchCanLabel, "CAN Bus",
                data.canReceiveErrorCount() == 0 && data.canTransmitErrorCount() == 0);
        updateChecklist(preMatchImuLabel, "IMU data",
                Double.isFinite(data.pigeonYawDeg())
                        && Double.isFinite(data.pigeonPitchDeg())
                        && Double.isFinite(data.pigeonRollDeg()));
        updateChecklist(preMatchCameraLabel, "Camera",
                data.cameraConnected());
        updateChecklist(preMatchIntakeLabel, "Intake homed",
                data.intakeHomed());

        // Shot readiness
        readyLabel.setText(data.readyToScore() ? "READY" : "NOT READY");
        readyLabel.setBackground(data.readyToScore() ? OK : BAD);
        readyReasonLabel.setText("Reason: " + sanitize(data.readyReason()));
        nextActionLabel.setText("Action: " + nextAction(data));
        if (data.hubActive()) {
            hubActivityLabel.setText("HUB: ACTIVE");
            hubActivityLabel.setForeground(OK);
        } else {
            hubActivityLabel.setText(String.format("HUB: INACTIVE (%.0fs to shift)", data.hubSecondsToNextShift()));
            hubActivityLabel.setForeground(BAD);
        }

        // Align pipeline
        alignPhaseLabel.setText("Align phase: " + data.alignState() + (data.alignCommandActive() ? " (ACTIVE)" : ""));
        yawLabel.setText("Yaw: " + formatMaybe(data.alignYawDeg()) + " deg");
        pitchLabel.setText("Pitch: " + formatMaybe(data.alignPitchDeg()) + " deg");
        visionLabel.setText("Vision: " + yesNo(data.alignHasTarget()) + "  Feasible: " + yesNo(data.alignGeometryFeasible()));
        abortLabel.setText("Last abort: " + sanitize(data.alignAbortReason()));
        updateYawBar(data.alignYawDeg());

        // Shot checklist
        updateChecklist(intakeChecklistLabel, "Intake homed", data.intakeHomed());
        updateChecklist(shooterChecklistLabel, "Shooter at speed", data.shooterAtSpeed());
        updateChecklist(targetChecklistLabel, "Vision target", data.alignHasTarget());
        updateChecklist(geometryChecklistLabel, "Shot geometry", data.alignGeometryFeasible());
        boolean yawAligned = Double.isFinite(data.alignYawDeg())
                && Math.abs(data.alignYawDeg()) <= Constants.Vision.YAW_TOLERANCE_DEG;
        updateChecklist(yawChecklistLabel, "Yaw aligned", yawAligned);

        // Operator tab: subsystem metrics
        shooterLabel.setText("Left " + ONE_DECIMAL.format(data.shooterLeftRps())
                + " / Right " + ONE_DECIMAL.format(data.shooterRightRps()) + " RPS");
        shooterAtSpeedLabel.setText("At speed: " + yesNo(data.shooterAtSpeed()));
        intakeLabel.setText("Homed: " + yesNo(data.intakeHomed())
                + "  Limit: " + yesNo(data.intakeLimitSwitchPressed())
                + "  Tilt: " + ONE_DECIMAL.format(data.intakeTiltDeg()) + " deg");
        conveyorLabel.setText("Feeder " + ONE_DECIMAL.format(data.feederCurrentAmps())
                + " A  Hopper " + ONE_DECIMAL.format(data.hopperCurrentAmps()) + " A");
        // --- CLIMBER DISABLED ---
        climberLabel.setText("DISABLED — no hardware");
        // climberLabel.setText("Armed: " + yesNo(data.climberArmed())
        //         + "  Pos: " + ONE_DECIMAL.format(data.climberPositionRot())
        //         + " rot  I: " + ONE_DECIMAL.format(data.climberCurrentAmps()) + " A");

        operatorVisionLabel.setText("Target: " + yesNo(data.alignHasTarget())
                + "  Feasible: " + yesNo(data.alignGeometryFeasible()));
        operatorPhaseLabel.setText("Align phase: " + data.alignState());
        operatorReadyLabel.setText(data.readyToScore() ? "READY" : "NOT READY");
        operatorReadyLabel.setForeground(data.readyToScore() ? OK : BAD);
        operatorReadyReasonLabel.setText("Reason: " + sanitize(data.readyReason()));

        // Operator tab: swerve module angles
        swerveAnglesLabel.setText("FL " + ONE_DECIMAL.format(data.swerveFLAngleDeg())
                + "  FR " + ONE_DECIMAL.format(data.swerveFRAngleDeg())
                + "  BL " + ONE_DECIMAL.format(data.swerveBLAngleDeg())
                + "  BR " + ONE_DECIMAL.format(data.swerveBRAngleDeg()) + " deg");

        // Operator tab: system health
        int canErrors = (int) (data.canReceiveErrorCount() + data.canTransmitErrorCount());
        canHealthLabel.setText("CAN: " + ZERO_DECIMAL.format(data.canBusUtilization() * 100.0)
                + "% util, " + canErrors + " errors");
        canHealthLabel.setForeground(canErrors > 0 ? BAD : OK);

        cameraStatusLabel.setText("Camera: " + (data.cameraConnected() ? "CONNECTED" : "DISCONNECTED"));
        cameraStatusLabel.setForeground(data.cameraConnected() ? OK : BAD);
        cameraDebugLabel.setText("Debug: " + buildCameraDebugSummary(data));
        cameraDebugLabel.setForeground(data.cameraConnected() ? TEXT : WARN);
        cameraErrorLabel.setText("Error: " + sanitize(nonBlankOr(data.cameraLastError(), "none")));
        cameraErrorLabel.setForeground(isBlank(data.cameraLastError()) ? MUTED : WARN);
        pigeonLabel.setText("Pigeon: Y " + formatMaybe(data.pigeonYawDeg())
                + "  P " + formatMaybe(data.pigeonPitchDeg())
                + "  R " + formatMaybe(data.pigeonRollDeg()) + " deg");
        pigeonLabel.setForeground(
                Double.isFinite(data.pigeonYawDeg())
                        && Double.isFinite(data.pigeonPitchDeg())
                        && Double.isFinite(data.pigeonRollDeg()) ? TEXT : WARN);

        // Operator tab: motor temperatures
        updateTemperatureLabels(data);

        updateSwerveTools(data);

        // Command ack
        ackLabel.setText("Ack: " + sanitize(data.ackLastCommand())
                + " #" + data.ackSeq()
                + "  " + sanitize(data.ackStatus())
                + "  " + sanitize(data.ackMessage()));
        ackLabel.setForeground("OK".equals(data.ackStatus()) ? OK : WARN);

        // Controls tab: active buttons + trigger events
        driverButtonsLabel.setText("Driver buttons: " + sanitize(data.driverButtonsActive()));
        operatorButtonsLabel.setText("Operator buttons: " + sanitize(data.operatorButtonsActive()));
        controlLastEventLabel.setText("Last event #" + data.controlEventSeq()
                + " @ " + formatMaybe(data.controlEventTimestampSec())
                + "s  " + sanitize(data.controlEventMessage()));
        updateControlEventFeed(data);

        updateCommandAvailability(data);
        updateEventFeed(data);

        fieldPanel.update(data);
        long telemetryAgeMs = Math.max(0L, (nowNanos - lastTimestampSeenNanos) / 1_000_000L);
        bringUpArea.setText(buildBringUpText(data, telemetryAgeMs));
        bringUpArea.setCaretPosition(0);
        pitRawArea.setText(buildRawText(data));
        updateVisionStreamStatus(data);
    }

    // =========================================================================
    // HEADER UPDATERS
    // =========================================================================
    private void updateBatteryLabel(DashboardData data) {
        double v = data.batteryVoltage();
        batteryLabel.setText("Bat: " + ONE_DECIMAL.format(v) + "V");
        if (data.isBrownout()) {
            batteryLabel.setBackground(BAD);
        } else if (data.brownoutAlert()) {
            batteryLabel.setBackground(BAD);
        } else if (v < 11.5) {
            batteryLabel.setBackground(WARN);
        } else {
            batteryLabel.setBackground(OK);
        }
    }

    private void updateMatchInfoLabel(DashboardData data) {
        String event = sanitize(data.eventName());
        long num = data.matchNumber();
        if (num > 0) {
            matchInfoLabel.setText("Match " + num);
        } else if (!"--".equals(event)) {
            matchInfoLabel.setText(event);
        } else {
            matchInfoLabel.setText("Practice");
        }
    }

    private void updateVisionStreamStatus(DashboardData data) {
        visionStreamStatusLabel.setText("Stream: " + visionStreamPanel.getStatusText());
        visionStreamSourceLabel.setText("Source: " + buildVisionStreamSourceLabel(data));
        visionStreamCameraLabel.setText("Camera: " + buildCameraDebugSummary(data));
        visionStreamErrorLabel.setText("Error: " + sanitize(nonBlankOr(data.cameraLastError(), "none")));
        visionStreamErrorLabel.setForeground(isBlank(data.cameraLastError()) ? MUTED : WARN);
    }

    private void updateVisionStreamSelection() {
        if (visionStreamModeCombo == null) {
            return;
        }
        boolean overlay = "Overlay".equals(visionStreamModeCombo.getSelectedItem());
        String path = overlay
                ? "http://" + client.getRobotHost() + ":" + Constants.Vision.CAMERA_OVERLAY_STREAM_PORT + "/?action=stream"
                : "http://" + client.getRobotHost() + ":" + Constants.Vision.CAMERA_RAW_STREAM_PORT + "/?action=stream";
        visionStreamPanel.setStreamUrl(path);
    }

    private String buildVisionStreamSourceLabel(DashboardData data) {
        String mode = visionStreamModeCombo == null ? "Overlay" : String.valueOf(visionStreamModeCombo.getSelectedItem());
        return mode + " host=" + client.getRobotHost()
                + " status=" + sanitize(data.cameraStatus())
                + " tag=" + data.visionTagId();
    }

    private String buildCameraDebugSummary(DashboardData data) {
        return sanitize(data.cameraStatus())
                + " dev=" + data.cameraActiveDeviceId()
                + " frames=" + data.cameraFrameCount()
                + " age=" + formatMaybe(cameraFrameAgeSec(data)) + "s";
    }

    private double cameraFrameAgeSec(DashboardData data) {
        if (!Double.isFinite(data.robotTimestampSec()) || !Double.isFinite(data.cameraLastFrameTimestampSec())) {
            return Double.NaN;
        }
        return Math.max(0.0, data.robotTimestampSec() - data.cameraLastFrameTimestampSec());
    }

    private static boolean isBlank(String value) {
        return value == null || value.isBlank();
    }

    private static String nonBlankOr(String value, String fallback) {
        return isBlank(value) ? fallback : value;
    }

    private void updateMatchState(DashboardData data) {
        String phase = determinePhase(data);
        phaseLabel.setText("Phase: " + phase);

        if (data.matchTimeSec() >= 0.0 && Double.isFinite(data.matchTimeSec())) {
            matchTimeLabel.setText("Time: " + ONE_DECIMAL.format(data.matchTimeSec()) + "s");
        } else {
            matchTimeLabel.setText("Time: --.-");
        }

        Color phaseColor = determinePhaseColor(data);
        phaseLabel.setBackground(phaseColor);
        matchTimeLabel.setBackground(phaseColor);
    }

    private void updateTelemetryFreshness(DashboardData data, long nowNanos) {
        if (Double.isFinite(data.robotTimestampSec())
                && data.robotTimestampSec() > 0.0
                && Double.compare(data.robotTimestampSec(), lastRobotTimestampSec) != 0) {
            lastRobotTimestampSec = data.robotTimestampSec();
            lastTimestampSeenNanos = nowNanos;
        }

        if (!data.connected()) {
            freshnessLabel.setText("Telemetry: OFFLINE");
            freshnessLabel.setBackground(BAD);
            return;
        }

        long ageMs = Math.max(0L, (nowNanos - lastTimestampSeenNanos) / 1_000_000L);
        if (ageMs <= 250) {
            freshnessLabel.setText("Telemetry: LIVE " + ageMs + "ms");
            freshnessLabel.setBackground(OK);
        } else if (ageMs <= 1000) {
            freshnessLabel.setText("Telemetry: DELAY " + ageMs + "ms");
            freshnessLabel.setBackground(WARN);
        } else {
            freshnessLabel.setText("Telemetry: STALE " + ageMs + "ms");
            freshnessLabel.setBackground(BAD);
        }
    }

    // =========================================================================
    // TEMPERATURE DISPLAY
    // =========================================================================
    private void updateTemperatureLabels(DashboardData data) {
        driveTempLabel.setText("Drive: FL " + ZERO_DECIMAL.format(data.driveFLTempC())
                + " FR " + ZERO_DECIMAL.format(data.driveFRTempC())
                + " BL " + ZERO_DECIMAL.format(data.driveBLTempC())
                + " BR " + ZERO_DECIMAL.format(data.driveBRTempC()) + " C");
        double maxDriveTemp = Math.max(
                Math.max(data.driveFLTempC(), data.driveFRTempC()),
                Math.max(data.driveBLTempC(), data.driveBRTempC()));
        driveTempLabel.setForeground(tempColor(maxDriveTemp));

        shooterTempLabel.setText("Shooter: L " + ZERO_DECIMAL.format(data.shooterLeftTempC())
                + " R " + ZERO_DECIMAL.format(data.shooterRightTempC()) + " C");
        double maxShooterTemp = Math.max(data.shooterLeftTempC(), data.shooterRightTempC());
        shooterTempLabel.setForeground(tempColor(maxShooterTemp));
    }

    private void updateAutoSelectionCard(DashboardData data) {
        String selectedAutoName = sanitize(data.selectedAutoName());
        autoSelectorLabel.setText("Auto: " + selectedAutoName);
        if ("Do Nothing".equals(data.selectedAutoName())) {
            autoSelectorLabel.setForeground(WARN);
        } else if ("--".equals(selectedAutoName)) {
            autoSelectorLabel.setForeground(BAD);
        } else {
            autoSelectorLabel.setForeground(OK);
        }

        String source = formatAutoSource(data.selectedAutoSource());
        autoSourceLabel.setText("Source: " + source);
        if ("CUSTOM DASHBOARD".equals(data.selectedAutoSource())) {
            autoSourceLabel.setForeground(INFO);
        } else if ("SMARTDASHBOARD".equals(data.selectedAutoSource())) {
            autoSourceLabel.setForeground(OK);
        } else {
            autoSourceLabel.setForeground(MUTED);
        }

        if (data.autoCommandRunning()) {
            autoExecutionLabel.setText("Status: RUNNING");
            autoExecutionLabel.setForeground(OK);
        } else if ("AUTONOMOUS".equals(data.mode()) && data.enabled()) {
            autoExecutionLabel.setText("Status: FINISHED");
            autoExecutionLabel.setForeground(MUTED);
        } else {
            autoExecutionLabel.setText("Status: Idle");
            autoExecutionLabel.setForeground(MUTED);
        }

        syncAutoChooserModel(data);
        updateAutoChooserControls(data, selectedAutoName);
    }

    private static Color tempColor(double tempC) {
        if (tempC >= TEMP_CRITICAL_C) return BAD;
        if (tempC >= TEMP_WARN_C) return WARN;
        return TEXT;
    }

    // =========================================================================
    // COMMAND AVAILABILITY
    // =========================================================================
    private void updateCommandAvailability(DashboardData data) {
        boolean connected = data.connected();
        boolean disabled = "DISABLED".equals(data.mode());
        boolean teleopEnabled = data.enabled() && "TELEOP".equals(data.mode());
        boolean intakeHomeEnabled = data.enabled()
                && ("TELEOP".equals(data.mode()) || "TEST".equals(data.mode()));

        setButtonState(
                zeroHeadingButton,
                connected && (disabled || teleopEnabled),
                "Allowed in disabled or teleop",
                connected ? "Unavailable outside disabled/teleop" : "No robot connection");
        setButtonState(
                stopDriveButton,
                connected,
                "Always available when connected",
                "No robot connection");
        setButtonState(
                alignShootButton,
                connected && teleopEnabled,
                "Requires enabled teleop",
                connected ? "Enable teleop first" : "No robot connection");
        setButtonState(
                fallbackShootButton,
                connected && teleopEnabled,
                "Requires enabled teleop",
                connected ? "Enable teleop first" : "No robot connection");
        setButtonState(
                intakeHomeButton,
                connected && intakeHomeEnabled,
                "Requires enabled teleop/test",
                connected ? "Enable teleop/test to run intake home" : "No robot connection");
        setButtonState(
                level1ClimbButton,
                false, // --- CLIMBER DISABLED ---
                "Climber disabled — no hardware installed",
                "Climber not installed");
        boolean swerveValidationEnabled = connected && data.enabled()
                && ("TELEOP".equals(data.mode()) || "TEST".equals(data.mode()));
        setButtonState(
                swerveSteerPositiveButton,
                swerveValidationEnabled,
                "Requires enabled teleop/test",
                connected ? "Enable teleop/test to run module validation" : "No robot connection");
        setButtonState(
                swerveSteerNegativeButton,
                swerveValidationEnabled,
                "Requires enabled teleop/test",
                connected ? "Enable teleop/test to run module validation" : "No robot connection");
        setButtonState(
                swerveDrivePositiveButton,
                swerveValidationEnabled,
                "Requires enabled teleop/test",
                connected ? "Enable teleop/test to run module validation" : "No robot connection");
        setButtonState(
                swerveDriveNegativeButton,
                swerveValidationEnabled,
                "Requires enabled teleop/test",
                connected ? "Enable teleop/test to run module validation" : "No robot connection");
        setButtonState(
                swerveStopValidationButton,
                connected,
                "Always available when connected",
                "No robot connection");
        setButtonState(
                swerveCalibrateButton,
                connected,
                "Reads current no-offset CANcoder values",
                "No robot connection");
    }

    private void setButtonState(
            JButton button,
            boolean enabled,
            String enabledTooltip,
            String disabledTooltip) {
        if (button == null) {
            return;
        }
        button.setEnabled(enabled);
        button.setBackground(enabled ? BUTTON_ACTIVE : BUTTON_DISABLED);
        button.setToolTipText(enabled ? enabledTooltip : disabledTooltip);
    }

    // =========================================================================
    // CHECKLIST & PROGRESS BAR
    // =========================================================================
    private void updateChecklist(JLabel label, String title, boolean pass) {
        label.setText(title + ": " + (pass ? "PASS" : "WAIT"));
        label.setBackground(pass ? OK : PENDING);
        label.setForeground(Color.WHITE);
    }

    private void updateSwerveTools(DashboardData data) {
        String selectedModule = selectedSwerveModuleToken();
        swerveValidationStatusLabel.setText("Validation: "
                + sanitize(data.swerveValidationModeDisplayName())
                + " on " + sanitize(data.swerveValidationModuleDisplayName())
                + " active=" + yesNo(data.swerveValidationActive()));
        swerveValidationStatusLabel.setForeground(data.swerveValidationActive() ? OK : MUTED);

        swerveValidationOutputsLabel.setText("Outputs: drive="
                + formatSignedPercent(data.swerveValidationDrivePercent())
                + " steer=" + formatSignedPercent(data.swerveValidationSteerPercent()));

        swerveValidationDeltasLabel.setText("Deltas: angle="
                + formatMaybe(data.swerveValidationAngleDeltaDeg()) + " deg"
                + " cancoder=" + formatRotationMaybe(data.swerveValidationCANcoderDeltaRot()) + " rot");

        swerveValidationLiveLabel.setText(selectedModule + ": angle="
                + formatMaybe(selectedModuleAngleDeg(data, selectedModule))
                + " deg pos=" + formatRotationMaybe(selectedModulePosRot(data, selectedModule))
                + " abs=" + formatRotationMaybe(selectedModuleAbsRawRot(data, selectedModule)));

        swerveCalibrationLabel.setText(selectedModule + ": calRaw="
                + formatRotationMaybe(selectedModuleCalibrationRawRot(data, selectedModule))
                + " offset=" + formatRotationMaybe(selectedModuleCalibrationOffsetRot(data, selectedModule)));
    }

    private void updateYawBar(double yawDeg) {
        if (!Double.isFinite(yawDeg)) {
            yawBar.setValue(0);
            yawBar.setString("Yaw error: --");
            yawBar.setForeground(INFO);
            return;
        }

        int clamped = (int) Math.round(Math.max(-30.0, Math.min(30.0, yawDeg)));
        yawBar.setValue(clamped);
        yawBar.setString("Yaw error: " + ONE_DECIMAL.format(yawDeg) + " deg");

        double absYaw = Math.abs(yawDeg);
        if (absYaw <= Constants.Vision.YAW_TOLERANCE_DEG) {
            yawBar.setForeground(OK);
        } else if (absYaw <= 8.0) {
            yawBar.setForeground(WARN);
        } else {
            yawBar.setForeground(BAD);
        }
    }

    // =========================================================================
    // CONTROL EVENT FEED
    // =========================================================================
    private void updateControlEventFeed(DashboardData data) {
        if (!controlEventsInitialized) {
            controlEventsInitialized = true;
            lastControlEventSeqLogged = data.controlEventSeq();
            if (data.controlEventSeq() > 0) {
                appendControlEvent("Event #" + data.controlEventSeq() + ": " + sanitize(data.controlEventMessage()));
            } else {
                appendControlEvent("Waiting for control trigger events");
            }
            return;
        }

        if (data.controlEventSeq() < lastControlEventSeqLogged) {
            appendControlEvent("Control event sequence reset detected");
            lastControlEventSeqLogged = data.controlEventSeq();
            return;
        }

        if (data.controlEventSeq() > lastControlEventSeqLogged) {
            lastControlEventSeqLogged = data.controlEventSeq();
            appendControlEvent("Event #" + data.controlEventSeq()
                    + " @ " + formatMaybe(data.controlEventTimestampSec()) + "s: "
                    + sanitize(data.controlEventMessage()));
        }
    }

    private void appendControlEvent(String message) {
        String timestamp = LocalTime.now().format(LOG_TIME_FORMAT);
        controlEventLines.addLast(timestamp + "  " + message);
        while (controlEventLines.size() > MAX_CONTROL_EVENT_LINES) {
            controlEventLines.removeFirst();
        }
        controlLogArea.setText(String.join("\n", controlEventLines));
        controlLogArea.setCaretPosition(controlLogArea.getDocument().getLength());
    }

    // =========================================================================
    // EVENT FEED
    // =========================================================================
    private void updateEventFeed(DashboardData data) {
        if (!connectionInitialized) {
            connectionInitialized = true;
            lastConnected = data.connected();
            lastAckSeqLogged = data.ackSeq();
            appendEvent(data.connected() ? "Connected to robot" : "Waiting for robot connection");
        } else if (data.connected() != lastConnected) {
            lastConnected = data.connected();
            appendEvent(data.connected() ? "Connection restored" : "Connection lost");
            if (data.connected()) {
                lastAckSeqLogged = data.ackSeq();
            }
        }

        if (data.connected() && data.ackSeq() < lastAckSeqLogged) {
            appendEvent("Ack sequence reset detected");
            lastAckSeqLogged = data.ackSeq();
        }

        if (!readyInitialized) {
            readyInitialized = true;
            lastReady = data.readyToScore();
        } else if (data.readyToScore() != lastReady) {
            lastReady = data.readyToScore();
            appendEvent(data.readyToScore()
                    ? "Shot readiness became READY"
                    : "Shot readiness dropped: " + sanitize(data.readyReason()));
        }

        String selectedAutoName = sanitize(data.selectedAutoName());
        String selectedAutoSource = formatAutoSource(data.selectedAutoSource());
        if (!autoSelectionEventsInitialized) {
            autoSelectionEventsInitialized = true;
            lastSelectedAutoName = selectedAutoName;
            lastSelectedAutoSource = selectedAutoSource;
        } else if (!selectedAutoName.equals(lastSelectedAutoName)
                || !selectedAutoSource.equals(lastSelectedAutoSource)) {
            lastSelectedAutoName = selectedAutoName;
            lastSelectedAutoSource = selectedAutoSource;
            appendEvent("Auto selected: " + selectedAutoName + " via " + selectedAutoSource);
        }

        if (data.ackSeq() > lastAckSeqLogged) {
            lastAckSeqLogged = data.ackSeq();
            appendEvent("Command ack: " + sanitize(data.ackLastCommand())
                    + " #" + data.ackSeq()
                    + " " + sanitize(data.ackStatus())
                    + " (" + sanitize(data.ackMessage()) + ")");
        }
    }

    private void appendEvent(String message) {
        String timestamp = LocalTime.now().format(LOG_TIME_FORMAT);
        eventLogLines.addLast(timestamp + "  " + message);
        while (eventLogLines.size() > MAX_EVENT_LINES) {
            eventLogLines.removeFirst();
        }
        eventLogArea.setText(String.join("\n", eventLogLines));
        eventLogArea.setCaretPosition(eventLogArea.getDocument().getLength());
    }

    // =========================================================================
    // PIT RAW TEXT
    // =========================================================================
    private String buildRawText(DashboardData data) {
        StringBuilder sb = new StringBuilder();
        sb.append("Connection: ").append(data.connected()).append('\n');
        sb.append("Mode: ").append(data.mode()).append(" enabled=").append(data.enabled()).append('\n');
        sb.append("Alliance: ").append(data.alliance()).append("  MatchTime=").append(data.matchTimeSec()).append('\n');
        sb.append("RobotTimestampSec: ").append(data.robotTimestampSec()).append('\n');
        sb.append("Battery: ").append(ONE_DECIMAL.format(data.batteryVoltage())).append("V")
                .append(" brownout=").append(data.isBrownout())
                .append(" alert=").append(data.brownoutAlert()).append('\n');
        sb.append("Auto: '").append(sanitize(data.selectedAutoName()))
                .append("' source='").append(formatAutoSource(data.selectedAutoSource()))
                .append("' options=[").append(joinAutoOptions(data.autoOptions()))
                .append("] running=").append(data.autoCommandRunning()).append('\n');
        sb.append("Match: #").append(data.matchNumber())
                .append(" event='").append(sanitize(data.eventName())).append("'\n");
        sb.append("Camera: connected=").append(data.cameraConnected()).append('\n');
        sb.append("CameraDebug: status=").append(sanitize(data.cameraStatus()))
                .append(" activeDev=").append(data.cameraActiveDeviceId())
                .append(" name='").append(sanitize(data.cameraActiveName()))
                .append("' path='").append(sanitize(data.cameraActivePath()))
                .append("' frameCount=").append(data.cameraFrameCount())
                .append(" lastFrameTs=").append(formatMaybe(data.cameraLastFrameTimestampSec()))
                .append('\n');
        sb.append("CameraEnum: ").append(sanitize(data.cameraEnumerated())).append('\n');
        sb.append("CameraErr: ").append(sanitize(data.cameraLastError())).append('\n');
        sb.append("Vision: tagId=").append(data.visionTagId())
                .append(" distM=").append(formatMaybe(data.visionDistanceM())).append('\n');
        sb.append("CAN: util=").append(ONE_DECIMAL.format(data.canBusUtilization() * 100.0)).append("%")
                .append(" rxErr=").append(data.canReceiveErrorCount())
                .append(" txErr=").append(data.canTransmitErrorCount()).append('\n');
        sb.append("Pose: x=").append(data.poseX_m()).append(" y=").append(data.poseY_m())
                .append(" heading=").append(data.headingDeg()).append('\n');
        sb.append("Pigeon raw deg: yaw=").append(formatMaybe(data.pigeonYawDeg()))
                .append(" pitch=").append(formatMaybe(data.pigeonPitchDeg()))
                .append(" roll=").append(formatMaybe(data.pigeonRollDeg())).append('\n');
        sb.append("Swerve angles: FL=").append(ONE_DECIMAL.format(data.swerveFLAngleDeg()))
                .append(" FR=").append(ONE_DECIMAL.format(data.swerveFRAngleDeg()))
                .append(" BL=").append(ONE_DECIMAL.format(data.swerveBLAngleDeg()))
                .append(" BR=").append(ONE_DECIMAL.format(data.swerveBRAngleDeg())).append('\n');
        sb.append("CANcoder raw rot: FL=").append(formatMaybe(data.cancoderFLRawRot()))
                .append(" FR=").append(formatMaybe(data.cancoderFRRawRot()))
                .append(" BL=").append(formatMaybe(data.cancoderBLRawRot()))
                .append(" BR=").append(formatMaybe(data.cancoderBRRawRot())).append('\n');
        sb.append("CANcoder offset rot: FL=").append(formatMaybe(data.cancoderFLOffsetRot()))
                .append(" FR=").append(formatMaybe(data.cancoderFROffsetRot()))
                .append(" BL=").append(formatMaybe(data.cancoderBLOffsetRot()))
                .append(" BR=").append(formatMaybe(data.cancoderBROffsetRot())).append('\n');
        sb.append("CANcoder pos rot: FL=").append(formatMaybe(data.cancoderFLPosRot()))
                .append(" FR=").append(formatMaybe(data.cancoderFRPosRot()))
                .append(" BL=").append(formatMaybe(data.cancoderBLPosRot()))
                .append(" BR=").append(formatMaybe(data.cancoderBRPosRot())).append('\n');
        sb.append("CANcoder absRaw rot: FL=").append(formatMaybe(data.cancoderFLAbsRawRot()))
                .append(" FR=").append(formatMaybe(data.cancoderFRAbsRawRot()))
                .append(" BL=").append(formatMaybe(data.cancoderBLAbsRawRot()))
                .append(" BR=").append(formatMaybe(data.cancoderBRAbsRawRot())).append('\n');
        sb.append("CANcoder OK: FL=").append(data.cancoderFLOk())
                .append(" FR=").append(data.cancoderFROk())
                .append(" BL=").append(data.cancoderBLOk())
                .append(" BR=").append(data.cancoderBROk()).append('\n');
        sb.append("Drive temps: FL=").append(ZERO_DECIMAL.format(data.driveFLTempC()))
                .append(" FR=").append(ZERO_DECIMAL.format(data.driveFRTempC()))
                .append(" BL=").append(ZERO_DECIMAL.format(data.driveBLTempC()))
                .append(" BR=").append(ZERO_DECIMAL.format(data.driveBRTempC())).append(" C\n");
        sb.append("Shooter temps: L=").append(ZERO_DECIMAL.format(data.shooterLeftTempC()))
                .append(" R=").append(ZERO_DECIMAL.format(data.shooterRightTempC())).append(" C\n");
        sb.append("Shooter: L=").append(data.shooterLeftRps()).append(" R=").append(data.shooterRightRps())
                .append(" atSpeed=").append(data.shooterAtSpeed()).append('\n');
        sb.append("Intake: homed=").append(data.intakeHomed())
                .append(" limit=").append(data.intakeLimitSwitchPressed())
                .append(" tiltDeg=").append(data.intakeTiltDeg())
                .append(" rollerA=").append(data.intakeRollerCurrentAmps()).append('\n');
        sb.append("Conveyor: feederA=").append(data.feederCurrentAmps())
                .append(" hopperA=").append(data.hopperCurrentAmps()).append('\n');
        sb.append("Climber: DISABLED\n");
        // sb.append("Climber: armed=").append(data.climberArmed())
        //         .append(" posRot=").append(data.climberPositionRot())
        //         .append(" currentA=").append(data.climberCurrentAmps()).append('\n');
        sb.append("Align: state=").append(data.alignState())
                .append(" active=").append(data.alignCommandActive())
                .append(" hasTarget=").append(data.alignHasTarget())
                .append(" feasible=").append(data.alignGeometryFeasible())
                .append(" shootable=").append(data.alignHasShootableTarget())
                .append(" yaw=").append(data.alignYawDeg())
                .append(" pitch=").append(data.alignPitchDeg())
                .append(" abort='").append(sanitize(data.alignAbortReason())).append("'\n");
        sb.append("ReadyToScore: ").append(data.readyToScore())
                .append(" reason='").append(sanitize(data.readyReason())).append("'\n");
        sb.append("DriverButtons: ").append(sanitize(data.driverButtonsActive())).append('\n');
        sb.append("OperatorButtons: ").append(sanitize(data.operatorButtonsActive())).append('\n');
        sb.append("ControlEvent: seq=").append(data.controlEventSeq())
                .append(" ts=").append(data.controlEventTimestampSec())
                .append(" msg='").append(sanitize(data.controlEventMessage())).append("'\n");
        sb.append("Ack: cmd='").append(sanitize(data.ackLastCommand()))
                .append("' seq=").append(data.ackSeq())
                .append(" status='").append(sanitize(data.ackStatus()))
                .append("' msg='").append(sanitize(data.ackMessage()))
                .append("' ts=").append(data.ackTimestampSec()).append('\n');
        return sb.toString();
    }

    private String buildBringUpText(DashboardData data, long telemetryAgeMs) {
        StringBuilder sb = new StringBuilder();
        boolean connected = data.connected();
        boolean liveTelemetry = connected && telemetryAgeMs <= 1000;

        sb.append("3318 DEVICE BRING-UP / DEBUG VIEW\n");
        sb.append("=================================\n");
        sb.append("Link: ").append(statusForLink(connected, telemetryAgeMs))
                .append(" (connected=").append(connected)
                .append(", ageMs=").append(telemetryAgeMs).append(")\n");
        sb.append("Mode: ").append(data.mode()).append(" enabled=").append(data.enabled()).append('\n');
        sb.append('\n');

        sb.append("[Power + CAN]\n");
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, data.batteryVoltage() >= 11.5, "WARN"),
                "Battery",
                ONE_DECIMAL.format(data.batteryVoltage()) + "V brownout="
                        + data.isBrownout() + " alert=" + data.brownoutAlert());
        appendDeviceLine(
                sb,
                statusFromBoolean(
                        connected,
                        liveTelemetry,
                        data.canReceiveErrorCount() == 0 && data.canTransmitErrorCount() == 0,
                        "WARN"),
                "CAN bus",
                ONE_DECIMAL.format(data.canBusUtilization() * 100.0) + "% util"
                        + " rxErr=" + data.canReceiveErrorCount()
                        + " txErr=" + data.canTransmitErrorCount());
        sb.append('\n');

        sb.append("[IMU + Vision]\n");
        appendDeviceLine(
                sb,
                statusFromBoolean(
                        connected,
                        liveTelemetry,
                        Double.isFinite(data.pigeonYawDeg())
                                && Double.isFinite(data.pigeonPitchDeg())
                                && Double.isFinite(data.pigeonRollDeg()),
                        "CHECK"),
                "Pigeon2 (CAN " + Constants.CAN.PIGEON + ")",
                "yaw=" + formatMaybe(data.pigeonYawDeg())
                        + " pitch=" + formatMaybe(data.pigeonPitchDeg())
                        + " roll=" + formatMaybe(data.pigeonRollDeg()));
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, data.cameraConnected(), "WARN"),
                "USB camera (C920)",
                "connected=" + data.cameraConnected()
                        + " status=" + sanitize(data.cameraStatus())
                        + " cfgDev=" + Constants.Vision.CAMERA_DEVICE_ID
                        + " activeDev=" + data.cameraActiveDeviceId()
                        + " fps=" + Constants.Vision.CAMERA_FPS
                        + " size=" + Constants.Vision.CAMERA_WIDTH + "x" + Constants.Vision.CAMERA_HEIGHT);
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, data.cameraFrameCount() > 0, "WARN"),
                "Camera frames",
                "count=" + data.cameraFrameCount()
                        + " lastFrameTs=" + formatMaybe(data.cameraLastFrameTimestampSec())
                        + " ageSec=" + formatMaybe(cameraFrameAgeSec(data)));
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, !isBlank(data.cameraEnumerated()), "CHECK"),
                "Enumerated USB cameras",
                sanitize(nonBlankOr(data.cameraEnumerated(), "none")));
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, isBlank(data.cameraLastError()), "WARN"),
                "Camera last error",
                sanitize(nonBlankOr(data.cameraLastError(), "none")));
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, data.autoOptions().length > 0, "CHECK"),
                "Auto selection",
                "selected=" + sanitize(data.selectedAutoName())
                        + " source=" + formatAutoSource(data.selectedAutoSource())
                        + " options=" + joinAutoOptions(data.autoOptions()));
        sb.append('\n');

        sb.append("[Swerve]\n");
        appendSwerveModule(
                sb,
                connected,
                liveTelemetry,
                "Front Left",
                Constants.CAN.FRONT_LEFT_DRIVE,
                Constants.CAN.FRONT_LEFT_STEER,
                Constants.CAN.FRONT_LEFT_CANCODER,
                data.swerveFLAngleDeg(),
                data.driveFLTempC(),
                data.cancoderFLRawRot(),
                data.cancoderFLOffsetRot(),
                data.cancoderFLPosRot(),
                data.cancoderFLAbsRawRot(),
                data.cancoderFLOk());
        appendSwerveModule(
                sb,
                connected,
                liveTelemetry,
                "Front Right",
                Constants.CAN.FRONT_RIGHT_DRIVE,
                Constants.CAN.FRONT_RIGHT_STEER,
                Constants.CAN.FRONT_RIGHT_CANCODER,
                data.swerveFRAngleDeg(),
                data.driveFRTempC(),
                data.cancoderFRRawRot(),
                data.cancoderFROffsetRot(),
                data.cancoderFRPosRot(),
                data.cancoderFRAbsRawRot(),
                data.cancoderFROk());
        appendSwerveModule(
                sb,
                connected,
                liveTelemetry,
                "Back Left",
                Constants.CAN.BACK_LEFT_DRIVE,
                Constants.CAN.BACK_LEFT_STEER,
                Constants.CAN.BACK_LEFT_CANCODER,
                data.swerveBLAngleDeg(),
                data.driveBLTempC(),
                data.cancoderBLRawRot(),
                data.cancoderBLOffsetRot(),
                data.cancoderBLPosRot(),
                data.cancoderBLAbsRawRot(),
                data.cancoderBLOk());
        appendSwerveModule(
                sb,
                connected,
                liveTelemetry,
                "Back Right",
                Constants.CAN.BACK_RIGHT_DRIVE,
                Constants.CAN.BACK_RIGHT_STEER,
                Constants.CAN.BACK_RIGHT_CANCODER,
                data.swerveBRAngleDeg(),
                data.driveBRTempC(),
                data.cancoderBRRawRot(),
                data.cancoderBROffsetRot(),
                data.cancoderBRPosRot(),
                data.cancoderBRAbsRawRot(),
                data.cancoderBROk());
        sb.append('\n');

        sb.append("[Shooter]\n");
        appendDeviceLine(
                sb,
                statusFromTemp(connected, liveTelemetry, data.shooterLeftTempC()),
                "Shooter Left TalonFX (CAN " + Constants.CAN.SHOOTER_LEFT + ")",
                "rps=" + ONE_DECIMAL.format(data.shooterLeftRps())
                        + " temp=" + ONE_DECIMAL.format(data.shooterLeftTempC()) + "C");
        appendDeviceLine(
                sb,
                statusFromTemp(connected, liveTelemetry, data.shooterRightTempC()),
                "Shooter Right TalonFX (CAN " + Constants.CAN.SHOOTER_RIGHT + ")",
                "rps=" + ONE_DECIMAL.format(data.shooterRightRps())
                        + " temp=" + ONE_DECIMAL.format(data.shooterRightTempC()) + "C");
        sb.append('\n');

        sb.append("[Intake + Conveyor]\n");
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, data.intakeHomed(), "CHECK"),
                "Intake Tilt SparkMax (CAN " + Constants.CAN.INTAKE_TILT_NEO + ")",
                "tiltDeg=" + ONE_DECIMAL.format(data.intakeTiltDeg())
                        + " homed=" + data.intakeHomed()
                        + " limitSwitch=" + data.intakeLimitSwitchPressed());
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, Double.isFinite(data.intakeRollerCurrentAmps()), "CHECK"),
                "Intake Roller TalonFX (CAN " + Constants.CAN.INTAKE_ROLLER + ")",
                "current=" + ONE_DECIMAL.format(data.intakeRollerCurrentAmps()) + "A");
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, Double.isFinite(data.hopperCurrentAmps()), "CHECK"),
                "Hopper SparkMax (CAN " + Constants.CAN.HOPPER_FLOOR_NEO + ")",
                "current=" + ONE_DECIMAL.format(data.hopperCurrentAmps()) + "A");
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, Double.isFinite(data.feederCurrentAmps()), "CHECK"),
                "Feeder SparkMax (CAN " + Constants.CAN.FEEDER_NEO + ")",
                "current=" + ONE_DECIMAL.format(data.feederCurrentAmps()) + "A");
        sb.append('\n');

        sb.append("[Climber] DISABLED — no hardware installed\n");
        // appendDeviceLine(
        //         sb,
        //         statusFromBoolean(connected, liveTelemetry, Double.isFinite(data.climberPositionRot()), "CHECK"),
        //         "Climber Leader TalonFX (CAN 20)",
        //         "position=" + ONE_DECIMAL.format(data.climberPositionRot())
        //                 + " rot current=" + ONE_DECIMAL.format(data.climberCurrentAmps())
        //                 + "A armed=" + data.climberArmed());
        // appendDeviceLine(
        //         sb,
        //         statusFromBoolean(connected, liveTelemetry, data.climberArmed(), "CHECK"),
        //         "Climber Follower TalonFX (CAN 21)",
        //         "inferred from leader telemetry + climber arm gate");
        sb.append('\n');

        sb.append("[Notes]\n");
        sb.append("- Some statuses are inferred because direct per-device fault topics are not published yet.\n");
        sb.append("- CANcoder raw/offset telemetry appears after running the calibration command.\n");

        return sb.toString();
    }

    private static void appendSwerveModule(
            StringBuilder sb,
            boolean connected,
            boolean liveTelemetry,
            String name,
            int driveCanId,
            int steerCanId,
            int cancoderCanId,
            double moduleAngleDeg,
            double driveTempC,
            double cancoderRawRot,
            double cancoderOffsetRot,
            double cancoderPosRot,
            double cancoderAbsRawRot,
            boolean cancoderOk) {
        appendDeviceLine(
                sb,
                statusFromTemp(connected, liveTelemetry, driveTempC),
                name + " Drive TalonFX (CAN " + driveCanId + ")",
                "temp=" + ONE_DECIMAL.format(driveTempC) + "C");
        appendDeviceLine(
                sb,
                statusFromBoolean(connected, liveTelemetry, Double.isFinite(moduleAngleDeg), "CHECK"),
                name + " Steer TalonFX (CAN " + steerCanId + ")",
                "moduleAngle=" + formatMaybe(moduleAngleDeg) + " deg");
        String cancoderStatus = cancoderOk
                ? statusFromBoolean(connected, liveTelemetry, true, "CHECK")
                : (connected && liveTelemetry ? "ERROR" : statusFromBoolean(connected, liveTelemetry, false, "CHECK"));
        appendDeviceLine(
                sb,
                cancoderStatus,
                name + " CANcoder (CAN " + cancoderCanId + ")",
                "pos=" + formatRotationMaybe(cancoderPosRot)
                        + " absRaw=" + formatRotationMaybe(cancoderAbsRawRot)
                        + " calRaw=" + formatRotationMaybe(cancoderRawRot)
                        + " offset=" + formatRotationMaybe(cancoderOffsetRot)
                        + " ok=" + cancoderOk);
    }

    private static void appendDeviceLine(StringBuilder sb, String status, String name, String details) {
        sb.append('[').append(status).append("] ").append(name).append(": ").append(details).append('\n');
    }

    private static String statusForLink(boolean connected, long telemetryAgeMs) {
        if (!connected) return "OFFLINE";
        if (telemetryAgeMs > 1000) return "STALE";
        if (telemetryAgeMs > 250) return "DELAY";
        return "LIVE";
    }

    private static String statusFromTemp(boolean connected, boolean liveTelemetry, double tempC) {
        if (!connected) return "OFFLINE";
        if (!liveTelemetry) return "STALE";
        if (!Double.isFinite(tempC)) return "CHECK";
        if (tempC >= TEMP_WARN_C) return "WARN";
        return "OK";
    }

    private static String statusFromBoolean(
            boolean connected,
            boolean liveTelemetry,
            boolean pass,
            String failStatus) {
        if (!connected) return "OFFLINE";
        if (!liveTelemetry) return "STALE";
        return pass ? "OK" : failStatus;
    }

    // =========================================================================
    // UTILITIES
    // =========================================================================
    private static String sanitize(String value) {
        return value == null || value.isBlank() ? "--" : value;
    }

    private static String formatAutoSource(String value) {
        if (value == null || value.isBlank()) {
            return "--";
        }
        return switch (value) {
            case "CUSTOM DASHBOARD" -> "Custom Dashboard";
            case "SMARTDASHBOARD" -> "SmartDashboard";
            case "DEFAULT" -> "Default";
            default -> value;
        };
    }

    private static String joinAutoOptions(String[] autoOptions) {
        if (autoOptions == null || autoOptions.length == 0) {
            return "--";
        }
        return String.join(", ", autoOptions);
    }

    private static String formatMaybe(double value) {
        return Double.isFinite(value) ? ONE_DECIMAL.format(value) : "--";
    }

    private static String formatRotationMaybe(double value) {
        return Double.isFinite(value) ? ROTATION_DECIMAL.format(value) : "--";
    }

    private static String formatSignedPercent(double value) {
        return Double.isFinite(value) ? String.format("%+.0f%%", value * 100.0) : "--";
    }

    private String selectedSwerveModuleToken() {
        if (swerveModuleCombo == null || swerveModuleCombo.getSelectedItem() == null) {
            return "FL";
        }
        return swerveModuleCombo.getSelectedItem().toString();
    }

    private static double selectedModuleAngleDeg(DashboardData data, String moduleToken) {
        return switch (moduleToken) {
            case "FR" -> data.swerveFRAngleDeg();
            case "BL" -> data.swerveBLAngleDeg();
            case "BR" -> data.swerveBRAngleDeg();
            default -> data.swerveFLAngleDeg();
        };
    }

    private static double selectedModulePosRot(DashboardData data, String moduleToken) {
        return switch (moduleToken) {
            case "FR" -> data.cancoderFRPosRot();
            case "BL" -> data.cancoderBLPosRot();
            case "BR" -> data.cancoderBRPosRot();
            default -> data.cancoderFLPosRot();
        };
    }

    private static double selectedModuleAbsRawRot(DashboardData data, String moduleToken) {
        return switch (moduleToken) {
            case "FR" -> data.cancoderFRAbsRawRot();
            case "BL" -> data.cancoderBLAbsRawRot();
            case "BR" -> data.cancoderBRAbsRawRot();
            default -> data.cancoderFLAbsRawRot();
        };
    }

    private static double selectedModuleCalibrationRawRot(DashboardData data, String moduleToken) {
        return switch (moduleToken) {
            case "FR" -> data.cancoderFRRawRot();
            case "BL" -> data.cancoderBLRawRot();
            case "BR" -> data.cancoderBRRawRot();
            default -> data.cancoderFLRawRot();
        };
    }

    private static double selectedModuleCalibrationOffsetRot(DashboardData data, String moduleToken) {
        return switch (moduleToken) {
            case "FR" -> data.cancoderFROffsetRot();
            case "BL" -> data.cancoderBLOffsetRot();
            case "BR" -> data.cancoderBROffsetRot();
            default -> data.cancoderFLOffsetRot();
        };
    }

    private static String yesNo(boolean value) {
        return value ? "YES" : "NO";
    }

    private void syncAutoChooserModel(DashboardData data) {
        if (autoChooserCombo == null) {
            return;
        }

        String[] autoOptions = data.autoOptions();
        if (autoOptions == null) {
            autoOptions = new String[0];
        }

        String currentSelection = comboSelection();
        boolean optionsChanged = autoChooserCombo.getItemCount() != autoOptions.length;
        if (!optionsChanged) {
            for (int i = 0; i < autoOptions.length; i++) {
                if (!autoOptions[i].equals(autoChooserCombo.getItemAt(i))) {
                    optionsChanged = true;
                    break;
                }
            }
        }

        if (optionsChanged) {
            updatingAutoChooserModel = true;
            autoChooserCombo.removeAllItems();
            for (String autoOption : autoOptions) {
                autoChooserCombo.addItem(autoOption);
            }
            updatingAutoChooserModel = false;
        }

        String selectedAutoName = sanitize(data.selectedAutoName());
        if (pendingAutoSelectionName == null
                || pendingAutoSelectionName.isBlank()
                || pendingAutoSelectionName.equals(lastRobotSelectedAutoNameForChooser)) {
            pendingAutoSelectionName = selectedAutoName;
        }

        if (!containsAutoOption(autoOptions, pendingAutoSelectionName)) {
            pendingAutoSelectionName = containsAutoOption(autoOptions, selectedAutoName)
                    ? selectedAutoName
                    : (autoOptions.length > 0 ? autoOptions[0] : null);
        }

        String desiredSelection = pendingAutoSelectionName;
        if (desiredSelection == null && autoOptions.length > 0) {
            desiredSelection = autoOptions[0];
            pendingAutoSelectionName = desiredSelection;
        }

        if (desiredSelection != null && !desiredSelection.equals(currentSelection)) {
            updatingAutoChooserModel = true;
            autoChooserCombo.setSelectedItem(desiredSelection);
            updatingAutoChooserModel = false;
        }

        lastRobotSelectedAutoNameForChooser = selectedAutoName;
    }

    private void updateAutoChooserControls(DashboardData data, String selectedAutoName) {
        boolean connected = data.connected();
        boolean disabled = "DISABLED".equals(data.mode());
        String[] autoOptions = data.autoOptions();
        boolean hasOptions = autoOptions != null && autoOptions.length > 0;
        boolean pendingValid = pendingAutoSelectionName != null && !pendingAutoSelectionName.isBlank();
        boolean hasPendingChange = pendingValid && !pendingAutoSelectionName.equals(selectedAutoName);

        if (autoChooserCombo != null) {
            autoChooserCombo.setEnabled(connected && disabled && hasOptions);
            autoChooserCombo.setToolTipText(autoChooserTooltip(connected, disabled, hasOptions));
        }

        if (!connected) {
            autoChooserStatusLabel.setText("Chooser: OFFLINE");
            autoChooserStatusLabel.setForeground(BAD);
        } else if (!hasOptions) {
            autoChooserStatusLabel.setText("Chooser: Waiting for robot auto list");
            autoChooserStatusLabel.setForeground(WARN);
        } else if (!disabled) {
            autoChooserStatusLabel.setText("Chooser: LOCKED while enabled");
            autoChooserStatusLabel.setForeground(WARN);
        } else if (hasPendingChange) {
            autoChooserStatusLabel.setText("Chooser: Pending apply -> " + pendingAutoSelectionName);
            autoChooserStatusLabel.setForeground(PENDING);
        } else {
            autoChooserStatusLabel.setText("Chooser: Synced and ready");
            autoChooserStatusLabel.setForeground(OK);
        }

        setButtonState(
                autoApplyButton,
                connected && disabled && hasOptions && hasPendingChange,
                "Apply the pending auto selection while disabled",
                autoApplyTooltip(connected, disabled, hasOptions, hasPendingChange));
    }

    private String comboSelection() {
        if (autoChooserCombo == null) {
            return null;
        }
        Object selectedItem = autoChooserCombo.getSelectedItem();
        return selectedItem == null ? null : selectedItem.toString();
    }

    private static boolean containsAutoOption(String[] autoOptions, String candidate) {
        if (candidate == null || autoOptions == null) {
            return false;
        }
        for (String autoOption : autoOptions) {
            if (candidate.equals(autoOption)) {
                return true;
            }
        }
        return false;
    }

    private static String autoChooserTooltip(boolean connected, boolean disabled, boolean hasOptions) {
        if (!connected) {
            return "No robot connection";
        }
        if (!hasOptions) {
            return "Waiting for auto options from robot";
        }
        if (!disabled) {
            return "Auto selection is only allowed while disabled";
        }
        return "Pick an autonomous routine";
    }

    private static String autoApplyTooltip(
            boolean connected,
            boolean disabled,
            boolean hasOptions,
            boolean hasPendingChange) {
        if (!connected) {
            return "No robot connection";
        }
        if (!hasOptions) {
            return "Waiting for auto options from robot";
        }
        if (!disabled) {
            return "Auto selection is only allowed while disabled";
        }
        if (!hasPendingChange) {
            return "Pick a different auto to apply";
        }
        return "Send the selected auto to the robot";
    }

    private static String nextAction(DashboardData data) {
        if (data.readyToScore()) {
            return "Shoot now";
        }

        String reason = sanitize(data.readyReason());
        if ("Intake not homed".equals(reason)) {
            return "Run Intake Home in enabled teleop/test";
        }
        if ("No vision target".equals(reason)) {
            return "Acquire AprilTag before firing";
        }
        if ("Yaw not aligned".equals(reason)) {
            return "Hold align command steady";
        }
        if ("Shooter spinning up".equals(reason)) {
            return "Wait for shooter speed";
        }
        if ("Align command idle".equals(reason)) {
            return "Run Align + Shoot command";
        }
        return "Hold current state";
    }

    private static String determinePhase(DashboardData data) {
        if ("AUTONOMOUS".equals(data.mode())) {
            return "AUTONOMOUS";
        }
        if ("TELEOP".equals(data.mode())) {
            if (!data.enabled()) {
                return "TELEOP (DISABLED)";
            }
            if (data.matchTimeSec() >= 0.0 && data.matchTimeSec() <= 15.0) {
                return "TELEOP LAST 15";
            }
            if (data.matchTimeSec() >= 0.0 && data.matchTimeSec() <= 30.0) {
                return "TELEOP ENDGAME";
            }
            return "TELEOP";
        }
        if ("DISABLED".equals(data.mode())) {
            return "DISABLED";
        }
        if ("TEST".equals(data.mode())) {
            return "TEST";
        }
        return "UNKNOWN";
    }

    private static Color determinePhaseColor(DashboardData data) {
        if ("AUTONOMOUS".equals(data.mode()) && data.enabled()) {
            return INFO;
        }
        if ("TELEOP".equals(data.mode()) && data.enabled()) {
            if (data.matchTimeSec() >= 0.0 && data.matchTimeSec() <= 15.0) {
                return BAD;
            }
            if (data.matchTimeSec() >= 0.0 && data.matchTimeSec() <= 30.0) {
                return WARN;
            }
            return OK;
        }
        if ("DISABLED".equals(data.mode())) {
            return new Color(102, 120, 144);
        }
        return CARD;
    }

    private static JLabel createChecklistLabel(String title) {
        JLabel label = new JLabel(title + ": WAIT");
        label.setOpaque(true);
        label.setBackground(PENDING);
        label.setForeground(Color.WHITE);
        label.setFont(new Font("Segoe UI", Font.BOLD, 14));
        label.setBorder(BorderFactory.createEmptyBorder(3, 8, 3, 8));
        return label;
    }

    // =========================================================================
    // FIELD VISUALIZATION PANEL
    // =========================================================================
    private static final class FieldPanel extends JPanel {
        private double poseX_m;
        private double poseY_m;
        private double headingDeg;
        private String mode = "UNKNOWN";
        private String alliance = "UNKNOWN";
        private boolean cameraConnected;
        private boolean hasTarget;
        private int visionTagId = -1;
        private double visionDistanceM = Double.NaN;
        private double alignYawDeg = Double.NaN;
        private double autoStartX_m = Double.NaN;
        private double autoStartY_m = Double.NaN;
        private double autoStartHeadingDeg = Double.NaN;

        private static final double FIELD_LENGTH_M = 17.6;
        private static final double FIELD_WIDTH_M = 8.0;

        // HUB center positions (meters)
        private static final double RED_HUB_X = 11.92;
        private static final double RED_HUB_Y = 4.03;
        private static final double BLUE_HUB_X = 4.63;
        private static final double BLUE_HUB_Y = 4.03;
        private static final double HUB_RADIUS_M = 0.85;

        // Red and Blue HUB tag IDs (matching Constants.Vision)
        private static final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
        private static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

        private static final Color HUB_RED = new Color(180, 40, 40, 100);
        private static final Color HUB_BLUE = new Color(40, 40, 180, 100);
        private static final Color HUB_RED_BORDER = new Color(220, 80, 80, 180);
        private static final Color HUB_BLUE_BORDER = new Color(80, 80, 220, 180);
        private static final Color VISION_LINE = new Color(0, 255, 100, 120);
        private static final Color EXPECTED_POSE = new Color(0, 200, 255, 160);
        private static final Color PLACEMENT_OK = new Color(0, 220, 80);
        private static final Color PLACEMENT_WARN = new Color(255, 80, 40);
        private static final Font INFO_FONT = new Font("Segoe UI", Font.BOLD, 13);
        private static final Font TAG_FONT = new Font("Segoe UI", Font.BOLD, 11);

        FieldPanel() {
            setBackground(new Color(9, 14, 24));
            setBorder(BorderFactory.createLineBorder(new Color(60, 87, 120), 2));
        }

        void update(DashboardData data) {
            this.poseX_m = data.poseX_m();
            this.poseY_m = data.poseY_m();
            this.headingDeg = data.headingDeg();
            this.mode = data.mode();
            this.alliance = data.alliance();
            this.cameraConnected = data.cameraConnected();
            this.hasTarget = data.alignHasTarget();
            this.visionTagId = data.visionTagId();
            this.visionDistanceM = data.visionDistanceM();
            this.alignYawDeg = data.alignYawDeg();

            // Expected auto starting pose (flip for red alliance)
            double startX = data.autoStartX_m();
            double startY = data.autoStartY_m();
            double startH = data.autoStartHeadingDeg();
            if (Double.isFinite(startX) && "RED".equals(data.alliance())) {
                startX = FIELD_LENGTH_M - startX;
                startY = FIELD_WIDTH_M - startY;
                startH = startH + 180.0;
            }
            this.autoStartX_m = startX;
            this.autoStartY_m = startY;
            this.autoStartHeadingDeg = startH;

            repaint();
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            int pad = 30;
            int width = getWidth() - (pad * 2);
            int height = getHeight() - (pad * 2);

            // Field background
            g2.setColor(new Color(20, 43, 70));
            g2.fillRoundRect(pad, pad, width, height, 18, 18);
            g2.setColor(new Color(85, 122, 165));
            g2.setStroke(new BasicStroke(2f));
            g2.drawRoundRect(pad, pad, width, height, 18, 18);

            // Center lines
            g2.setColor(new Color(70, 100, 130));
            g2.drawLine(pad + width / 2, pad, pad + width / 2, pad + height);
            g2.drawLine(pad, pad + height / 2, pad + width, pad + height / 2);

            // Draw expected auto starting position (ghost marker, only when disabled)
            boolean hasExpectedPose = Double.isFinite(autoStartX_m) && Double.isFinite(autoStartY_m);
            int expectedPxX = 0, expectedPxY = 0;
            double placementDistM = Double.NaN;
            if (hasExpectedPose && "DISABLED".equals(mode)) {
                expectedPxX = pad + (int) Math.round(clamp(autoStartX_m / FIELD_LENGTH_M, 0.0, 1.0) * width);
                expectedPxY = pad + height - (int) Math.round(clamp(autoStartY_m / FIELD_WIDTH_M, 0.0, 1.0) * height);

                // Crosshair marker
                g2.setColor(EXPECTED_POSE);
                g2.setStroke(new BasicStroke(2f));
                g2.drawOval(expectedPxX - 14, expectedPxY - 14, 28, 28);
                g2.drawLine(expectedPxX - 18, expectedPxY, expectedPxX + 18, expectedPxY);
                g2.drawLine(expectedPxX, expectedPxY - 18, expectedPxX, expectedPxY + 18);

                // Heading arrow for expected pose
                if (Double.isFinite(autoStartHeadingDeg)) {
                    double expRad = Math.toRadians(autoStartHeadingDeg);
                    int expArrowX = expectedPxX + (int) Math.round(Math.cos(expRad) * 24.0);
                    int expArrowY = expectedPxY - (int) Math.round(Math.sin(expRad) * 24.0);
                    g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                            0, new float[]{4, 4}, 0));
                    g2.drawLine(expectedPxX, expectedPxY, expArrowX, expArrowY);
                }

                // Compute distance between current pose and expected pose
                double dx = poseX_m - autoStartX_m;
                double dy = poseY_m - autoStartY_m;
                placementDistM = Math.sqrt(dx * dx + dy * dy);
            }

            // Draw HUBs
            drawHub(g2, pad, width, height, RED_HUB_X, RED_HUB_Y,
                    HUB_RED, HUB_RED_BORDER, "RED HUB", RED_HUB_TAG_IDS);
            drawHub(g2, pad, width, height, BLUE_HUB_X, BLUE_HUB_Y,
                    HUB_BLUE, HUB_BLUE_BORDER, "BLUE HUB", BLUE_HUB_TAG_IDS);

            // Robot position
            int robotX = pad + (int) Math.round(clamp(poseX_m / FIELD_LENGTH_M, 0.0, 1.0) * width);
            int robotY = pad + height - (int) Math.round(clamp(poseY_m / FIELD_WIDTH_M, 0.0, 1.0) * height);

            // Draw vision line to target HUB if we have a target
            if (hasTarget && visionTagId >= 0) {
                double hubX = isRedTag(visionTagId) ? RED_HUB_X : BLUE_HUB_X;
                double hubY = isRedTag(visionTagId) ? RED_HUB_Y : BLUE_HUB_Y;
                int hubPxX = pad + (int) Math.round(clamp(hubX / FIELD_LENGTH_M, 0.0, 1.0) * width);
                int hubPxY = pad + height - (int) Math.round(clamp(hubY / FIELD_WIDTH_M, 0.0, 1.0) * height);

                g2.setColor(VISION_LINE);
                g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                        0, new float[]{8, 6}, 0));
                g2.drawLine(robotX, robotY, hubPxX, hubPxY);
                g2.setStroke(new BasicStroke(2f));
            }

            // Robot dot
            g2.setColor("AUTONOMOUS".equals(mode) ? WARN : OK);
            g2.fillOval(robotX - 10, robotY - 10, 20, 20);

            // Heading arrow
            double radians = Math.toRadians(headingDeg);
            int arrowX = robotX + (int) Math.round(Math.cos(radians) * 28.0);
            int arrowY = robotY - (int) Math.round(Math.sin(radians) * 28.0);
            g2.setStroke(new BasicStroke(3f));
            g2.drawLine(robotX, robotY, arrowX, arrowY);

            // ---- Top-left: Pose + heading ----
            g2.setFont(new Font("Segoe UI", Font.BOLD, 16));
            g2.setColor(TEXT);
            g2.drawString("Pose (" + ONE_DECIMAL.format(poseX_m) + ", "
                    + ONE_DECIMAL.format(poseY_m) + ")  Heading "
                    + ONE_DECIMAL.format(headingDeg) + "\u00B0", pad, 22);

            // ---- Top-right: Vision status indicator ----
            int statusX = pad + width - 8;
            g2.setFont(INFO_FONT);
            if (cameraConnected) {
                if (hasTarget) {
                    g2.setColor(OK);
                    String tagLabel = "TAG #" + visionTagId;
                    int labelW = g2.getFontMetrics().stringWidth(tagLabel);
                    g2.fillOval(statusX - labelW - 20, 10, 12, 12);
                    g2.drawString(tagLabel, statusX - labelW, 22);
                } else {
                    g2.setColor(WARN);
                    int labelW = g2.getFontMetrics().stringWidth("NO TARGET");
                    g2.fillOval(statusX - labelW - 20, 10, 12, 12);
                    g2.drawString("NO TARGET", statusX - labelW, 22);
                }
            } else {
                g2.setColor(BAD);
                int labelW = g2.getFontMetrics().stringWidth("CAM OFF");
                g2.fillOval(statusX - labelW - 20, 10, 12, 12);
                g2.drawString("CAM OFF", statusX - labelW, 22);
            }

            // ---- Bottom-left: Distance + yaw info ----
            int bottomY = pad + height + 20;
            g2.setFont(INFO_FONT);
            g2.setColor(TEXT);
            StringBuilder info = new StringBuilder();
            if (Double.isFinite(visionDistanceM)) {
                info.append("Dist: ").append(ONE_DECIMAL.format(visionDistanceM)).append("m");
            }
            if (Double.isFinite(alignYawDeg)) {
                if (info.length() > 0) info.append("  ");
                info.append("Yaw: ").append(ONE_DECIMAL.format(alignYawDeg)).append("\u00B0");
            }
            if (info.length() > 0) {
                g2.drawString(info.toString(), pad, bottomY);
            }

            // ---- Bottom-right: Placement check or odometry-only warning ----
            if (hasExpectedPose && "DISABLED".equals(mode) && Double.isFinite(placementDistM)) {
                String placementMsg;
                if (placementDistM < 0.5) {
                    g2.setColor(PLACEMENT_OK);
                    placementMsg = "PLACEMENT OK (" + ONE_DECIMAL.format(placementDistM) + "m)";
                } else {
                    g2.setColor(PLACEMENT_WARN);
                    placementMsg = "CHECK PLACEMENT (" + ONE_DECIMAL.format(placementDistM) + "m off)";
                }
                int msgW = g2.getFontMetrics().stringWidth(placementMsg);
                g2.drawString(placementMsg, statusX - msgW, bottomY);
            } else if (!cameraConnected || !hasTarget) {
                g2.setColor(WARN);
                String warn = "ODOMETRY ONLY";
                int warnW = g2.getFontMetrics().stringWidth(warn);
                g2.drawString(warn, statusX - warnW, bottomY);
            }

            g2.dispose();
        }

        private void drawHub(Graphics2D g2, int pad, int fieldW, int fieldH,
                             double hubX_m, double hubY_m,
                             Color fill, Color border, String label, int[] tagIds) {
            int cx = pad + (int) Math.round(clamp(hubX_m / FIELD_LENGTH_M, 0.0, 1.0) * fieldW);
            int cy = pad + fieldH - (int) Math.round(clamp(hubY_m / FIELD_WIDTH_M, 0.0, 1.0) * fieldH);
            int radiusPx = (int) Math.round((HUB_RADIUS_M / FIELD_LENGTH_M) * fieldW);

            // Hub fill
            g2.setColor(fill);
            g2.fillOval(cx - radiusPx, cy - radiusPx, radiusPx * 2, radiusPx * 2);

            // Hub border — highlight if our active tag belongs to this hub
            boolean active = isTagInSet(visionTagId, tagIds) && hasTarget;
            g2.setColor(active ? border.brighter() : border);
            g2.setStroke(new BasicStroke(active ? 3f : 1.5f));
            g2.drawOval(cx - radiusPx, cy - radiusPx, radiusPx * 2, radiusPx * 2);

            // Hub label
            g2.setFont(TAG_FONT);
            g2.setColor(new Color(200, 200, 200, 180));
            FontMetrics fm = g2.getFontMetrics();
            g2.drawString(label, cx - fm.stringWidth(label) / 2, cy - 2);

            // Tag IDs under hub label
            String ids = formatTagIds(tagIds);
            g2.setFont(new Font("Segoe UI", Font.PLAIN, 10));
            fm = g2.getFontMetrics();
            g2.drawString(ids, cx - fm.stringWidth(ids) / 2, cy + fm.getAscent() + 1);
        }

        private static String formatTagIds(int[] ids) {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < ids.length; i++) {
                if (i > 0) sb.append(",");
                sb.append(ids[i]);
            }
            return sb.toString();
        }

        private static boolean isRedTag(int tagId) {
            return isTagInSet(tagId, RED_HUB_TAG_IDS);
        }

        private static boolean isTagInSet(int tagId, int[] ids) {
            if (tagId < 0) return false;
            for (int id : ids) {
                if (id == tagId) return true;
            }
            return false;
        }

        private static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }

    @Override
    public void dispose() {
        visionStreamPanel.shutdown();
        super.dispose();
    }

    static {
        UIManager.put("TabbedPane.foreground", TEXT);
        UIManager.put("TabbedPane.selected", CARD);
    }
}
