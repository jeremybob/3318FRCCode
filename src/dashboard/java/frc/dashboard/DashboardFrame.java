package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.RenderingHints;
import java.text.DecimalFormat;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayDeque;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.ScrollPaneConstants;
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

    // Driver tab: auto selector & execution
    private final JLabel autoSelectorLabel = new JLabel("Auto: --");
    private final JLabel autoExecutionLabel = new JLabel("Status: Idle");

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
    private final JLabel climberLabel = new JLabel("Armed: NO  Pos: -- rot  I: -- A");
    private final JLabel operatorVisionLabel = new JLabel("Target: NO  Feasible: NO");
    private final JLabel operatorPhaseLabel = new JLabel("Align phase: IDLE");
    private final JLabel operatorReadyLabel = new JLabel("NOT READY");
    private final JLabel operatorReadyReasonLabel = new JLabel("Reason: --");

    // Operator tab: swerve module angles
    private final JLabel swerveAnglesLabel = new JLabel("FL -- FR -- BL -- BR --");

    // Operator tab: system health (CAN + camera)
    private final JLabel canHealthLabel = new JLabel("CAN: --");
    private final JLabel cameraStatusLabel = new JLabel("Camera: --");
    private final JLabel pigeonLabel = new JLabel("Pigeon: Y -- P -- R --");

    // Operator tab: motor temperatures
    private final JLabel driveTempLabel = new JLabel("Drive: FL -- FR -- BL -- BR --");
    private final JLabel shooterTempLabel = new JLabel("Shooter: L -- R --");

    // Command ack
    private final JLabel ackLabel = new JLabel("Ack: --");
    private final JTextArea eventLogArea = new JTextArea();
    private final ArrayDeque<String> eventLogLines = new ArrayDeque<>();

    // Pit tab
    private final JTextArea pitRawArea = new JTextArea();
    // Controls tab
    private final JLabel driverButtonsLabel = new JLabel("Driver buttons: --");
    private final JLabel operatorButtonsLabel = new JLabel("Operator buttons: --");
    private final JLabel controlLastEventLabel = new JLabel("Last event: --");
    private final JTextArea controlLogArea = new JTextArea();
    private final ArrayDeque<String> controlEventLines = new ArrayDeque<>();

    // Buttons
    private JButton zeroHeadingButton;
    private JButton stopDriveButton;
    private JButton alignShootButton;
    private JButton fallbackShootButton;
    private JButton intakeHomeButton;
    private JButton level1ClimbButton;

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
        styleMetricLabel(autoSelectorLabel);
        styleMetricLabel(autoExecutionLabel);
        styleCompactLabel(alignPhaseLabel);
        styleCompactLabel(yawLabel);
        styleCompactLabel(pitchLabel);
        styleCompactLabel(visionLabel);
        styleCompactLabel(abortLabel);
        styleMetricLabel(ackLabel);

        JPanel side = new JPanel();
        side.setLayout(new BoxLayout(side, BoxLayout.Y_AXIS));
        side.setBackground(BG);

        addSideCard(side, wrapLabelCard("Auto Selection", autoSelectorLabel, autoExecutionLabel));
        addSideCard(side, buildPreMatchChecklistCard());
        addSideCard(side, wrapLabelCard("Shot Readiness", readyLabel, readyReasonLabel, nextActionLabel));
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
        top.add(wrapLabelCard("System Health", canHealthLabel, cameraStatusLabel, pigeonLabel));
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
        String autoName = sanitize(data.selectedAutoName());
        autoSelectorLabel.setText("Auto: " + autoName);
        if ("Do Nothing".equals(data.selectedAutoName())) {
            autoSelectorLabel.setForeground(WARN);
        } else if ("--".equals(autoName)) {
            autoSelectorLabel.setForeground(BAD);
        } else {
            autoSelectorLabel.setForeground(OK);
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
        climberLabel.setText("Armed: " + yesNo(data.climberArmed())
                + "  Pos: " + ONE_DECIMAL.format(data.climberPositionRot())
                + " rot  I: " + ONE_DECIMAL.format(data.climberCurrentAmps()) + " A");

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
        pigeonLabel.setText("Pigeon: Y " + formatMaybe(data.pigeonYawDeg())
                + "  P " + formatMaybe(data.pigeonPitchDeg())
                + "  R " + formatMaybe(data.pigeonRollDeg()) + " deg");
        pigeonLabel.setForeground(
                Double.isFinite(data.pigeonYawDeg())
                        && Double.isFinite(data.pigeonPitchDeg())
                        && Double.isFinite(data.pigeonRollDeg()) ? TEXT : WARN);

        // Operator tab: motor temperatures
        updateTemperatureLabels(data);

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

        fieldPanel.updatePose(data.poseX_m(), data.poseY_m(), data.headingDeg(), data.mode());
        pitRawArea.setText(buildRawText(data));
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
                connected && teleopEnabled && data.climberArmed(),
                "Requires teleop and climb arm gate",
                connected ? "Hold Start + Back in teleop" : "No robot connection");
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
                .append("' running=").append(data.autoCommandRunning()).append('\n');
        sb.append("Match: #").append(data.matchNumber())
                .append(" event='").append(sanitize(data.eventName())).append("'\n");
        sb.append("Camera: connected=").append(data.cameraConnected()).append('\n');
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
        sb.append("Climber: armed=").append(data.climberArmed())
                .append(" posRot=").append(data.climberPositionRot())
                .append(" currentA=").append(data.climberCurrentAmps()).append('\n');
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

    // =========================================================================
    // UTILITIES
    // =========================================================================
    private static String sanitize(String value) {
        return value == null || value.isBlank() ? "--" : value;
    }

    private static String formatMaybe(double value) {
        return Double.isFinite(value) ? ONE_DECIMAL.format(value) : "--";
    }

    private static String yesNo(boolean value) {
        return value ? "YES" : "NO";
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

        // Approximate full-field dimensions in meters for scaling.
        private static final double FIELD_LENGTH_M = 17.6;
        private static final double FIELD_WIDTH_M = 8.0;

        FieldPanel() {
            setBackground(new Color(9, 14, 24));
            setBorder(BorderFactory.createLineBorder(new Color(60, 87, 120), 2));
        }

        void updatePose(double x_m, double y_m, double heading_deg, String mode) {
            this.poseX_m = x_m;
            this.poseY_m = y_m;
            this.headingDeg = heading_deg;
            this.mode = mode;
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

            g2.setColor(new Color(20, 43, 70));
            g2.fillRoundRect(pad, pad, width, height, 18, 18);
            g2.setColor(new Color(85, 122, 165));
            g2.setStroke(new BasicStroke(2f));
            g2.drawRoundRect(pad, pad, width, height, 18, 18);

            g2.setColor(new Color(70, 100, 130));
            g2.drawLine(pad + width / 2, pad, pad + width / 2, pad + height);
            g2.drawLine(pad, pad + height / 2, pad + width, pad + height / 2);

            int robotX = pad + (int) Math.round(clamp(poseX_m / FIELD_LENGTH_M, 0.0, 1.0) * width);
            int robotY = pad + height - (int) Math.round(clamp(poseY_m / FIELD_WIDTH_M, 0.0, 1.0) * height);

            g2.setColor("AUTONOMOUS".equals(mode) ? WARN : OK);
            g2.fillOval(robotX - 10, robotY - 10, 20, 20);

            double radians = Math.toRadians(headingDeg);
            int arrowX = robotX + (int) Math.round(Math.cos(radians) * 28.0);
            int arrowY = robotY - (int) Math.round(Math.sin(radians) * 28.0);
            g2.setStroke(new BasicStroke(3f));
            g2.drawLine(robotX, robotY, arrowX, arrowY);

            g2.setColor(TEXT);
            g2.setFont(new Font("Segoe UI", Font.BOLD, 16));
            g2.drawString("Pose (" + ONE_DECIMAL.format(poseX_m) + ", "
                    + ONE_DECIMAL.format(poseY_m) + ")  Heading "
                    + ONE_DECIMAL.format(headingDeg) + " deg", pad, 22);
            g2.dispose();
        }

        private static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }

    static {
        UIManager.put("TabbedPane.foreground", TEXT);
        UIManager.put("TabbedPane.selected", CARD);
    }
}
