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

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;
import javax.swing.SwingConstants;
import javax.swing.Timer;
import javax.swing.UIManager;

public class DashboardFrame extends JFrame {

    private static final Color BG = new Color(14, 20, 30);
    private static final Color CARD = new Color(24, 33, 48);
    private static final Color TEXT = new Color(240, 243, 247);
    private static final Color MUTED = new Color(161, 174, 194);
    private static final Color OK = new Color(28, 166, 80);
    private static final Color WARN = new Color(214, 141, 40);
    private static final Color BAD = new Color(201, 57, 57);

    private static final DecimalFormat ONE_DECIMAL = new DecimalFormat("0.0");

    private final DashboardNtClient client;

    private final JLabel connectionLabel = new JLabel("Disconnected");
    private final JLabel modeLabel = new JLabel("Mode: UNKNOWN");
    private final JLabel allianceLabel = new JLabel("Alliance: UNKNOWN");
    private final JLabel matchTimeLabel = new JLabel("Match: --.-");

    private final JLabel readyLabel = new JLabel("NOT READY", SwingConstants.CENTER);
    private final JLabel readyReasonLabel = new JLabel("Reason: --");
    private final JLabel alignPhaseLabel = new JLabel("Align phase: IDLE");
    private final JLabel yawLabel = new JLabel("Yaw: --");
    private final JLabel pitchLabel = new JLabel("Pitch: --");
    private final FieldPanel fieldPanel = new FieldPanel();

    private final JLabel shooterLabel = new JLabel("Left -- / Right -- RPS");
    private final JLabel shooterAtSpeedLabel = new JLabel("At speed: false");
    private final JLabel intakeLabel = new JLabel("Homed: false  Tilt: -- deg");
    private final JLabel conveyorLabel = new JLabel("Feeder -- A  Hopper -- A");
    private final JLabel climberLabel = new JLabel("Armed: false  Pos: -- rot");
    private final JLabel visionLabel = new JLabel("Target: false  Feasible: false");
    private final JLabel abortLabel = new JLabel("Last abort: --");
    private final JLabel operatorVisionLabel = new JLabel("Target: false  Feasible: false");
    private final JLabel operatorPhaseLabel = new JLabel("Align phase: IDLE");
    private final JLabel operatorReadyLabel = new JLabel("NOT READY");
    private final JLabel operatorReadyReasonLabel = new JLabel("Reason: --");

    private final JTextArea pitRawArea = new JTextArea();
    private final JLabel ackLabel = new JLabel("Ack: --");

    public DashboardFrame(DashboardNtClient client) {
        super("3318 Competition Dashboard");
        this.client = client;

        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setMinimumSize(new Dimension(1180, 760));
        getContentPane().setBackground(BG);
        setLayout(new BorderLayout(10, 10));

        add(buildHeader(), BorderLayout.NORTH);
        add(buildTabs(), BorderLayout.CENTER);

        Timer timer = new Timer(100, e -> refresh());
        timer.start();
    }

    private JPanel buildHeader() {
        JPanel header = new JPanel(new GridLayout(1, 4, 8, 8));
        header.setBorder(BorderFactory.createEmptyBorder(10, 10, 0, 10));
        header.setBackground(BG);

        styleHeaderLabel(connectionLabel);
        styleHeaderLabel(modeLabel);
        styleHeaderLabel(allianceLabel);
        styleHeaderLabel(matchTimeLabel);

        header.add(connectionLabel);
        header.add(modeLabel);
        header.add(allianceLabel);
        header.add(matchTimeLabel);
        return header;
    }

    private JTabbedPane buildTabs() {
        JTabbedPane tabs = new JTabbedPane();
        tabs.setBackground(CARD);
        tabs.setForeground(TEXT);
        tabs.addTab("Driver", buildDriverTab());
        tabs.addTab("Operator", buildOperatorTab());
        tabs.addTab("Pit", buildPitTab());
        return tabs;
    }

    private JPanel buildDriverTab() {
        JPanel root = new JPanel(new BorderLayout(10, 10));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        fieldPanel.setPreferredSize(new Dimension(760, 560));
        root.add(fieldPanel, BorderLayout.CENTER);

        JPanel side = new JPanel(new GridLayout(5, 1, 8, 8));
        side.setPreferredSize(new Dimension(360, 560));
        side.setBackground(BG);

        readyLabel.setFont(new Font("Dialog", Font.BOLD, 34));
        readyLabel.setOpaque(true);
        readyLabel.setBackground(BAD);
        readyLabel.setForeground(Color.WHITE);
        readyLabel.setBorder(BorderFactory.createEmptyBorder(18, 8, 18, 8));

        side.add(wrapCard("Shot Readiness", readyLabel, readyReasonLabel));
        side.add(wrapCard("Align Pipeline", alignPhaseLabel, yawLabel, pitchLabel, visionLabel, abortLabel));
        side.add(wrapCard("Command Ack", ackLabel));
        side.add(buildDriverActionCard());
        side.add(buildOperatorActionCard());

        root.add(side, BorderLayout.EAST);
        return root;
    }

    private JPanel buildOperatorTab() {
        JPanel root = new JPanel(new GridLayout(2, 3, 10, 10));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        root.add(wrapCard("Shooter", shooterLabel, shooterAtSpeedLabel));
        root.add(wrapCard("Intake", intakeLabel));
        root.add(wrapCard("Conveyor", conveyorLabel));
        root.add(wrapCard("Climber", climberLabel));
        root.add(wrapCard("Vision", operatorVisionLabel, operatorPhaseLabel));
        root.add(wrapCard("Readiness", operatorReadyLabel, operatorReadyReasonLabel));
        return root;
    }

    private JPanel buildPitTab() {
        JPanel root = new JPanel(new BorderLayout(8, 8));
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.setBackground(BG);

        pitRawArea.setEditable(false);
        pitRawArea.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 14));
        pitRawArea.setBackground(new Color(10, 14, 20));
        pitRawArea.setForeground(TEXT);
        pitRawArea.setLineWrap(true);
        pitRawArea.setWrapStyleWord(true);
        pitRawArea.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        root.add(pitRawArea, BorderLayout.CENTER);

        return root;
    }

    private JPanel buildDriverActionCard() {
        JPanel panel = new JPanel(new GridLayout(2, 2, 8, 8));
        panel.setBackground(CARD);

        panel.add(createCommandButton("Zero Heading", DashboardNtClient.DashboardCommand.ZERO_HEADING));
        panel.add(createCommandButton("Stop Drive", DashboardNtClient.DashboardCommand.STOP_DRIVE));
        panel.add(createCommandButton("Align + Shoot", DashboardNtClient.DashboardCommand.ALIGN_SHOOT));
        panel.add(createCommandButton("Fallback Shot", DashboardNtClient.DashboardCommand.FALLBACK_SHOOT));

        JPanel container = wrapCard("Driver Actions", panel);
        return container;
    }

    private JPanel buildOperatorActionCard() {
        JPanel panel = new JPanel(new GridLayout(1, 2, 8, 8));
        panel.setBackground(CARD);

        panel.add(createCommandButton("Intake Home", DashboardNtClient.DashboardCommand.INTAKE_HOME));
        panel.add(createCommandButton("Level 1 Climb", DashboardNtClient.DashboardCommand.LEVEL1_CLIMB));

        JPanel container = wrapCard("Operator Actions", panel);
        return container;
    }

    private JButton createCommandButton(String text, DashboardNtClient.DashboardCommand command) {
        JButton button = new JButton(text);
        button.setFocusPainted(false);
        button.setFont(new Font("Dialog", Font.BOLD, 16));
        button.setBackground(new Color(50, 83, 120));
        button.setForeground(TEXT);
        button.addActionListener(e -> client.sendCommand(command));
        return button;
    }

    private JPanel wrapCard(String title, JLabel... labels) {
        JPanel panel = new JPanel(new GridBagLayout());
        panel.setBackground(CARD);
        panel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(new Color(43, 58, 79), 1),
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
        titleLabel.setFont(new Font("Dialog", Font.BOLD, 14));
        panel.add(titleLabel, gbc);

        for (JLabel label : labels) {
            gbc.gridy++;
            label.setForeground(TEXT);
            label.setFont(new Font("Dialog", Font.PLAIN, 20));
            panel.add(label, gbc);
        }
        return panel;
    }

    private JPanel wrapCard(String title, JPanel content) {
        JPanel panel = new JPanel(new BorderLayout(6, 6));
        panel.setBackground(CARD);
        panel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(new Color(43, 58, 79), 1),
                BorderFactory.createEmptyBorder(10, 10, 10, 10)));

        JLabel titleLabel = new JLabel(title);
        titleLabel.setForeground(MUTED);
        titleLabel.setFont(new Font("Dialog", Font.BOLD, 14));
        panel.add(titleLabel, BorderLayout.NORTH);
        panel.add(content, BorderLayout.CENTER);
        return panel;
    }

    private void styleHeaderLabel(JLabel label) {
        label.setOpaque(true);
        label.setBackground(CARD);
        label.setForeground(TEXT);
        label.setFont(new Font("Dialog", Font.BOLD, 18));
        label.setHorizontalAlignment(SwingConstants.CENTER);
        label.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
    }

    private void refresh() {
        DashboardData data = client.read();

        connectionLabel.setText(data.connected() ? "Connected" : "Disconnected");
        connectionLabel.setBackground(data.connected() ? OK : BAD);

        modeLabel.setText("Mode: " + data.mode() + (data.enabled() ? " (ENABLED)" : " (DISABLED)"));
        allianceLabel.setText("Alliance: " + data.alliance());
        matchTimeLabel.setText("Match: " + ONE_DECIMAL.format(data.matchTimeSec()) + "s");

        readyLabel.setText(data.readyToScore() ? "READY" : "NOT READY");
        readyLabel.setBackground(data.readyToScore() ? OK : BAD);
        readyReasonLabel.setText("Reason: " + data.readyReason());

        alignPhaseLabel.setText("Align phase: " + data.alignState());
        yawLabel.setText("Yaw: " + formatMaybe(data.alignYawDeg()) + " deg");
        pitchLabel.setText("Pitch: " + formatMaybe(data.alignPitchDeg()) + " deg");
        visionLabel.setText("Target: " + data.alignHasTarget() + "  Feasible: " + data.alignGeometryFeasible());
        abortLabel.setText("Last abort: " + sanitize(data.alignAbortReason()));
        operatorVisionLabel.setText("Target: " + data.alignHasTarget() + "  Feasible: " + data.alignGeometryFeasible());
        operatorPhaseLabel.setText("Align phase: " + data.alignState());
        operatorReadyLabel.setText(data.readyToScore() ? "READY" : "NOT READY");
        operatorReadyReasonLabel.setText("Reason: " + data.readyReason());

        shooterLabel.setText("Left " + ONE_DECIMAL.format(data.shooterLeftRps())
                + " / Right " + ONE_DECIMAL.format(data.shooterRightRps()) + " RPS");
        shooterAtSpeedLabel.setText("At speed: " + data.shooterAtSpeed());
        intakeLabel.setText("Homed: " + data.intakeHomed()
                + "  Limit: " + data.intakeLimitSwitchPressed()
                + "  Tilt: " + ONE_DECIMAL.format(data.intakeTiltDeg()) + " deg");
        conveyorLabel.setText("Feeder " + ONE_DECIMAL.format(data.feederCurrentAmps())
                + " A  Hopper " + ONE_DECIMAL.format(data.hopperCurrentAmps()) + " A");
        climberLabel.setText("Armed: " + data.climberArmed()
                + "  Pos: " + ONE_DECIMAL.format(data.climberPositionRot())
                + " rot  I: " + ONE_DECIMAL.format(data.climberCurrentAmps()) + " A");

        ackLabel.setText("Ack: " + sanitize(data.ackLastCommand())
                + " #" + data.ackSeq()
                + "  " + sanitize(data.ackStatus())
                + "  " + sanitize(data.ackMessage()));
        ackLabel.setForeground("OK".equals(data.ackStatus()) ? OK : WARN);

        fieldPanel.updatePose(data.poseX_m(), data.poseY_m(), data.headingDeg(), data.mode());

        pitRawArea.setText(buildRawText(data));
    }

    private String buildRawText(DashboardData data) {
        StringBuilder sb = new StringBuilder();
        sb.append("Connection: ").append(data.connected()).append('\n');
        sb.append("Mode: ").append(data.mode()).append(" enabled=").append(data.enabled()).append('\n');
        sb.append("Alliance: ").append(data.alliance()).append("  MatchTime=").append(data.matchTimeSec()).append('\n');
        sb.append("Pose: x=").append(data.poseX_m()).append(" y=").append(data.poseY_m())
                .append(" heading=").append(data.headingDeg()).append('\n');
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
        sb.append("Ack: cmd='").append(sanitize(data.ackLastCommand()))
                .append("' seq=").append(data.ackSeq())
                .append(" status='").append(sanitize(data.ackStatus()))
                .append("' msg='").append(sanitize(data.ackMessage()))
                .append("' ts=").append(data.ackTimestampSec()).append('\n');
        return sb.toString();
    }

    private static String sanitize(String value) {
        return value == null || value.isBlank() ? "--" : value;
    }

    private static String formatMaybe(double value) {
        return Double.isFinite(value) ? ONE_DECIMAL.format(value) : "--";
    }

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

            g2.setColor(new Color(21, 45, 74));
            g2.fillRoundRect(pad, pad, width, height, 18, 18);
            g2.setColor(new Color(85, 122, 165));
            g2.setStroke(new BasicStroke(2f));
            g2.drawRoundRect(pad, pad, width, height, 18, 18);

            g2.drawLine(pad + width / 2, pad, pad + width / 2, pad + height);

            int robotX = pad + (int) Math.round(clamp(poseX_m / FIELD_LENGTH_M, 0.0, 1.0) * width);
            int robotY = pad + height - (int) Math.round(clamp(poseY_m / FIELD_WIDTH_M, 0.0, 1.0) * height);

            g2.setColor("AUTONOMOUS".equals(mode) ? WARN : OK);
            g2.fillOval(robotX - 9, robotY - 9, 18, 18);

            double radians = Math.toRadians(headingDeg);
            int arrowX = robotX + (int) Math.round(Math.cos(radians) * 24.0);
            int arrowY = robotY - (int) Math.round(Math.sin(radians) * 24.0);
            g2.drawLine(robotX, robotY, arrowX, arrowY);

            g2.setColor(TEXT);
            g2.setFont(new Font("Dialog", Font.BOLD, 16));
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
