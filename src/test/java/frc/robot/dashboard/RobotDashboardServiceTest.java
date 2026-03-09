package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;

class RobotDashboardServiceTest {

    private NetworkTableInstance nt;
    private NetworkTable table;
    private TestActions actions;
    private RobotDashboardService service;

    private IntegerPublisher intakeHomeCmdPub;
    private IntegerPublisher zeroHeadingCmdPub;
    private IntegerPublisher level1ClimbCmdPub;
    private IntegerPublisher stopDriveCmdPub;
    private IntegerPublisher alignOnlyCmdPub;
    private IntegerPublisher alignShootCmdPub;
    private IntegerPublisher fallbackShootCmdPub;
    private IntegerPublisher calibrateCANcodersCmdPub;
    private IntegerPublisher swerveValidationCmdPub;
    private IntegerPublisher stopSwerveValidationCmdPub;
    private StringPublisher swerveValidationModuleCmdPub;
    private StringPublisher swerveValidationModeCmdPub;
    private IntegerPublisher selectAutoCmdPub;
    private StringPublisher selectAutoNameCmdPub;

    private StringSubscriber ackCommandSub;
    private StringSubscriber ackStatusSub;
    private IntegerSubscriber ackSeqSub;
    private StringSubscriber ackMessageSub;
    private StringSubscriber selectedAutoNameSub;
    private StringSubscriber selectedAutoSourceSub;
    private StringArraySubscriber autoOptionsSub;
    private DoubleSubscriber robotTimestampSub;
    private StringSubscriber driverButtonsSub;
    private StringSubscriber operatorButtonsSub;
    private IntegerSubscriber controlEventSeqSub;
    private DoubleSubscriber controlEventTimestampSub;
    private StringSubscriber controlEventMessageSub;

    @BeforeEach
    void setUp() {
        nt = NetworkTableInstance.create();
        table = nt.getTable("Dashboard");
        actions = new TestActions();
        service = new RobotDashboardService(actions, nt);

        intakeHomeCmdPub = table.getIntegerTopic("cmd/intake_home_seq").publish();
        zeroHeadingCmdPub = table.getIntegerTopic("cmd/zero_heading_seq").publish();
        level1ClimbCmdPub = table.getIntegerTopic("cmd/level1_climb_seq").publish();
        stopDriveCmdPub = table.getIntegerTopic("cmd/stop_drive_seq").publish();
        alignOnlyCmdPub = table.getIntegerTopic("cmd/align_only_seq").publish();
        alignShootCmdPub = table.getIntegerTopic("cmd/align_shoot_seq").publish();
        fallbackShootCmdPub = table.getIntegerTopic("cmd/fallback_shoot_seq").publish();
        calibrateCANcodersCmdPub = table.getIntegerTopic("cmd/calibrate_cancoders_seq").publish();
        swerveValidationCmdPub = table.getIntegerTopic("cmd/swerve_validation_seq").publish();
        stopSwerveValidationCmdPub = table.getIntegerTopic("cmd/stop_swerve_validation_seq").publish();
        swerveValidationModuleCmdPub = table.getStringTopic("cmd/swerve_validation_module").publish();
        swerveValidationModeCmdPub = table.getStringTopic("cmd/swerve_validation_mode").publish();
        selectAutoCmdPub = table.getIntegerTopic("cmd/select_auto_seq").publish();
        selectAutoNameCmdPub = table.getStringTopic("cmd/select_auto_name").publish();

        ackCommandSub = table.getStringTopic("ack/last_command").subscribe("");
        ackStatusSub = table.getStringTopic("ack/last_status").subscribe("");
        ackSeqSub = table.getIntegerTopic("ack/last_seq").subscribe(0);
        ackMessageSub = table.getStringTopic("ack/message").subscribe("");
        selectedAutoNameSub = table.getStringTopic("auto/selected_name").subscribe("");
        selectedAutoSourceSub = table.getStringTopic("auto/selected_source").subscribe("");
        autoOptionsSub = table.getStringArrayTopic("auto/options").subscribe(new String[0]);
        robotTimestampSub = table.getDoubleTopic("robot/timestamp_sec").subscribe(0.0);
        driverButtonsSub = table.getStringTopic("controls/driver_buttons_active").subscribe("--");
        operatorButtonsSub = table.getStringTopic("controls/operator_buttons_active").subscribe("--");
        controlEventSeqSub = table.getIntegerTopic("controls/last_event_seq").subscribe(0);
        controlEventTimestampSub = table.getDoubleTopic("controls/last_event_timestamp_sec").subscribe(0.0);
        controlEventMessageSub = table.getStringTopic("controls/last_event_message").subscribe("");
    }

    @AfterEach
    void tearDown() {
        nt.close();
    }

    @Test
    void intakeHomeRejectedWhenDisabled() {
        intakeHomeCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 1.0));
        nt.flush();

        assertEquals("intake_home", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Requires enabled teleop/test (disabled mode blocks motor output)", ackMessageSub.get());
        assertEquals(0, actions.intakeHomeCalls);
    }

    @Test
    void intakeHomeAcceptedWhenEnabledTeleop() {
        intakeHomeCmdPub.set(2);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, false, 2.0));
        nt.flush();

        assertEquals("intake_home", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(2L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.intakeHomeCalls);
    }

    @Test
    void intakeHomeAcceptedWhenEnabledTestMode() {
        intakeHomeCmdPub.set(3);
        nt.flush();

        service.periodic(snapshot("TEST", true, false, 2.1));
        nt.flush();

        assertEquals("intake_home", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(3L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.intakeHomeCalls);
    }

    // @Test
    // void level1ClimbRejectedClimberDisabled() {
    //     level1ClimbCmdPub.set(1);
    //     nt.flush();

    //     // --- CLIMBER DISABLED: always rejected regardless of arm gate ---
    //     service.periodic(snapshot("TELEOP", true, false, 3.0));
    //     nt.flush();

    //     assertEquals("level1_climb", ackCommandSub.get());
    //     assertEquals("REJECTED", ackStatusSub.get());
    //     assertEquals("Climber disabled \u2014 no hardware installed", ackMessageSub.get());
    //     assertEquals(0, actions.level1ClimbCalls);
    // }

    // @Test
    // void level1ClimbRejectedEvenWhenArmed() {
    //     level1ClimbCmdPub.set(2);
    //     nt.flush();

    //     // --- CLIMBER DISABLED: always rejected even with arm gate ---
    //     service.periodic(snapshot("TELEOP", true, true, 3.1));
    //     nt.flush();

    //     assertEquals("level1_climb", ackCommandSub.get());
    //     assertEquals("REJECTED", ackStatusSub.get());
    //     assertEquals("Climber disabled \u2014 no hardware installed", ackMessageSub.get());
    //     assertEquals(0, actions.level1ClimbCalls);
    // }

    @Test
    void zeroHeadingAcceptedWhenDisabled() {
        zeroHeadingCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 3.5));
        nt.flush();

        assertEquals("zero_heading", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.zeroHeadingCalls);
    }

    @Test
    void alignShootAcceptedWhenEnabledTeleop() {
        alignShootCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, false, 3.75));
        nt.flush();

        assertEquals("align_shoot", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.alignShootCalls);
    }

    @Test
    void alignOnlyAcceptedWhenEnabledTeleop() {
        alignOnlyCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, false, 3.78));
        nt.flush();

        assertEquals("align_only", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.alignOnlyCalls);
    }

    @Test
    void alignOnlyRejectedWhenDisabled() {
        alignOnlyCmdPub.set(2);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 3.79));
        nt.flush();

        assertEquals("align_only", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals(2L, ackSeqSub.get());
        assertEquals("Only allowed in enabled teleop", ackMessageSub.get());
        assertEquals(0, actions.alignOnlyCalls);
    }

    @Test
    void alignShootRejectedWhenDisabled() {
        alignShootCmdPub.set(2);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 3.8));
        nt.flush();

        assertEquals("align_shoot", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals(2L, ackSeqSub.get());
        assertEquals("Only allowed in enabled teleop", ackMessageSub.get());
        assertEquals(0, actions.alignShootCalls);
    }

    @Test
    void fallbackShootAcceptedWhenEnabledTeleop() {
        fallbackShootCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, false, 3.9));
        nt.flush();

        assertEquals("fallback_shoot", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.fallbackShootCalls);
    }

    @Test
    void fallbackShootRejectedWhenAutonomous() {
        fallbackShootCmdPub.set(2);
        nt.flush();

        service.periodic(snapshot("AUTONOMOUS", true, false, 3.95));
        nt.flush();

        assertEquals("fallback_shoot", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals(2L, ackSeqSub.get());
        assertEquals("Only allowed in enabled teleop", ackMessageSub.get());
        assertEquals(0, actions.fallbackShootCalls);
    }

    @Test
    void commandSequenceRunsOnlyOncePerSequenceValue() {
        stopDriveCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, false, 4.0));
        service.periodic(snapshot("TELEOP", true, false, 4.02));
        nt.flush();

        assertEquals(1, actions.stopDriveCalls);
        assertEquals(1L, ackSeqSub.get());
    }

    @Test
    void publishesRobotTimestampHeartbeat() {
        service.periodic(snapshot("TELEOP", true, true, 42.5));
        nt.flush();
        assertEquals(42.5, robotTimestampSub.get(), 1e-9);
    }

    @Test
    void publishesControllerDiagnostics() {
        service.periodic(snapshot(
                "TELEOP",
                true,
                true,
                50.0,
                "Y, RT",
                "A, START, BACK",
                17,
                49.95,
                "Operator:RT -> AlignAndShoot requested"));
        nt.flush();

        assertEquals("Y, RT", driverButtonsSub.get());
        assertEquals("A, START, BACK", operatorButtonsSub.get());
        assertEquals(17L, controlEventSeqSub.get());
        assertEquals(49.95, controlEventTimestampSub.get(), 1e-9);
        assertEquals("Operator:RT -> AlignAndShoot requested", controlEventMessageSub.get());
    }

    @Test
    void publishesAutoSelectionMetadata() {
        service.periodic(snapshot("DISABLED", false, false, 60.0));
        nt.flush();

        assertEquals("Do Nothing", selectedAutoNameSub.get());
        assertEquals("SMARTDASHBOARD", selectedAutoSourceSub.get());
        assertEquals(3, autoOptionsSub.get().length);
        assertEquals("Do Nothing", autoOptionsSub.get()[0]);
        assertEquals("Taxi Only", autoOptionsSub.get()[1]);
        assertEquals("Calibrate CANcoders", autoOptionsSub.get()[2]);
    }

    @Test
    void autoSelectionAcceptedWhenDisabledAndKnown() {
        selectAutoNameCmdPub.set("Taxi Only");
        selectAutoCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 61.0));
        nt.flush();

        assertEquals("select_auto", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Selected Taxi Only", ackMessageSub.get());
        assertEquals("Taxi Only", actions.lastSelectedAutoName);
    }

    @Test
    void autoSelectionRejectedWhenEnabled() {
        selectAutoNameCmdPub.set("Taxi Only");
        selectAutoCmdPub.set(2);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, false, 62.0));
        nt.flush();

        assertEquals("select_auto", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals(2L, ackSeqSub.get());
        assertEquals("Only allowed while robot is disabled", ackMessageSub.get());
        assertEquals(0, actions.selectAutoCalls);
    }

    @Test
    void autoSelectionRejectedWhenUnknown() {
        selectAutoNameCmdPub.set("Nope");
        selectAutoCmdPub.set(3);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 63.0));
        nt.flush();

        assertEquals("select_auto", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals(3L, ackSeqSub.get());
        assertEquals("Unknown auto: Nope", ackMessageSub.get());
        assertEquals(0, actions.selectAutoCalls);
    }

    @Test
    void calibrateCANcodersAcceptedInDisabled() {
        calibrateCANcodersCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 64.0));
        nt.flush();

        assertEquals("calibrate_cancoders", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.calibrateCANcodersCalls);
    }

    @Test
    void swerveValidationAcceptedWhenEnabledTest() {
        swerveValidationModuleCmdPub.set("FR");
        swerveValidationModeCmdPub.set("STEER_POSITIVE");
        swerveValidationCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("TEST", true, false, 65.0));
        nt.flush();

        assertEquals("swerve_validation", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Accepted FR / STEER_POSITIVE", ackMessageSub.get());
        assertEquals(1, actions.swerveValidationCalls);
        assertEquals("FR", actions.lastValidationModule);
        assertEquals("STEER_POSITIVE", actions.lastValidationMode);
    }

    @Test
    void swerveValidationRejectedWhenModeUnknown() {
        swerveValidationModuleCmdPub.set("FR");
        swerveValidationModeCmdPub.set("NOPE");
        swerveValidationCmdPub.set(2);
        nt.flush();

        service.periodic(snapshot("TEST", true, false, 66.0));
        nt.flush();

        assertEquals("swerve_validation", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals(2L, ackSeqSub.get());
        assertEquals("Unknown module or mode", ackMessageSub.get());
        assertEquals(0, actions.swerveValidationCalls);
    }

    @Test
    void stopSwerveValidationAcceptedWhenConnected() {
        stopSwerveValidationCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("DISABLED", false, false, 67.0));
        nt.flush();

        assertEquals("stop_swerve_validation", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(1L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.stopSwerveValidationCalls);
    }

    private static DashboardSnapshot snapshot(
            String mode,
            boolean enabled,
            boolean climberArmed,
            double timestampSec) {
        return snapshot(mode, enabled, climberArmed, timestampSec, "--", "--", 0, 0.0, "");
    }

    private static DashboardSnapshot snapshot(
            String mode,
            boolean enabled,
            boolean climberArmed,
            double timestampSec,
            String driverButtonsActive,
            String operatorButtonsActive,
            long controlEventSeq,
            double controlEventTimestampSec,
            String controlEventMessage) {
        return new DashboardSnapshot(
                timestampSec,
                mode,
                enabled,
                "BLUE",
                135.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                false,
                false,
                false,
                0.0,
                0.0,
                0.0,
                0.0,
                climberArmed,
                0.0,
                0.0,
                "IDLE",
                false,
                false,
                false,
                false,
                Double.NaN,
                Double.NaN,
                "",
                false,
                "",
                // 2026 REBUILT: HUB shift activity
                true,
                25.0,
                // System health
                12.5,
                false,
                false,
                // Auto
                "Do Nothing",
                "SMARTDASHBOARD",
                new String[] {"Do Nothing", "Taxi Only", "Calibrate CANcoders"},
                false,
                // Expected auto starting pose
                Double.NaN,
                Double.NaN,
                Double.NaN,
                // Match info
                0,
                "",
                // Camera
                true,
                "STREAMING",
                0,
                "Logitech C920",
                "/dev/video0",
                "dev=0 Logitech C920 @ /dev/video0",
                "",
                42,
                12.0,
                -1,
                false,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                // CAN health
                0.0,
                0,
                0,
                // Swerve angles
                0.0, 0.0, 0.0, 0.0,
                // CANCoder health (pos, absRaw, ok)
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                true, true, true, true,
                // Motor temps
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0,
                // Controller diagnostics
                driverButtonsActive,
                operatorButtonsActive,
                controlEventSeq,
                controlEventTimestampSec,
                controlEventMessage,
                false,
                "NONE",
                "--",
                "IDLE",
                "Idle",
                0.0,
                0.0,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN);
    }

    private static final class TestActions implements RobotDashboardService.Actions {
        int zeroHeadingCalls;
        int stopDriveCalls;
        int intakeHomeCalls;
        int alignOnlyCalls;
        int alignShootCalls;
        int fallbackShootCalls;
        int level1ClimbCalls;
        int calibrateCANcodersCalls;
        int swerveValidationCalls;
        int stopSwerveValidationCalls;
        int selectAutoCalls;
        String lastSelectedAutoName;
        String lastValidationModule;
        String lastValidationMode;

        @Override
        public void zeroHeading() {
            zeroHeadingCalls++;
        }

        @Override
        public void stopDrive() {
            stopDriveCalls++;
        }

        @Override
        public void scheduleIntakeHome() {
            intakeHomeCalls++;
        }

        @Override
        public void scheduleAlignAndShoot() {
            alignShootCalls++;
        }

        @Override
        public void scheduleAlignOnly() {
            alignOnlyCalls++;
        }

        @Override
        public void scheduleFallbackShoot() {
            fallbackShootCalls++;
        }

        @Override
        public void scheduleLevel1Climb() {
            level1ClimbCalls++;
        }

        @Override
        public void scheduleCANcoderCalibration() {
            calibrateCANcodersCalls++;
        }

        @Override
        public void requestSwerveValidation(String moduleName, String modeName) {
            swerveValidationCalls++;
            lastValidationModule = moduleName;
            lastValidationMode = modeName;
        }

        @Override
        public void stopSwerveValidation() {
            stopSwerveValidationCalls++;
        }

        @Override
        public void selectAutoByName(String autoName) {
            selectAutoCalls++;
            lastSelectedAutoName = autoName;
        }
    }
}
