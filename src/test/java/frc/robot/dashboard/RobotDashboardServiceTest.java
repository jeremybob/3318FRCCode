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
    private IntegerPublisher alignShootCmdPub;
    private IntegerPublisher fallbackShootCmdPub;

    private StringSubscriber ackCommandSub;
    private StringSubscriber ackStatusSub;
    private IntegerSubscriber ackSeqSub;
    private StringSubscriber ackMessageSub;
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
        alignShootCmdPub = table.getIntegerTopic("cmd/align_shoot_seq").publish();
        fallbackShootCmdPub = table.getIntegerTopic("cmd/fallback_shoot_seq").publish();

        ackCommandSub = table.getStringTopic("ack/last_command").subscribe("");
        ackStatusSub = table.getStringTopic("ack/last_status").subscribe("");
        ackSeqSub = table.getIntegerTopic("ack/last_seq").subscribe(0);
        ackMessageSub = table.getStringTopic("ack/message").subscribe("");
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

    @Test
    void level1ClimbRejectedWithoutArmGate() {
        level1ClimbCmdPub.set(1);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, false, 3.0));
        nt.flush();

        assertEquals("level1_climb", ackCommandSub.get());
        assertEquals("REJECTED", ackStatusSub.get());
        assertEquals("Requires enabled teleop and climber arm gate", ackMessageSub.get());
        assertEquals(0, actions.level1ClimbCalls);
    }

    @Test
    void level1ClimbAcceptedWhenTeleopAndArmed() {
        level1ClimbCmdPub.set(2);
        nt.flush();

        service.periodic(snapshot("TELEOP", true, true, 3.1));
        nt.flush();

        assertEquals("level1_climb", ackCommandSub.get());
        assertEquals("OK", ackStatusSub.get());
        assertEquals(2L, ackSeqSub.get());
        assertEquals("Accepted", ackMessageSub.get());
        assertEquals(1, actions.level1ClimbCalls);
    }

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
                // System health
                12.5,
                false,
                false,
                // Auto
                "Do Nothing",
                false,
                // Match info
                0,
                "",
                // Camera
                true,
                // CAN health
                0.0,
                0,
                0,
                // Swerve angles
                0.0, 0.0, 0.0, 0.0,
                // Motor temps
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0,
                // Controller diagnostics
                driverButtonsActive,
                operatorButtonsActive,
                controlEventSeq,
                controlEventTimestampSec,
                controlEventMessage);
    }

    private static final class TestActions implements RobotDashboardService.Actions {
        int zeroHeadingCalls;
        int stopDriveCalls;
        int intakeHomeCalls;
        int alignShootCalls;
        int fallbackShootCalls;
        int level1ClimbCalls;

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
        public void scheduleFallbackShoot() {
            fallbackShootCalls++;
        }

        @Override
        public void scheduleLevel1Climb() {
            level1ClimbCalls++;
        }
    }
}
