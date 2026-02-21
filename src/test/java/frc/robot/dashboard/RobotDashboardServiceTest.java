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
    private IntegerPublisher level1ClimbCmdPub;
    private IntegerPublisher stopDriveCmdPub;

    private StringSubscriber ackCommandSub;
    private StringSubscriber ackStatusSub;
    private IntegerSubscriber ackSeqSub;
    private StringSubscriber ackMessageSub;
    private DoubleSubscriber robotTimestampSub;

    @BeforeEach
    void setUp() {
        nt = NetworkTableInstance.create();
        table = nt.getTable("Dashboard");
        actions = new TestActions();
        service = new RobotDashboardService(actions, nt);

        intakeHomeCmdPub = table.getIntegerTopic("cmd/intake_home_seq").publish();
        level1ClimbCmdPub = table.getIntegerTopic("cmd/level1_climb_seq").publish();
        stopDriveCmdPub = table.getIntegerTopic("cmd/stop_drive_seq").publish();

        ackCommandSub = table.getStringTopic("ack/last_command").subscribe("");
        ackStatusSub = table.getStringTopic("ack/last_status").subscribe("");
        ackSeqSub = table.getIntegerTopic("ack/last_seq").subscribe(0);
        ackMessageSub = table.getStringTopic("ack/message").subscribe("");
        robotTimestampSub = table.getDoubleTopic("robot/timestamp_sec").subscribe(0.0);
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

    private static DashboardSnapshot snapshot(
            String mode,
            boolean enabled,
            boolean climberArmed,
            double timestampSec) {
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
                "");
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
