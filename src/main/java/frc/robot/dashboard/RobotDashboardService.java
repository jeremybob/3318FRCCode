package frc.robot.dashboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class RobotDashboardService {

    public interface Actions {
        void zeroHeading();
        void stopDrive();
        void scheduleIntakeHome();
        void scheduleAlignAndShoot();
        void scheduleFallbackShoot();
        void scheduleLevel1Climb();
    }

    private static final String CONTRACT_VERSION = "2026.1.0";

    private final Actions actions;

    private final StringPublisher contractVersionPub;

    private final StringPublisher robotModePub;
    private final BooleanPublisher robotEnabledPub;
    private final StringPublisher alliancePub;
    private final DoublePublisher matchTimePub;

    private final DoublePublisher poseXPub;
    private final DoublePublisher poseYPub;
    private final DoublePublisher headingPub;

    private final DoublePublisher shooterLeftRpsPub;
    private final DoublePublisher shooterRightRpsPub;
    private final BooleanPublisher shooterAtSpeedPub;

    private final BooleanPublisher intakeHomedPub;
    private final BooleanPublisher intakeLimitPub;
    private final DoublePublisher intakeTiltPub;
    private final DoublePublisher intakeRollerCurrentPub;

    private final DoublePublisher feederCurrentPub;
    private final DoublePublisher hopperCurrentPub;

    private final BooleanPublisher climberArmedPub;
    private final DoublePublisher climberPositionPub;
    private final DoublePublisher climberCurrentPub;

    private final StringPublisher alignStatePub;
    private final BooleanPublisher alignCommandActivePub;
    private final BooleanPublisher alignHasTargetPub;
    private final BooleanPublisher alignGeometryFeasiblePub;
    private final BooleanPublisher alignHasShootableTargetPub;
    private final DoublePublisher alignYawPub;
    private final DoublePublisher alignPitchPub;
    private final StringPublisher alignAbortReasonPub;

    private final BooleanPublisher readyToScorePub;
    private final StringPublisher readyReasonPub;

    private final StringPublisher lastCommandPub;
    private final StringPublisher lastStatusPub;
    private final IntegerPublisher lastSeqPub;
    private final StringPublisher lastMessagePub;
    private final DoublePublisher lastAckTimestampPub;

    private final IntegerSubscriber zeroHeadingCmdSub;
    private final IntegerSubscriber stopDriveCmdSub;
    private final IntegerSubscriber intakeHomeCmdSub;
    private final IntegerSubscriber alignShootCmdSub;
    private final IntegerSubscriber fallbackShootCmdSub;
    private final IntegerSubscriber level1ClimbCmdSub;

    private long zeroHeadingSeqSeen = 0;
    private long stopDriveSeqSeen = 0;
    private long intakeHomeSeqSeen = 0;
    private long alignShootSeqSeen = 0;
    private long fallbackShootSeqSeen = 0;
    private long level1ClimbSeqSeen = 0;

    public RobotDashboardService(Actions actions) {
        this.actions = actions;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Dashboard");

        contractVersionPub = table.getStringTopic("meta/contract_version").publish();

        robotModePub = table.getStringTopic("robot/mode").publish();
        robotEnabledPub = table.getBooleanTopic("robot/enabled").publish();
        alliancePub = table.getStringTopic("robot/alliance").publish();
        matchTimePub = table.getDoubleTopic("robot/match_time_sec").publish();

        poseXPub = table.getDoubleTopic("drive/pose_x_m").publish();
        poseYPub = table.getDoubleTopic("drive/pose_y_m").publish();
        headingPub = table.getDoubleTopic("drive/heading_deg").publish();

        shooterLeftRpsPub = table.getDoubleTopic("shooter/left_rps").publish();
        shooterRightRpsPub = table.getDoubleTopic("shooter/right_rps").publish();
        shooterAtSpeedPub = table.getBooleanTopic("shooter/at_speed").publish();

        intakeHomedPub = table.getBooleanTopic("intake/homed").publish();
        intakeLimitPub = table.getBooleanTopic("intake/limit_switch_pressed").publish();
        intakeTiltPub = table.getDoubleTopic("intake/tilt_deg").publish();
        intakeRollerCurrentPub = table.getDoubleTopic("intake/roller_current_amps").publish();

        feederCurrentPub = table.getDoubleTopic("feeder/current_amps").publish();
        hopperCurrentPub = table.getDoubleTopic("hopper/current_amps").publish();

        climberArmedPub = table.getBooleanTopic("climber/armed").publish();
        climberPositionPub = table.getDoubleTopic("climber/position_rot").publish();
        climberCurrentPub = table.getDoubleTopic("climber/current_amps").publish();

        alignStatePub = table.getStringTopic("align/state").publish();
        alignCommandActivePub = table.getBooleanTopic("align/command_active").publish();
        alignHasTargetPub = table.getBooleanTopic("align/has_target").publish();
        alignGeometryFeasiblePub = table.getBooleanTopic("align/geometry_feasible").publish();
        alignHasShootableTargetPub = table.getBooleanTopic("align/has_shootable_target").publish();
        alignYawPub = table.getDoubleTopic("align/yaw_deg").publish();
        alignPitchPub = table.getDoubleTopic("align/pitch_deg").publish();
        alignAbortReasonPub = table.getStringTopic("align/last_abort_reason").publish();

        readyToScorePub = table.getBooleanTopic("shot/ready").publish();
        readyReasonPub = table.getStringTopic("shot/ready_reason").publish();

        lastCommandPub = table.getStringTopic("ack/last_command").publish();
        lastStatusPub = table.getStringTopic("ack/last_status").publish();
        lastSeqPub = table.getIntegerTopic("ack/last_seq").publish();
        lastMessagePub = table.getStringTopic("ack/message").publish();
        lastAckTimestampPub = table.getDoubleTopic("ack/timestamp_sec").publish();

        zeroHeadingCmdSub = table.getIntegerTopic("cmd/zero_heading_seq").subscribe(0);
        stopDriveCmdSub = table.getIntegerTopic("cmd/stop_drive_seq").subscribe(0);
        intakeHomeCmdSub = table.getIntegerTopic("cmd/intake_home_seq").subscribe(0);
        alignShootCmdSub = table.getIntegerTopic("cmd/align_shoot_seq").subscribe(0);
        fallbackShootCmdSub = table.getIntegerTopic("cmd/fallback_shoot_seq").subscribe(0);
        level1ClimbCmdSub = table.getIntegerTopic("cmd/level1_climb_seq").subscribe(0);

        // Ignore any stale sequence values already present when robot code starts.
        zeroHeadingSeqSeen = zeroHeadingCmdSub.get();
        stopDriveSeqSeen = stopDriveCmdSub.get();
        intakeHomeSeqSeen = intakeHomeCmdSub.get();
        alignShootSeqSeen = alignShootCmdSub.get();
        fallbackShootSeqSeen = fallbackShootCmdSub.get();
        level1ClimbSeqSeen = level1ClimbCmdSub.get();
    }

    public void periodic(DashboardSnapshot snapshot) {
        publishSnapshot(snapshot);
        processCommandRequests(snapshot);
    }

    private void publishSnapshot(DashboardSnapshot snapshot) {
        contractVersionPub.set(CONTRACT_VERSION);

        robotModePub.set(snapshot.robotMode());
        robotEnabledPub.set(snapshot.enabled());
        alliancePub.set(snapshot.alliance());
        matchTimePub.set(snapshot.matchTimeSec());

        poseXPub.set(snapshot.poseX_m());
        poseYPub.set(snapshot.poseY_m());
        headingPub.set(snapshot.headingDeg());

        shooterLeftRpsPub.set(snapshot.shooterLeftRps());
        shooterRightRpsPub.set(snapshot.shooterRightRps());
        shooterAtSpeedPub.set(snapshot.shooterAtSpeed());

        intakeHomedPub.set(snapshot.intakeHomed());
        intakeLimitPub.set(snapshot.intakeLimitSwitchPressed());
        intakeTiltPub.set(snapshot.intakeTiltDeg());
        intakeRollerCurrentPub.set(snapshot.intakeRollerCurrentAmps());

        feederCurrentPub.set(snapshot.feederCurrentAmps());
        hopperCurrentPub.set(snapshot.hopperCurrentAmps());

        climberArmedPub.set(snapshot.climberArmed());
        climberPositionPub.set(snapshot.climberPositionRot());
        climberCurrentPub.set(snapshot.climberCurrentAmps());

        alignStatePub.set(snapshot.alignState());
        alignCommandActivePub.set(snapshot.alignCommandActive());
        alignHasTargetPub.set(snapshot.alignHasTarget());
        alignGeometryFeasiblePub.set(snapshot.alignGeometryFeasible());
        alignHasShootableTargetPub.set(snapshot.alignHasShootableTarget());
        alignYawPub.set(snapshot.alignYawDeg());
        alignPitchPub.set(snapshot.alignPitchDeg());
        alignAbortReasonPub.set(snapshot.alignAbortReason());

        readyToScorePub.set(snapshot.readyToScore());
        readyReasonPub.set(snapshot.readyReason());
    }

    private void processCommandRequests(DashboardSnapshot snapshot) {
        boolean disabled = "DISABLED".equals(snapshot.robotMode());
        boolean teleopEnabled = snapshot.enabled() && "TELEOP".equals(snapshot.robotMode());

        zeroHeadingSeqSeen = runCommandIfNew(
                zeroHeadingCmdSub,
                zeroHeadingSeqSeen,
                "zero_heading",
                actions::zeroHeading,
                disabled || teleopEnabled,
                "Only allowed in disabled or teleop mode",
                snapshot.timestampSec());

        stopDriveSeqSeen = runCommandIfNew(
                stopDriveCmdSub,
                stopDriveSeqSeen,
                "stop_drive",
                actions::stopDrive,
                true,
                "Accepted",
                snapshot.timestampSec());

        intakeHomeSeqSeen = runCommandIfNew(
                intakeHomeCmdSub,
                intakeHomeSeqSeen,
                "intake_home",
                actions::scheduleIntakeHome,
                disabled,
                "Only allowed while disabled",
                snapshot.timestampSec());

        alignShootSeqSeen = runCommandIfNew(
                alignShootCmdSub,
                alignShootSeqSeen,
                "align_shoot",
                actions::scheduleAlignAndShoot,
                teleopEnabled,
                "Only allowed in enabled teleop",
                snapshot.timestampSec());

        fallbackShootSeqSeen = runCommandIfNew(
                fallbackShootCmdSub,
                fallbackShootSeqSeen,
                "fallback_shoot",
                actions::scheduleFallbackShoot,
                teleopEnabled,
                "Only allowed in enabled teleop",
                snapshot.timestampSec());

        level1ClimbSeqSeen = runCommandIfNew(
                level1ClimbCmdSub,
                level1ClimbSeqSeen,
                "level1_climb",
                actions::scheduleLevel1Climb,
                teleopEnabled && snapshot.climberArmed(),
                "Requires enabled teleop and climber arm gate",
                snapshot.timestampSec());
    }

    private long runCommandIfNew(
            IntegerSubscriber subscriber,
            long lastSeen,
            String commandName,
            Runnable commandAction,
            boolean allowed,
            String rejectedReason,
            double timestampSec) {
        long seq = subscriber.get();
        if (seq <= lastSeen) {
            return lastSeen;
        }

        if (allowed) {
            commandAction.run();
            publishAck(commandName, "OK", seq, "Accepted", timestampSec);
        } else {
            publishAck(commandName, "REJECTED", seq, rejectedReason, timestampSec);
        }
        return seq;
    }

    private void publishAck(
            String commandName,
            String status,
            long seq,
            String message,
            double timestampSec) {
        lastCommandPub.set(commandName);
        lastStatusPub.set(status);
        lastSeqPub.set(seq);
        lastMessagePub.set(message);
        lastAckTimestampPub.set(timestampSec);
    }
}
