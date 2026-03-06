package frc.robot.dashboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;

public class RobotDashboardService {

    public interface Actions {
        void zeroHeading();
        void stopDrive();
        void scheduleIntakeHome();
        void scheduleAlignAndShoot();
        void scheduleFallbackShoot();
        void scheduleLevel1Climb();
        void selectAutoByName(String autoName);
    }

    private static final String CONTRACT_VERSION = "2026.8.0";

    private final Actions actions;

    private final StringPublisher contractVersionPub;

    private final StringPublisher robotModePub;
    private final BooleanPublisher robotEnabledPub;
    private final StringPublisher alliancePub;
    private final DoublePublisher matchTimePub;
    private final DoublePublisher robotTimestampPub;

    private final DoublePublisher poseXPub;
    private final DoublePublisher poseYPub;
    private final DoublePublisher headingPub;
    private final DoublePublisher pigeonYawPub;
    private final DoublePublisher pigeonPitchPub;
    private final DoublePublisher pigeonRollPub;

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

    // System health
    private final DoublePublisher batteryVoltagePub;
    private final BooleanPublisher brownoutAlertPub;
    private final BooleanPublisher isBrownoutPub;

    // Auto selection & execution
    private final StringPublisher selectedAutoNamePub;
    private final StringPublisher selectedAutoSourcePub;
    private final StringArrayPublisher autoOptionsPub;
    private final BooleanPublisher autoCommandRunningPub;

    // Match info
    private final IntegerPublisher matchNumberPub;
    private final StringPublisher eventNamePub;

    // Camera / vision connection
    private final BooleanPublisher cameraConnectedPub;
    private final IntegerPublisher visionTagIdPub;
    private final DoublePublisher visionDistancePub;

    // CAN bus health
    private final DoublePublisher canBusUtilizationPub;
    private final IntegerPublisher canReceiveErrorCountPub;
    private final IntegerPublisher canTransmitErrorCountPub;

    // Swerve module angles
    private final DoublePublisher swerveFLAnglePub;
    private final DoublePublisher swerveFRAnglePub;
    private final DoublePublisher swerveBLAnglePub;
    private final DoublePublisher swerveBRAnglePub;

    // CANCoder health (per-module)
    private final DoublePublisher cancoderFLPosPub;
    private final DoublePublisher cancoderFRPosPub;
    private final DoublePublisher cancoderBLPosPub;
    private final DoublePublisher cancoderBRPosPub;
    private final DoublePublisher cancoderFLAbsRawPub;
    private final DoublePublisher cancoderFRAbsRawPub;
    private final DoublePublisher cancoderBLAbsRawPub;
    private final DoublePublisher cancoderBRAbsRawPub;
    private final BooleanPublisher cancoderFLOkPub;
    private final BooleanPublisher cancoderFROkPub;
    private final BooleanPublisher cancoderBLOkPub;
    private final BooleanPublisher cancoderBROkPub;

    // Motor temperatures
    private final DoublePublisher driveFLTempPub;
    private final DoublePublisher driveFRTempPub;
    private final DoublePublisher driveBLTempPub;
    private final DoublePublisher driveBRTempPub;
    private final DoublePublisher shooterLeftTempPub;
    private final DoublePublisher shooterRightTempPub;

    // Controller diagnostics
    private final StringPublisher driverButtonsActivePub;
    private final StringPublisher operatorButtonsActivePub;
    private final IntegerPublisher controlEventSeqPub;
    private final DoublePublisher controlEventTimestampPub;
    private final StringPublisher controlEventMessagePub;

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
    private final StringSubscriber selectAutoNameCmdSub;
    private final IntegerSubscriber selectAutoCmdSub;

    private long zeroHeadingSeqSeen = 0;
    private long stopDriveSeqSeen = 0;
    private long intakeHomeSeqSeen = 0;
    private long alignShootSeqSeen = 0;
    private long fallbackShootSeqSeen = 0;
    private long level1ClimbSeqSeen = 0;
    private long selectAutoSeqSeen = 0;

    public RobotDashboardService(Actions actions) {
        this(actions, NetworkTableInstance.getDefault());
    }

    RobotDashboardService(Actions actions, NetworkTableInstance ntInstance) {
        this.actions = actions;

        NetworkTable table = ntInstance.getTable("Dashboard");

        contractVersionPub = table.getStringTopic("meta/contract_version").publish();

        robotModePub = table.getStringTopic("robot/mode").publish();
        robotEnabledPub = table.getBooleanTopic("robot/enabled").publish();
        alliancePub = table.getStringTopic("robot/alliance").publish();
        matchTimePub = table.getDoubleTopic("robot/match_time_sec").publish();
        robotTimestampPub = table.getDoubleTopic("robot/timestamp_sec").publish();

        poseXPub = table.getDoubleTopic("drive/pose_x_m").publish();
        poseYPub = table.getDoubleTopic("drive/pose_y_m").publish();
        headingPub = table.getDoubleTopic("drive/heading_deg").publish();
        pigeonYawPub = table.getDoubleTopic("imu/pigeon_yaw_deg").publish();
        pigeonPitchPub = table.getDoubleTopic("imu/pigeon_pitch_deg").publish();
        pigeonRollPub = table.getDoubleTopic("imu/pigeon_roll_deg").publish();

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

        // System health
        batteryVoltagePub = table.getDoubleTopic("health/battery_voltage").publish();
        brownoutAlertPub = table.getBooleanTopic("health/brownout_alert").publish();
        isBrownoutPub = table.getBooleanTopic("health/is_brownout").publish();

        // Auto selection & execution
        selectedAutoNamePub = table.getStringTopic("auto/selected_name").publish();
        selectedAutoSourcePub = table.getStringTopic("auto/selected_source").publish();
        autoOptionsPub = table.getStringArrayTopic("auto/options").publish();
        autoCommandRunningPub = table.getBooleanTopic("auto/command_running").publish();

        // Match info
        matchNumberPub = table.getIntegerTopic("match/number").publish();
        eventNamePub = table.getStringTopic("match/event_name").publish();

        // Camera / vision connection
        cameraConnectedPub = table.getBooleanTopic("vision/camera_connected").publish();
        visionTagIdPub = table.getIntegerTopic("vision/tag_id").publish();
        visionDistancePub = table.getDoubleTopic("vision/distance_m").publish();

        // CAN bus health
        canBusUtilizationPub = table.getDoubleTopic("health/can_bus_utilization").publish();
        canReceiveErrorCountPub = table.getIntegerTopic("health/can_receive_errors").publish();
        canTransmitErrorCountPub = table.getIntegerTopic("health/can_transmit_errors").publish();

        // Swerve module angles
        swerveFLAnglePub = table.getDoubleTopic("swerve/fl_angle_deg").publish();
        swerveFRAnglePub = table.getDoubleTopic("swerve/fr_angle_deg").publish();
        swerveBLAnglePub = table.getDoubleTopic("swerve/bl_angle_deg").publish();
        swerveBRAnglePub = table.getDoubleTopic("swerve/br_angle_deg").publish();

        // CANCoder health
        cancoderFLPosPub = table.getDoubleTopic("cancoder/fl_pos_rot").publish();
        cancoderFRPosPub = table.getDoubleTopic("cancoder/fr_pos_rot").publish();
        cancoderBLPosPub = table.getDoubleTopic("cancoder/bl_pos_rot").publish();
        cancoderBRPosPub = table.getDoubleTopic("cancoder/br_pos_rot").publish();
        cancoderFLAbsRawPub = table.getDoubleTopic("cancoder/fl_abs_raw_rot").publish();
        cancoderFRAbsRawPub = table.getDoubleTopic("cancoder/fr_abs_raw_rot").publish();
        cancoderBLAbsRawPub = table.getDoubleTopic("cancoder/bl_abs_raw_rot").publish();
        cancoderBRAbsRawPub = table.getDoubleTopic("cancoder/br_abs_raw_rot").publish();
        cancoderFLOkPub = table.getBooleanTopic("cancoder/fl_ok").publish();
        cancoderFROkPub = table.getBooleanTopic("cancoder/fr_ok").publish();
        cancoderBLOkPub = table.getBooleanTopic("cancoder/bl_ok").publish();
        cancoderBROkPub = table.getBooleanTopic("cancoder/br_ok").publish();

        // Motor temperatures
        driveFLTempPub = table.getDoubleTopic("temps/drive_fl_c").publish();
        driveFRTempPub = table.getDoubleTopic("temps/drive_fr_c").publish();
        driveBLTempPub = table.getDoubleTopic("temps/drive_bl_c").publish();
        driveBRTempPub = table.getDoubleTopic("temps/drive_br_c").publish();
        shooterLeftTempPub = table.getDoubleTopic("temps/shooter_left_c").publish();
        shooterRightTempPub = table.getDoubleTopic("temps/shooter_right_c").publish();

        // Controller diagnostics
        driverButtonsActivePub = table.getStringTopic("controls/driver_buttons_active").publish();
        operatorButtonsActivePub = table.getStringTopic("controls/operator_buttons_active").publish();
        controlEventSeqPub = table.getIntegerTopic("controls/last_event_seq").publish();
        controlEventTimestampPub = table.getDoubleTopic("controls/last_event_timestamp_sec").publish();
        controlEventMessagePub = table.getStringTopic("controls/last_event_message").publish();

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
        selectAutoNameCmdSub = table.getStringTopic("cmd/select_auto_name").subscribe("");
        selectAutoCmdSub = table.getIntegerTopic("cmd/select_auto_seq").subscribe(0);

        // Ignore any stale sequence values already present when robot code starts.
        zeroHeadingSeqSeen = zeroHeadingCmdSub.get();
        stopDriveSeqSeen = stopDriveCmdSub.get();
        intakeHomeSeqSeen = intakeHomeCmdSub.get();
        alignShootSeqSeen = alignShootCmdSub.get();
        fallbackShootSeqSeen = fallbackShootCmdSub.get();
        level1ClimbSeqSeen = level1ClimbCmdSub.get();
        selectAutoSeqSeen = selectAutoCmdSub.get();
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
        robotTimestampPub.set(snapshot.timestampSec());

        poseXPub.set(snapshot.poseX_m());
        poseYPub.set(snapshot.poseY_m());
        headingPub.set(snapshot.headingDeg());
        pigeonYawPub.set(snapshot.pigeonYawDeg());
        pigeonPitchPub.set(snapshot.pigeonPitchDeg());
        pigeonRollPub.set(snapshot.pigeonRollDeg());

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

        // System health
        batteryVoltagePub.set(snapshot.batteryVoltage());
        brownoutAlertPub.set(snapshot.brownoutAlert());
        isBrownoutPub.set(snapshot.isBrownout());

        // Auto selection & execution
        selectedAutoNamePub.set(snapshot.selectedAutoName());
        selectedAutoSourcePub.set(snapshot.selectedAutoSource());
        autoOptionsPub.set(snapshot.autoOptions());
        autoCommandRunningPub.set(snapshot.autoCommandRunning());

        // Match info
        matchNumberPub.set(snapshot.matchNumber());
        eventNamePub.set(snapshot.eventName());

        // Camera / vision connection
        cameraConnectedPub.set(snapshot.cameraConnected());
        visionTagIdPub.set(snapshot.visionTagId());
        visionDistancePub.set(snapshot.visionDistanceM());

        // CAN bus health
        canBusUtilizationPub.set(snapshot.canBusUtilization());
        canReceiveErrorCountPub.set(snapshot.canReceiveErrorCount());
        canTransmitErrorCountPub.set(snapshot.canTransmitErrorCount());

        // Swerve module angles
        swerveFLAnglePub.set(snapshot.swerveFLAngleDeg());
        swerveFRAnglePub.set(snapshot.swerveFRAngleDeg());
        swerveBLAnglePub.set(snapshot.swerveBLAngleDeg());
        swerveBRAnglePub.set(snapshot.swerveBRAngleDeg());

        // CANCoder health
        cancoderFLPosPub.set(snapshot.cancoderFLPosRot());
        cancoderFRPosPub.set(snapshot.cancoderFRPosRot());
        cancoderBLPosPub.set(snapshot.cancoderBLPosRot());
        cancoderBRPosPub.set(snapshot.cancoderBRPosRot());
        cancoderFLAbsRawPub.set(snapshot.cancoderFLAbsRawRot());
        cancoderFRAbsRawPub.set(snapshot.cancoderFRAbsRawRot());
        cancoderBLAbsRawPub.set(snapshot.cancoderBLAbsRawRot());
        cancoderBRAbsRawPub.set(snapshot.cancoderBRAbsRawRot());
        cancoderFLOkPub.set(snapshot.cancoderFLOk());
        cancoderFROkPub.set(snapshot.cancoderFROk());
        cancoderBLOkPub.set(snapshot.cancoderBLOk());
        cancoderBROkPub.set(snapshot.cancoderBROk());

        // Motor temperatures
        driveFLTempPub.set(snapshot.driveFLTempC());
        driveFRTempPub.set(snapshot.driveFRTempC());
        driveBLTempPub.set(snapshot.driveBLTempC());
        driveBRTempPub.set(snapshot.driveBRTempC());
        shooterLeftTempPub.set(snapshot.shooterLeftTempC());
        shooterRightTempPub.set(snapshot.shooterRightTempC());

        // Controller diagnostics
        driverButtonsActivePub.set(snapshot.driverButtonsActive());
        operatorButtonsActivePub.set(snapshot.operatorButtonsActive());
        controlEventSeqPub.set(snapshot.controlEventSeq());
        controlEventTimestampPub.set(snapshot.controlEventTimestampSec());
        controlEventMessagePub.set(snapshot.controlEventMessage());
    }

    private void processCommandRequests(DashboardSnapshot snapshot) {
        boolean disabled = "DISABLED".equals(snapshot.robotMode());
        boolean teleopEnabled = snapshot.enabled() && "TELEOP".equals(snapshot.robotMode());
        boolean intakeHomeEnabled = snapshot.enabled()
                && ("TELEOP".equals(snapshot.robotMode()) || "TEST".equals(snapshot.robotMode()));

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
                intakeHomeEnabled,
                "Requires enabled teleop/test (disabled mode blocks motor output)",
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

        selectAutoSeqSeen = runAutoSelectionIfNew(
                snapshot,
                selectAutoCmdSub,
                selectAutoNameCmdSub,
                selectAutoSeqSeen,
                snapshot.timestampSec(),
                disabled);
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

    private long runAutoSelectionIfNew(
            DashboardSnapshot snapshot,
            IntegerSubscriber sequenceSubscriber,
            StringSubscriber nameSubscriber,
            long lastSeen,
            double timestampSec,
            boolean disabled) {
        long seq = sequenceSubscriber.get();
        if (seq <= lastSeen) {
            return lastSeen;
        }

        String requestedAutoName = nameSubscriber.get();
        String normalizedAutoName = requestedAutoName == null ? "" : requestedAutoName.trim();
        if (!disabled) {
            publishAck("select_auto", "REJECTED", seq, "Only allowed while robot is disabled", timestampSec);
            return seq;
        }
        if (normalizedAutoName.isEmpty()) {
            publishAck("select_auto", "REJECTED", seq, "No auto name provided", timestampSec);
            return seq;
        }
        if (!containsAutoOption(snapshot.autoOptions(), normalizedAutoName)) {
            publishAck("select_auto", "REJECTED", seq, "Unknown auto: " + normalizedAutoName, timestampSec);
            return seq;
        }

        actions.selectAutoByName(normalizedAutoName);
        publishAck("select_auto", "OK", seq, "Selected " + normalizedAutoName, timestampSec);
        return seq;
    }

    private static boolean containsAutoOption(String[] autoOptions, String requestedAutoName) {
        if (autoOptions == null) {
            return false;
        }

        for (String autoOption : autoOptions) {
            if (requestedAutoName.equals(autoOption)) {
                return true;
            }
        }
        return false;
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
