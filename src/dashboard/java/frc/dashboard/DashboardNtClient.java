package frc.dashboard;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;

public class DashboardNtClient implements AutoCloseable {

    public enum DashboardCommand {
        ZERO_HEADING,
        STOP_DRIVE,
        INTAKE_HOME,
        ALIGN_SHOOT,
        FALLBACK_SHOOT,
        LEVEL1_CLIMB
    }

    private final NetworkTableInstance nt = NetworkTableInstance.create();
    private final NetworkTable table = nt.getTable("Dashboard");

    private final StringSubscriber modeSub = table.getStringTopic("robot/mode").subscribe("UNKNOWN");
    private final BooleanSubscriber enabledSub = table.getBooleanTopic("robot/enabled").subscribe(false);
    private final StringSubscriber allianceSub = table.getStringTopic("robot/alliance").subscribe("UNKNOWN");
    private final DoubleSubscriber matchTimeSub = table.getDoubleTopic("robot/match_time_sec").subscribe(-1.0);
    private final DoubleSubscriber robotTimestampSub = table.getDoubleTopic("robot/timestamp_sec").subscribe(0.0);

    private final DoubleSubscriber poseXSub = table.getDoubleTopic("drive/pose_x_m").subscribe(0.0);
    private final DoubleSubscriber poseYSub = table.getDoubleTopic("drive/pose_y_m").subscribe(0.0);
    private final DoubleSubscriber headingSub = table.getDoubleTopic("drive/heading_deg").subscribe(0.0);
    private final DoubleSubscriber pigeonYawSub = table.getDoubleTopic("imu/pigeon_yaw_deg").subscribe(Double.NaN);
    private final DoubleSubscriber pigeonPitchSub = table.getDoubleTopic("imu/pigeon_pitch_deg").subscribe(Double.NaN);
    private final DoubleSubscriber pigeonRollSub = table.getDoubleTopic("imu/pigeon_roll_deg").subscribe(Double.NaN);

    private final DoubleSubscriber shooterLeftSub = table.getDoubleTopic("shooter/left_rps").subscribe(0.0);
    private final DoubleSubscriber shooterRightSub = table.getDoubleTopic("shooter/right_rps").subscribe(0.0);
    private final BooleanSubscriber shooterAtSpeedSub = table.getBooleanTopic("shooter/at_speed").subscribe(false);

    private final BooleanSubscriber intakeHomedSub = table.getBooleanTopic("intake/homed").subscribe(false);
    private final BooleanSubscriber intakeLimitSub = table.getBooleanTopic("intake/limit_switch_pressed").subscribe(false);
    private final DoubleSubscriber intakeTiltSub = table.getDoubleTopic("intake/tilt_deg").subscribe(0.0);
    private final DoubleSubscriber intakeRollerCurrentSub =
            table.getDoubleTopic("intake/roller_current_amps").subscribe(0.0);

    private final DoubleSubscriber feederCurrentSub = table.getDoubleTopic("feeder/current_amps").subscribe(0.0);
    private final DoubleSubscriber hopperCurrentSub = table.getDoubleTopic("hopper/current_amps").subscribe(0.0);

    private final BooleanSubscriber climberArmedSub = table.getBooleanTopic("climber/armed").subscribe(false);
    private final DoubleSubscriber climberPositionSub = table.getDoubleTopic("climber/position_rot").subscribe(0.0);
    private final DoubleSubscriber climberCurrentSub = table.getDoubleTopic("climber/current_amps").subscribe(0.0);

    private final StringSubscriber alignStateSub = table.getStringTopic("align/state").subscribe("IDLE");
    private final BooleanSubscriber alignCommandActiveSub =
            table.getBooleanTopic("align/command_active").subscribe(false);
    private final BooleanSubscriber alignHasTargetSub = table.getBooleanTopic("align/has_target").subscribe(false);
    private final BooleanSubscriber alignGeometrySub =
            table.getBooleanTopic("align/geometry_feasible").subscribe(false);
    private final BooleanSubscriber alignShootableSub =
            table.getBooleanTopic("align/has_shootable_target").subscribe(false);
    private final DoubleSubscriber alignYawSub = table.getDoubleTopic("align/yaw_deg").subscribe(Double.NaN);
    private final DoubleSubscriber alignPitchSub = table.getDoubleTopic("align/pitch_deg").subscribe(Double.NaN);
    private final StringSubscriber alignAbortSub = table.getStringTopic("align/last_abort_reason").subscribe("");

    private final BooleanSubscriber readySub = table.getBooleanTopic("shot/ready").subscribe(false);
    private final StringSubscriber readyReasonSub = table.getStringTopic("shot/ready_reason").subscribe("");

    // System health
    private final DoubleSubscriber batteryVoltageSub = table.getDoubleTopic("health/battery_voltage").subscribe(0.0);
    private final BooleanSubscriber brownoutAlertSub = table.getBooleanTopic("health/brownout_alert").subscribe(false);
    private final BooleanSubscriber isBrownoutSub = table.getBooleanTopic("health/is_brownout").subscribe(false);

    // Auto selection & execution
    private final StringSubscriber selectedAutoNameSub = table.getStringTopic("auto/selected_name").subscribe("");
    private final BooleanSubscriber autoCommandRunningSub =
            table.getBooleanTopic("auto/command_running").subscribe(false);

    // Match info
    private final IntegerSubscriber matchNumberSub = table.getIntegerTopic("match/number").subscribe(0);
    private final StringSubscriber eventNameSub = table.getStringTopic("match/event_name").subscribe("");

    // Camera / vision connection
    private final BooleanSubscriber cameraConnectedSub =
            table.getBooleanTopic("vision/camera_connected").subscribe(false);

    // CAN bus health
    private final DoubleSubscriber canBusUtilizationSub =
            table.getDoubleTopic("health/can_bus_utilization").subscribe(0.0);
    private final IntegerSubscriber canReceiveErrorCountSub =
            table.getIntegerTopic("health/can_receive_errors").subscribe(0);
    private final IntegerSubscriber canTransmitErrorCountSub =
            table.getIntegerTopic("health/can_transmit_errors").subscribe(0);

    // Swerve module angles
    private final DoubleSubscriber swerveFLAngleSub = table.getDoubleTopic("swerve/fl_angle_deg").subscribe(0.0);
    private final DoubleSubscriber swerveFRAngleSub = table.getDoubleTopic("swerve/fr_angle_deg").subscribe(0.0);
    private final DoubleSubscriber swerveBLAngleSub = table.getDoubleTopic("swerve/bl_angle_deg").subscribe(0.0);
    private final DoubleSubscriber swerveBRAngleSub = table.getDoubleTopic("swerve/br_angle_deg").subscribe(0.0);
    // CANcoder calibration telemetry
    private final DoubleSubscriber cancoderFLRawSub = table.getDoubleTopic("cancoder/fl_raw_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderFRRawSub = table.getDoubleTopic("cancoder/fr_raw_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBLRawSub = table.getDoubleTopic("cancoder/bl_raw_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBRRawSub = table.getDoubleTopic("cancoder/br_raw_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderFLOffsetSub = table.getDoubleTopic("cancoder/fl_offset_rot")
            .subscribe(Double.NaN);
    private final DoubleSubscriber cancoderFROffsetSub = table.getDoubleTopic("cancoder/fr_offset_rot")
            .subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBLOffsetSub = table.getDoubleTopic("cancoder/bl_offset_rot")
            .subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBROffsetSub = table.getDoubleTopic("cancoder/br_offset_rot")
            .subscribe(Double.NaN);

    // Motor temperatures
    private final DoubleSubscriber driveFLTempSub = table.getDoubleTopic("temps/drive_fl_c").subscribe(0.0);
    private final DoubleSubscriber driveFRTempSub = table.getDoubleTopic("temps/drive_fr_c").subscribe(0.0);
    private final DoubleSubscriber driveBLTempSub = table.getDoubleTopic("temps/drive_bl_c").subscribe(0.0);
    private final DoubleSubscriber driveBRTempSub = table.getDoubleTopic("temps/drive_br_c").subscribe(0.0);
    private final DoubleSubscriber shooterLeftTempSub = table.getDoubleTopic("temps/shooter_left_c").subscribe(0.0);
    private final DoubleSubscriber shooterRightTempSub = table.getDoubleTopic("temps/shooter_right_c").subscribe(0.0);

    // Ack
    private final StringSubscriber ackCommandSub = table.getStringTopic("ack/last_command").subscribe("");
    private final StringSubscriber ackStatusSub = table.getStringTopic("ack/last_status").subscribe("");
    private final IntegerSubscriber ackSeqSub = table.getIntegerTopic("ack/last_seq").subscribe(0);
    private final StringSubscriber ackMessageSub = table.getStringTopic("ack/message").subscribe("");
    private final DoubleSubscriber ackTimestampSub = table.getDoubleTopic("ack/timestamp_sec").subscribe(0.0);

    private final IntegerPublisher zeroHeadingPub = table.getIntegerTopic("cmd/zero_heading_seq").publish();
    private final IntegerPublisher stopDrivePub = table.getIntegerTopic("cmd/stop_drive_seq").publish();
    private final IntegerPublisher intakeHomePub = table.getIntegerTopic("cmd/intake_home_seq").publish();
    private final IntegerPublisher alignShootPub = table.getIntegerTopic("cmd/align_shoot_seq").publish();
    private final IntegerPublisher fallbackShootPub = table.getIntegerTopic("cmd/fallback_shoot_seq").publish();
    private final IntegerPublisher level1ClimbPub = table.getIntegerTopic("cmd/level1_climb_seq").publish();
    private final IntegerSubscriber zeroHeadingSeqSub = table.getIntegerTopic("cmd/zero_heading_seq").subscribe(0);
    private final IntegerSubscriber stopDriveSeqSub = table.getIntegerTopic("cmd/stop_drive_seq").subscribe(0);
    private final IntegerSubscriber intakeHomeSeqSub = table.getIntegerTopic("cmd/intake_home_seq").subscribe(0);
    private final IntegerSubscriber alignShootSeqSub = table.getIntegerTopic("cmd/align_shoot_seq").subscribe(0);
    private final IntegerSubscriber fallbackShootSeqSub = table.getIntegerTopic("cmd/fallback_shoot_seq").subscribe(0);
    private final IntegerSubscriber level1ClimbSeqSub = table.getIntegerTopic("cmd/level1_climb_seq").subscribe(0);

    private long zeroHeadingSeq = 0;
    private long stopDriveSeq = 0;
    private long intakeHomeSeq = 0;
    private long alignShootSeq = 0;
    private long fallbackShootSeq = 0;
    private long level1ClimbSeq = 0;

    public DashboardNtClient(String clientName, int teamNumber, String hostOverride) {
        nt.startClient4(clientName);
        if (teamNumber > 0) {
            nt.setServerTeam(teamNumber);
        } else {
            nt.setServer(hostOverride);
        }
    }

    public DashboardData read() {
        return new DashboardData(
                nt.isConnected(),
                modeSub.get(),
                enabledSub.get(),
                allianceSub.get(),
                matchTimeSub.get(),
                robotTimestampSub.get(),
                poseXSub.get(),
                poseYSub.get(),
                headingSub.get(),
                pigeonYawSub.get(),
                pigeonPitchSub.get(),
                pigeonRollSub.get(),
                shooterLeftSub.get(),
                shooterRightSub.get(),
                shooterAtSpeedSub.get(),
                intakeHomedSub.get(),
                intakeLimitSub.get(),
                intakeTiltSub.get(),
                intakeRollerCurrentSub.get(),
                feederCurrentSub.get(),
                hopperCurrentSub.get(),
                climberArmedSub.get(),
                climberPositionSub.get(),
                climberCurrentSub.get(),
                alignStateSub.get(),
                alignCommandActiveSub.get(),
                alignHasTargetSub.get(),
                alignGeometrySub.get(),
                alignShootableSub.get(),
                alignYawSub.get(),
                alignPitchSub.get(),
                alignAbortSub.get(),
                readySub.get(),
                readyReasonSub.get(),
                // System health
                batteryVoltageSub.get(),
                brownoutAlertSub.get(),
                isBrownoutSub.get(),
                // Auto
                selectedAutoNameSub.get(),
                autoCommandRunningSub.get(),
                // Match info
                matchNumberSub.get(),
                eventNameSub.get(),
                // Camera
                cameraConnectedSub.get(),
                // CAN health
                canBusUtilizationSub.get(),
                canReceiveErrorCountSub.get(),
                canTransmitErrorCountSub.get(),
                // Swerve angles
                swerveFLAngleSub.get(),
                swerveFRAngleSub.get(),
                swerveBLAngleSub.get(),
                swerveBRAngleSub.get(),
                // CANcoder calibration telemetry
                cancoderFLRawSub.get(),
                cancoderFRRawSub.get(),
                cancoderBLRawSub.get(),
                cancoderBRRawSub.get(),
                cancoderFLOffsetSub.get(),
                cancoderFROffsetSub.get(),
                cancoderBLOffsetSub.get(),
                cancoderBROffsetSub.get(),
                // Motor temps
                driveFLTempSub.get(),
                driveFRTempSub.get(),
                driveBLTempSub.get(),
                driveBRTempSub.get(),
                shooterLeftTempSub.get(),
                shooterRightTempSub.get(),
                // Ack
                ackCommandSub.get(),
                ackStatusSub.get(),
                ackSeqSub.get(),
                ackMessageSub.get(),
                ackTimestampSub.get());
    }

    public synchronized void sendCommand(DashboardCommand command) {
        switch (command) {
            case ZERO_HEADING -> {
                zeroHeadingSeq = nextCommandSequence(zeroHeadingSeq, zeroHeadingSeqSub.get());
                zeroHeadingPub.set(zeroHeadingSeq);
            }
            case STOP_DRIVE -> {
                stopDriveSeq = nextCommandSequence(stopDriveSeq, stopDriveSeqSub.get());
                stopDrivePub.set(stopDriveSeq);
            }
            case INTAKE_HOME -> {
                intakeHomeSeq = nextCommandSequence(intakeHomeSeq, intakeHomeSeqSub.get());
                intakeHomePub.set(intakeHomeSeq);
            }
            case ALIGN_SHOOT -> {
                alignShootSeq = nextCommandSequence(alignShootSeq, alignShootSeqSub.get());
                alignShootPub.set(alignShootSeq);
            }
            case FALLBACK_SHOOT -> {
                fallbackShootSeq = nextCommandSequence(fallbackShootSeq, fallbackShootSeqSub.get());
                fallbackShootPub.set(fallbackShootSeq);
            }
            case LEVEL1_CLIMB -> {
                level1ClimbSeq = nextCommandSequence(level1ClimbSeq, level1ClimbSeqSub.get());
                level1ClimbPub.set(level1ClimbSeq);
            }
            default -> throw new IllegalStateException("Unhandled command " + command);
        }
    }

    static long nextCommandSequence(long localSeq, long topicSeq) {
        return Math.max(localSeq, topicSeq) + 1;
    }

    @Override
    public void close() {
        nt.close();
    }
}
