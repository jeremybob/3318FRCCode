package frc.dashboard;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringSubscriber;

public class DashboardNtClient implements AutoCloseable {

    public enum DashboardCommand {
        ZERO_HEADING,
        STOP_DRIVE,
        INTAKE_HOME,
        ALIGN_ONLY,
        ALIGN_SHOOT,
        FALLBACK_SHOOT,
        LEVEL1_CLIMB,
        CALIBRATE_CANCODERS,
        STOP_SWERVE_VALIDATION
    }

    private final NetworkTableInstance nt = NetworkTableInstance.create();
    private final NetworkTable table = nt.getTable("Dashboard");
    private final NetworkTable smartDashboardTable = nt.getTable("SmartDashboard");
    private final String robotHost;

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
    // Fallback telemetry path while contract topics are being brought up.
    private final DoubleSubscriber pigeonYawSmartSub =
            smartDashboardTable.getDoubleTopic("Swerve/PigeonYawDeg").subscribe(Double.NaN);
    private final DoubleSubscriber pigeonPitchSmartSub =
            smartDashboardTable.getDoubleTopic("Swerve/PigeonPitchDeg").subscribe(Double.NaN);
    private final DoubleSubscriber pigeonRollSmartSub =
            smartDashboardTable.getDoubleTopic("Swerve/PigeonRollDeg").subscribe(Double.NaN);

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

    // 2026 REBUILT: HUB shift activity
    private final BooleanSubscriber hubActiveSub = table.getBooleanTopic("hub/active").subscribe(true);
    private final DoubleSubscriber hubSecondsToNextShiftSub =
            table.getDoubleTopic("hub/seconds_to_next_shift").subscribe(0.0);

    // System health
    private final DoubleSubscriber batteryVoltageSub = table.getDoubleTopic("health/battery_voltage").subscribe(0.0);
    private final BooleanSubscriber brownoutAlertSub = table.getBooleanTopic("health/brownout_alert").subscribe(false);
    private final BooleanSubscriber isBrownoutSub = table.getBooleanTopic("health/is_brownout").subscribe(false);

    // Auto selection & execution
    private final StringSubscriber selectedAutoNameSub = table.getStringTopic("auto/selected_name").subscribe("");
    private final StringSubscriber selectedAutoSourceSub = table.getStringTopic("auto/selected_source").subscribe("");
    private final StringArraySubscriber autoOptionsSub = table.getStringArrayTopic("auto/options").subscribe(new String[0]);
    private final BooleanSubscriber autoCommandRunningSub =
            table.getBooleanTopic("auto/command_running").subscribe(false);

    // Expected auto starting pose
    private final DoubleSubscriber autoStartXSub = table.getDoubleTopic("auto/start_x_m").subscribe(Double.NaN);
    private final DoubleSubscriber autoStartYSub = table.getDoubleTopic("auto/start_y_m").subscribe(Double.NaN);
    private final DoubleSubscriber autoStartHeadingSub =
            table.getDoubleTopic("auto/start_heading_deg").subscribe(Double.NaN);

    // Match info
    private final IntegerSubscriber matchNumberSub = table.getIntegerTopic("match/number").subscribe(0);
    private final StringSubscriber eventNameSub = table.getStringTopic("match/event_name").subscribe("");

    // Camera / vision connection
    private final BooleanSubscriber cameraConnectedSub =
            table.getBooleanTopic("vision/camera_connected").subscribe(false);
    private final StringSubscriber cameraStatusSub =
            table.getStringTopic("vision/camera_status").subscribe("STARTING");
    private final IntegerSubscriber cameraActiveDeviceSub =
            table.getIntegerTopic("vision/camera_active_device").subscribe(-1);
    private final StringSubscriber cameraActiveNameSub =
            table.getStringTopic("vision/camera_name").subscribe("");
    private final StringSubscriber cameraActivePathSub =
            table.getStringTopic("vision/camera_path").subscribe("");
    private final StringSubscriber cameraEnumeratedSub =
            table.getStringTopic("vision/camera_enumerated").subscribe("");
    private final StringSubscriber cameraLastErrorSub =
            table.getStringTopic("vision/camera_last_error").subscribe("");
    private final IntegerSubscriber cameraFrameCountSub =
            table.getIntegerTopic("vision/camera_frame_count").subscribe(0);
    private final DoubleSubscriber cameraLastFrameTimestampSub =
            table.getDoubleTopic("vision/camera_last_frame_ts_sec").subscribe(Double.NaN);
    private final IntegerSubscriber visionTagIdSub =
            table.getIntegerTopic("vision/tag_id").subscribe(-1);
    private final BooleanSubscriber visionHasTargetSub =
            table.getBooleanTopic("vision/has_target").subscribe(false);
    private final DoubleSubscriber visionYawSub =
            table.getDoubleTopic("vision/yaw_deg").subscribe(Double.NaN);
    private final DoubleSubscriber visionPitchSub =
            table.getDoubleTopic("vision/pitch_deg").subscribe(Double.NaN);
    private final DoubleSubscriber visionDistanceSub =
            table.getDoubleTopic("vision/distance_m").subscribe(Double.NaN);
    private final DoubleSubscriber visionTagPixelHeightSub =
            table.getDoubleTopic("vision/tag_pixel_height_px").subscribe(Double.NaN);
    private final DoubleSubscriber visionTargetTimestampSub =
            table.getDoubleTopic("vision/target_timestamp_sec").subscribe(Double.NaN);

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
    // CANCoder health (live position + status)
    private final DoubleSubscriber cancoderFLPosSub = table.getDoubleTopic("cancoder/fl_pos_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderFRPosSub = table.getDoubleTopic("cancoder/fr_pos_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBLPosSub = table.getDoubleTopic("cancoder/bl_pos_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBRPosSub = table.getDoubleTopic("cancoder/br_pos_rot").subscribe(Double.NaN);
    private final DoubleSubscriber cancoderFLAbsRawSub = table.getDoubleTopic("cancoder/fl_abs_raw_rot")
            .subscribe(Double.NaN);
    private final DoubleSubscriber cancoderFRAbsRawSub = table.getDoubleTopic("cancoder/fr_abs_raw_rot")
            .subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBLAbsRawSub = table.getDoubleTopic("cancoder/bl_abs_raw_rot")
            .subscribe(Double.NaN);
    private final DoubleSubscriber cancoderBRAbsRawSub = table.getDoubleTopic("cancoder/br_abs_raw_rot")
            .subscribe(Double.NaN);
    private final BooleanSubscriber cancoderFLOkSub = table.getBooleanTopic("cancoder/fl_ok").subscribe(false);
    private final BooleanSubscriber cancoderFROkSub = table.getBooleanTopic("cancoder/fr_ok").subscribe(false);
    private final BooleanSubscriber cancoderBLOkSub = table.getBooleanTopic("cancoder/bl_ok").subscribe(false);
    private final BooleanSubscriber cancoderBROkSub = table.getBooleanTopic("cancoder/br_ok").subscribe(false);

    // Motor temperatures
    private final DoubleSubscriber driveFLTempSub = table.getDoubleTopic("temps/drive_fl_c").subscribe(0.0);
    private final DoubleSubscriber driveFRTempSub = table.getDoubleTopic("temps/drive_fr_c").subscribe(0.0);
    private final DoubleSubscriber driveBLTempSub = table.getDoubleTopic("temps/drive_bl_c").subscribe(0.0);
    private final DoubleSubscriber driveBRTempSub = table.getDoubleTopic("temps/drive_br_c").subscribe(0.0);
    private final DoubleSubscriber shooterLeftTempSub = table.getDoubleTopic("temps/shooter_left_c").subscribe(0.0);
    private final DoubleSubscriber shooterRightTempSub = table.getDoubleTopic("temps/shooter_right_c").subscribe(0.0);

    // Controller diagnostics
    private final StringSubscriber driverButtonsSub =
            table.getStringTopic("controls/driver_buttons_active").subscribe("--");
    private final StringSubscriber operatorButtonsSub =
            table.getStringTopic("controls/operator_buttons_active").subscribe("--");
    private final IntegerSubscriber controlEventSeqSub = table.getIntegerTopic("controls/last_event_seq").subscribe(0);
    private final DoubleSubscriber controlEventTimestampSub =
            table.getDoubleTopic("controls/last_event_timestamp_sec").subscribe(0.0);
    private final StringSubscriber controlEventMessageSub =
            table.getStringTopic("controls/last_event_message").subscribe("");

    private final BooleanSubscriber swerveValidationActiveSub =
            table.getBooleanTopic("swerve/validation_active").subscribe(false);
    private final StringSubscriber swerveValidationModuleTokenSub =
            table.getStringTopic("swerve/validation_module_token").subscribe("NONE");
    private final StringSubscriber swerveValidationModuleDisplaySub =
            table.getStringTopic("swerve/validation_module_display").subscribe("--");
    private final StringSubscriber swerveValidationModeTokenSub =
            table.getStringTopic("swerve/validation_mode_token").subscribe("IDLE");
    private final StringSubscriber swerveValidationModeDisplaySub =
            table.getStringTopic("swerve/validation_mode_display").subscribe("Idle");
    private final DoubleSubscriber swerveValidationDrivePercentSub =
            table.getDoubleTopic("swerve/validation_drive_percent").subscribe(0.0);
    private final DoubleSubscriber swerveValidationSteerPercentSub =
            table.getDoubleTopic("swerve/validation_steer_percent").subscribe(0.0);
    private final DoubleSubscriber swerveValidationStartAngleSub =
            table.getDoubleTopic("swerve/validation_start_angle_deg").subscribe(Double.NaN);
    private final DoubleSubscriber swerveValidationAngleDeltaSub =
            table.getDoubleTopic("swerve/validation_angle_delta_deg").subscribe(Double.NaN);
    private final DoubleSubscriber swerveValidationStartCANcoderSub =
            table.getDoubleTopic("swerve/validation_start_cancoder_rot").subscribe(Double.NaN);
    private final DoubleSubscriber swerveValidationCANcoderDeltaSub =
            table.getDoubleTopic("swerve/validation_cancoder_delta_rot").subscribe(Double.NaN);

    // Ack
    private final StringSubscriber ackCommandSub = table.getStringTopic("ack/last_command").subscribe("");
    private final StringSubscriber ackStatusSub = table.getStringTopic("ack/last_status").subscribe("");
    private final IntegerSubscriber ackSeqSub = table.getIntegerTopic("ack/last_seq").subscribe(0);
    private final StringSubscriber ackMessageSub = table.getStringTopic("ack/message").subscribe("");
    private final DoubleSubscriber ackTimestampSub = table.getDoubleTopic("ack/timestamp_sec").subscribe(0.0);

    private final IntegerPublisher zeroHeadingPub = table.getIntegerTopic("cmd/zero_heading_seq").publish();
    private final IntegerPublisher stopDrivePub = table.getIntegerTopic("cmd/stop_drive_seq").publish();
    private final IntegerPublisher intakeHomePub = table.getIntegerTopic("cmd/intake_home_seq").publish();
    private final IntegerPublisher alignOnlyPub = table.getIntegerTopic("cmd/align_only_seq").publish();
    private final IntegerPublisher alignShootPub = table.getIntegerTopic("cmd/align_shoot_seq").publish();
    private final IntegerPublisher fallbackShootPub = table.getIntegerTopic("cmd/fallback_shoot_seq").publish();
    private final IntegerPublisher level1ClimbPub = table.getIntegerTopic("cmd/level1_climb_seq").publish();
    private final IntegerPublisher calibrateCANcodersPub = table.getIntegerTopic("cmd/calibrate_cancoders_seq").publish();
    private final StringPublisher swerveValidationModulePub = table.getStringTopic("cmd/swerve_validation_module").publish();
    private final StringPublisher swerveValidationModePub = table.getStringTopic("cmd/swerve_validation_mode").publish();
    private final IntegerPublisher swerveValidationPub = table.getIntegerTopic("cmd/swerve_validation_seq").publish();
    private final IntegerPublisher stopSwerveValidationPub = table.getIntegerTopic("cmd/stop_swerve_validation_seq").publish();
    private final StringPublisher selectAutoNamePub = table.getStringTopic("cmd/select_auto_name").publish();
    private final IntegerPublisher selectAutoPub = table.getIntegerTopic("cmd/select_auto_seq").publish();

    private long zeroHeadingSeq = 0;
    private long stopDriveSeq = 0;
    private long intakeHomeSeq = 0;
    private long alignOnlySeq = 0;
    private long alignShootSeq = 0;
    private long fallbackShootSeq = 0;
    private long level1ClimbSeq = 0;
    private long calibrateCANcodersSeq = 0;
    private long swerveValidationSeq = 0;
    private long stopSwerveValidationSeq = 0;
    private long selectAutoSeq = 0;

    public DashboardNtClient(String clientName, int teamNumber, String hostOverride) {
        robotHost = resolveRobotHost(teamNumber, hostOverride);
        nt.startClient4(clientName);
        if (teamNumber > 0) {
            nt.setServerTeam(teamNumber);
        } else {
            nt.setServer(hostOverride);
        }
    }

    public String getRobotHost() {
        return robotHost;
    }

    public DashboardData read() {
        double pigeonYawDeg = preferFinite(pigeonYawSub.get(), pigeonYawSmartSub.get());
        double pigeonPitchDeg = preferFinite(pigeonPitchSub.get(), pigeonPitchSmartSub.get());
        double pigeonRollDeg = preferFinite(pigeonRollSub.get(), pigeonRollSmartSub.get());

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
                pigeonYawDeg,
                pigeonPitchDeg,
                pigeonRollDeg,
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
                // 2026 REBUILT: HUB shift activity
                hubActiveSub.get(),
                hubSecondsToNextShiftSub.get(),
                // System health
                batteryVoltageSub.get(),
                brownoutAlertSub.get(),
                isBrownoutSub.get(),
                // Auto
                selectedAutoNameSub.get(),
                selectedAutoSourceSub.get(),
                autoOptionsSub.get(),
                autoCommandRunningSub.get(),
                // Expected auto starting pose
                autoStartXSub.get(),
                autoStartYSub.get(),
                autoStartHeadingSub.get(),
                // Match info
                matchNumberSub.get(),
                eventNameSub.get(),
                // Camera
                cameraConnectedSub.get(),
                cameraStatusSub.get(),
                (int) cameraActiveDeviceSub.get(),
                cameraActiveNameSub.get(),
                cameraActivePathSub.get(),
                cameraEnumeratedSub.get(),
                cameraLastErrorSub.get(),
                cameraFrameCountSub.get(),
                cameraLastFrameTimestampSub.get(),
                (int) visionTagIdSub.get(),
                visionHasTargetSub.get(),
                visionYawSub.get(),
                visionPitchSub.get(),
                visionDistanceSub.get(),
                visionTagPixelHeightSub.get(),
                visionTargetTimestampSub.get(),
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
                // CANCoder health
                cancoderFLPosSub.get(),
                cancoderFRPosSub.get(),
                cancoderBLPosSub.get(),
                cancoderBRPosSub.get(),
                cancoderFLAbsRawSub.get(),
                cancoderFRAbsRawSub.get(),
                cancoderBLAbsRawSub.get(),
                cancoderBRAbsRawSub.get(),
                cancoderFLOkSub.get(),
                cancoderFROkSub.get(),
                cancoderBLOkSub.get(),
                cancoderBROkSub.get(),
                // Motor temps
                driveFLTempSub.get(),
                driveFRTempSub.get(),
                driveBLTempSub.get(),
                driveBRTempSub.get(),
                shooterLeftTempSub.get(),
                shooterRightTempSub.get(),
                // Controller diagnostics
                driverButtonsSub.get(),
                operatorButtonsSub.get(),
                controlEventSeqSub.get(),
                controlEventTimestampSub.get(),
                controlEventMessageSub.get(),
                swerveValidationActiveSub.get(),
                swerveValidationModuleTokenSub.get(),
                swerveValidationModuleDisplaySub.get(),
                swerveValidationModeTokenSub.get(),
                swerveValidationModeDisplaySub.get(),
                swerveValidationDrivePercentSub.get(),
                swerveValidationSteerPercentSub.get(),
                swerveValidationStartAngleSub.get(),
                swerveValidationAngleDeltaSub.get(),
                swerveValidationStartCANcoderSub.get(),
                swerveValidationCANcoderDeltaSub.get(),
                // Ack
                ackCommandSub.get(),
                ackStatusSub.get(),
                ackSeqSub.get(),
                ackMessageSub.get(),
                ackTimestampSub.get());
    }

    private static String resolveRobotHost(int teamNumber, String hostOverride) {
        if (hostOverride != null && !hostOverride.isBlank()) {
            return hostOverride;
        }
        if (teamNumber <= 0) {
            return "10.33.18.2";
        }
        int first = teamNumber / 100;
        int second = teamNumber % 100;
        return "10." + first + "." + second + ".2";
    }

    private static double preferFinite(double primary, double fallback) {
        if (Double.isFinite(primary)) {
            return primary;
        }
        return Double.isFinite(fallback) ? fallback : Double.NaN;
    }

    static long nextCommandSequence(long localSeq, long topicSeq) {
        long highestSeen = Math.max(localSeq, topicSeq);
        if (highestSeen == Long.MAX_VALUE) {
            return Long.MAX_VALUE;
        }
        return highestSeen + 1;
    }

    public synchronized void sendCommand(DashboardCommand command) {
        // Use local-only sequence tracking. The previous approach subscribed to
        // the same topic being published, causing self-reflection race conditions.
        switch (command) {
            case ZERO_HEADING -> {
                ++zeroHeadingSeq;
                zeroHeadingPub.set(zeroHeadingSeq);
            }
            case STOP_DRIVE -> {
                ++stopDriveSeq;
                stopDrivePub.set(stopDriveSeq);
            }
            case INTAKE_HOME -> {
                ++intakeHomeSeq;
                intakeHomePub.set(intakeHomeSeq);
            }
            case ALIGN_ONLY -> {
                ++alignOnlySeq;
                alignOnlyPub.set(alignOnlySeq);
            }
            case ALIGN_SHOOT -> {
                ++alignShootSeq;
                alignShootPub.set(alignShootSeq);
            }
            case FALLBACK_SHOOT -> {
                ++fallbackShootSeq;
                fallbackShootPub.set(fallbackShootSeq);
            }
            case LEVEL1_CLIMB -> {
                ++level1ClimbSeq;
                level1ClimbPub.set(level1ClimbSeq);
            }
            case CALIBRATE_CANCODERS -> {
                ++calibrateCANcodersSeq;
                calibrateCANcodersPub.set(calibrateCANcodersSeq);
            }
            case STOP_SWERVE_VALIDATION -> {
                ++stopSwerveValidationSeq;
                stopSwerveValidationPub.set(stopSwerveValidationSeq);
            }
            default -> throw new IllegalStateException("Unhandled command " + command);
        }
    }

    public synchronized void sendAutoSelection(String autoName) {
        if (autoName == null || autoName.isBlank()) {
            return;
        }

        ++selectAutoSeq;
        selectAutoNamePub.set(autoName);
        selectAutoPub.set(selectAutoSeq);
    }

    public synchronized void sendSwerveValidation(String moduleToken, String modeToken) {
        if (moduleToken == null || moduleToken.isBlank() || modeToken == null || modeToken.isBlank()) {
            return;
        }

        ++swerveValidationSeq;
        swerveValidationModulePub.set(moduleToken);
        swerveValidationModePub.set(modeToken);
        swerveValidationPub.set(swerveValidationSeq);
    }

    @Override
    public void close() {
        nt.close();
    }
}
