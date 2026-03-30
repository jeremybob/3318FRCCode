// ============================================================================
// FILE: src/main/java/frc/robot/dashboard/RobotDashboardService.java
//
// PURPOSE: Bridge between the robot's subsystem state and the custom Swing
//   dashboard. Every 100 ms, this service publishes a DashboardSnapshot to
//   NetworkTables so the dashboard can display telemetry, and it reads back
//   operator commands (e.g., "run swerve validation", "select auto", "zero
//   heading") that the dashboard sends as NT string values.
//
//   CONTRACT_VERSION is a string that must match between this file and
//   DashboardNtClient on the dashboard side. If they mismatch, the dashboard
//   shows a warning so students know the robot code and dashboard are out of
//   sync.
// ============================================================================
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

import frc.robot.subsystems.swerve.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveValidationMode;

public class RobotDashboardService {
    // Publish telemetry at 10 Hz (every 0.1 s). Fast enough for smooth dashboard
    // updates, slow enough to avoid flooding NetworkTables traffic.
    private static final double SNAPSHOT_PUBLISH_PERIOD_SEC = 0.1;

    public interface Actions {
        void zeroHeading();
        void stopDrive();
        void scheduleIntakeHome();
        void scheduleAlignOnly();
        void scheduleAlignAndShoot();
        void scheduleFallbackShoot();
        void scheduleLevel1Climb();
        void scheduleCANcoderCalibration();
        void requestSwerveValidation(String moduleName, String modeName);
        void stopSwerveValidation();
        void selectAutoByName(String autoName);
        void selectCamera(String cameraTypeName);
        String getActiveCameraTypeName();
    }

    private static final String CONTRACT_VERSION = "2026.16.0";

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
    private final DoublePublisher driverRawTurnInputPub;
    private final DoublePublisher driverCommandedTranslationPub;
    private final DoublePublisher driverCommandedOmegaPub;
    private final DoublePublisher measuredOmegaPub;
    private final BooleanPublisher driverFieldRelativeEnabledPub;
    private final BooleanPublisher headingHoldActivePub;
    private final DoublePublisher headingHoldTargetPub;
    private final DoublePublisher headingHoldErrorPub;
    private final DoublePublisher headingHoldCorrectionPub;

    private final DoublePublisher shooterLeftRpsPub;
    private final DoublePublisher shooterRightRpsPub;
    private final BooleanPublisher shooterAtSpeedPub;

    private final BooleanPublisher intakeHomedPub;
    private final BooleanPublisher intakeLimitPub;
    private final DoublePublisher intakeSlidePub;
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
    private final DoublePublisher alignHeadingErrorPub;
    private final DoublePublisher alignAimErrorPub;
    private final DoublePublisher alignDistancePub;
    private final DoublePublisher alignTargetRpsPub;
    private final BooleanPublisher alignFeedGateReadyPub;
    private final StringPublisher alignAbortReasonPub;
    private final BooleanPublisher alignPositionHoldActivePub;
    private final DoublePublisher alignPositionHoldErrorPub;

    private final BooleanPublisher readyToScorePub;
    private final StringPublisher readyReasonPub;

    // 2026 REBUILT: HUB shift activity
    private final BooleanPublisher hubActivePub;
    private final DoublePublisher hubSecondsToNextShiftPub;

    // System health
    private final DoublePublisher batteryVoltagePub;
    private final BooleanPublisher brownoutAlertPub;
    private final BooleanPublisher isBrownoutPub;

    // Auto selection & execution
    private final StringPublisher selectedAutoNamePub;
    private final StringPublisher selectedAutoSourcePub;
    private final StringArrayPublisher autoOptionsPub;
    private final BooleanPublisher autoCommandRunningPub;

    // Expected auto starting pose
    private final DoublePublisher autoStartXPub;
    private final DoublePublisher autoStartYPub;
    private final DoublePublisher autoStartHeadingPub;

    // Match info
    private final IntegerPublisher matchNumberPub;
    private final StringPublisher eventNamePub;

    // Camera / vision connection
    private final BooleanPublisher cameraConnectedPub;
    private final StringPublisher cameraStatusPub;
    private final StringPublisher cameraNamePub;
    private final StringPublisher activeCameraTypePub;
    // Vision pose estimation
    private final IntegerPublisher visionTagIdPub;
    private final BooleanPublisher visionHasTargetPub;
    private final DoublePublisher visionHeadingErrorPub;
    private final DoublePublisher visionDistancePub;
    private final IntegerPublisher visionHubTagCountPub;
    private final DoublePublisher visionTargetTimestampPub;

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

    private final BooleanPublisher swerveValidationActivePub;
    private final StringPublisher swerveValidationModuleTokenPub;
    private final StringPublisher swerveValidationModuleDisplayPub;
    private final StringPublisher swerveValidationModeTokenPub;
    private final StringPublisher swerveValidationModeDisplayPub;
    private final DoublePublisher swerveValidationDrivePercentPub;
    private final DoublePublisher swerveValidationSteerPercentPub;
    private final DoublePublisher swerveValidationStartAnglePub;
    private final DoublePublisher swerveValidationAngleDeltaPub;
    private final DoublePublisher swerveValidationStartCANcoderPub;
    private final DoublePublisher swerveValidationCANcoderDeltaPub;

    private final StringPublisher lastCommandPub;
    private final StringPublisher lastStatusPub;
    private final IntegerPublisher lastSeqPub;
    private final StringPublisher lastMessagePub;
    private final DoublePublisher lastAckTimestampPub;

    private final IntegerSubscriber zeroHeadingCmdSub;
    private final IntegerSubscriber stopDriveCmdSub;
    private final IntegerSubscriber intakeHomeCmdSub;
    private final IntegerSubscriber alignOnlyCmdSub;
    private final IntegerSubscriber alignShootCmdSub;
    private final IntegerSubscriber fallbackShootCmdSub;
    private final IntegerSubscriber level1ClimbCmdSub;
    private final IntegerSubscriber calibrateCANcodersCmdSub;
    private final StringSubscriber swerveValidationModuleCmdSub;
    private final StringSubscriber swerveValidationModeCmdSub;
    private final IntegerSubscriber swerveValidationCmdSub;
    private final IntegerSubscriber stopSwerveValidationCmdSub;
    private final StringSubscriber selectAutoNameCmdSub;
    private final IntegerSubscriber selectAutoCmdSub;
    private final StringSubscriber selectCameraNameCmdSub;
    private final IntegerSubscriber selectCameraCmdSub;

    private long zeroHeadingSeqSeen = 0;
    private long stopDriveSeqSeen = 0;
    private long intakeHomeSeqSeen = 0;
    private long alignOnlySeqSeen = 0;
    private long alignShootSeqSeen = 0;
    private long fallbackShootSeqSeen = 0;
    private long level1ClimbSeqSeen = 0;
    private long calibrateCANcodersSeqSeen = 0;
    private long swerveValidationSeqSeen = 0;
    private long stopSwerveValidationSeqSeen = 0;
    private long selectAutoSeqSeen = 0;
    private long selectCameraSeqSeen = 0;
    private double lastSnapshotPublishTimestampSec = Double.NEGATIVE_INFINITY;

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
        driverRawTurnInputPub = table.getDoubleTopic("drive/raw_turn_input").publish();
        driverCommandedTranslationPub = table.getDoubleTopic("drive/commanded_translation_mps").publish();
        driverCommandedOmegaPub = table.getDoubleTopic("drive/commanded_omega_radps").publish();
        measuredOmegaPub = table.getDoubleTopic("drive/measured_omega_radps").publish();
        driverFieldRelativeEnabledPub = table.getBooleanTopic("drive/field_relative_enabled").publish();
        headingHoldActivePub = table.getBooleanTopic("drive/heading_hold_active").publish();
        headingHoldTargetPub = table.getDoubleTopic("drive/heading_hold_target_deg").publish();
        headingHoldErrorPub = table.getDoubleTopic("drive/heading_hold_error_deg").publish();
        headingHoldCorrectionPub = table.getDoubleTopic("drive/heading_hold_correction_radps").publish();

        shooterLeftRpsPub = table.getDoubleTopic("shooter/left_rps").publish();
        shooterRightRpsPub = table.getDoubleTopic("shooter/right_rps").publish();
        shooterAtSpeedPub = table.getBooleanTopic("shooter/at_speed").publish();

        intakeHomedPub = table.getBooleanTopic("intake/homed").publish();
        intakeLimitPub = table.getBooleanTopic("intake/limit_switch_pressed").publish();
        intakeSlidePub = table.getDoubleTopic("intake/slide_in").publish();
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
        alignHeadingErrorPub = table.getDoubleTopic("align/heading_error_deg").publish();
        alignAimErrorPub = table.getDoubleTopic("align/aim_error_deg").publish();
        alignDistancePub = table.getDoubleTopic("align/distance_m").publish();
        alignTargetRpsPub = table.getDoubleTopic("align/target_rps").publish();
        alignFeedGateReadyPub = table.getBooleanTopic("align/feed_gate_ready").publish();
        alignAbortReasonPub = table.getStringTopic("align/last_abort_reason").publish();
        alignPositionHoldActivePub = table.getBooleanTopic("align/position_hold_active").publish();
        alignPositionHoldErrorPub = table.getDoubleTopic("align/position_hold_error_m").publish();

        readyToScorePub = table.getBooleanTopic("shot/ready").publish();
        readyReasonPub = table.getStringTopic("shot/ready_reason").publish();

        // 2026 REBUILT: HUB shift activity
        hubActivePub = table.getBooleanTopic("hub/active").publish();
        hubSecondsToNextShiftPub = table.getDoubleTopic("hub/seconds_to_next_shift").publish();

        // System health
        batteryVoltagePub = table.getDoubleTopic("health/battery_voltage").publish();
        brownoutAlertPub = table.getBooleanTopic("health/brownout_alert").publish();
        isBrownoutPub = table.getBooleanTopic("health/is_brownout").publish();

        // Auto selection & execution
        selectedAutoNamePub = table.getStringTopic("auto/selected_name").publish();
        selectedAutoSourcePub = table.getStringTopic("auto/selected_source").publish();
        autoOptionsPub = table.getStringArrayTopic("auto/options").publish();
        autoCommandRunningPub = table.getBooleanTopic("auto/command_running").publish();

        // Expected auto starting pose
        autoStartXPub = table.getDoubleTopic("auto/start_x_m").publish();
        autoStartYPub = table.getDoubleTopic("auto/start_y_m").publish();
        autoStartHeadingPub = table.getDoubleTopic("auto/start_heading_deg").publish();

        // Match info
        matchNumberPub = table.getIntegerTopic("match/number").publish();
        eventNamePub = table.getStringTopic("match/event_name").publish();

        // Camera / vision connection
        cameraConnectedPub = table.getBooleanTopic("vision/camera_connected").publish();
        cameraStatusPub = table.getStringTopic("vision/camera_status").publish();
        cameraNamePub = table.getStringTopic("vision/camera_name").publish();
        activeCameraTypePub = table.getStringTopic("vision/active_camera").publish();
        // Vision pose estimation
        visionTagIdPub = table.getIntegerTopic("vision/tag_id").publish();
        visionHasTargetPub = table.getBooleanTopic("vision/has_target").publish();
        visionHeadingErrorPub = table.getDoubleTopic("vision/heading_error_deg").publish();
        visionDistancePub = table.getDoubleTopic("vision/distance_m").publish();
        visionHubTagCountPub = table.getIntegerTopic("vision/hub_tag_count").publish();
        visionTargetTimestampPub = table.getDoubleTopic("vision/target_timestamp_sec").publish();

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

        swerveValidationActivePub = table.getBooleanTopic("swerve/validation_active").publish();
        swerveValidationModuleTokenPub = table.getStringTopic("swerve/validation_module_token").publish();
        swerveValidationModuleDisplayPub = table.getStringTopic("swerve/validation_module_display").publish();
        swerveValidationModeTokenPub = table.getStringTopic("swerve/validation_mode_token").publish();
        swerveValidationModeDisplayPub = table.getStringTopic("swerve/validation_mode_display").publish();
        swerveValidationDrivePercentPub = table.getDoubleTopic("swerve/validation_drive_percent").publish();
        swerveValidationSteerPercentPub = table.getDoubleTopic("swerve/validation_steer_percent").publish();
        swerveValidationStartAnglePub = table.getDoubleTopic("swerve/validation_start_angle_deg").publish();
        swerveValidationAngleDeltaPub = table.getDoubleTopic("swerve/validation_angle_delta_deg").publish();
        swerveValidationStartCANcoderPub = table.getDoubleTopic("swerve/validation_start_cancoder_rot").publish();
        swerveValidationCANcoderDeltaPub = table.getDoubleTopic("swerve/validation_cancoder_delta_rot").publish();

        lastCommandPub = table.getStringTopic("ack/last_command").publish();
        lastStatusPub = table.getStringTopic("ack/last_status").publish();
        lastSeqPub = table.getIntegerTopic("ack/last_seq").publish();
        lastMessagePub = table.getStringTopic("ack/message").publish();
        lastAckTimestampPub = table.getDoubleTopic("ack/timestamp_sec").publish();

        zeroHeadingCmdSub = table.getIntegerTopic("cmd/zero_heading_seq").subscribe(0);
        stopDriveCmdSub = table.getIntegerTopic("cmd/stop_drive_seq").subscribe(0);
        intakeHomeCmdSub = table.getIntegerTopic("cmd/intake_home_seq").subscribe(0);
        alignOnlyCmdSub = table.getIntegerTopic("cmd/align_only_seq").subscribe(0);
        alignShootCmdSub = table.getIntegerTopic("cmd/align_shoot_seq").subscribe(0);
        fallbackShootCmdSub = table.getIntegerTopic("cmd/fallback_shoot_seq").subscribe(0);
        level1ClimbCmdSub = table.getIntegerTopic("cmd/level1_climb_seq").subscribe(0);
        calibrateCANcodersCmdSub = table.getIntegerTopic("cmd/calibrate_cancoders_seq").subscribe(0);
        swerveValidationModuleCmdSub = table.getStringTopic("cmd/swerve_validation_module").subscribe("");
        swerveValidationModeCmdSub = table.getStringTopic("cmd/swerve_validation_mode").subscribe("");
        swerveValidationCmdSub = table.getIntegerTopic("cmd/swerve_validation_seq").subscribe(0);
        stopSwerveValidationCmdSub = table.getIntegerTopic("cmd/stop_swerve_validation_seq").subscribe(0);
        selectAutoNameCmdSub = table.getStringTopic("cmd/select_auto_name").subscribe("");
        selectAutoCmdSub = table.getIntegerTopic("cmd/select_auto_seq").subscribe(0);
        selectCameraNameCmdSub = table.getStringTopic("cmd/select_camera_name").subscribe("");
        selectCameraCmdSub = table.getIntegerTopic("cmd/select_camera_seq").subscribe(0);

        // Ignore any stale sequence values already present when robot code starts.
        zeroHeadingSeqSeen = zeroHeadingCmdSub.get();
        stopDriveSeqSeen = stopDriveCmdSub.get();
        intakeHomeSeqSeen = intakeHomeCmdSub.get();
        alignOnlySeqSeen = alignOnlyCmdSub.get();
        alignShootSeqSeen = alignShootCmdSub.get();
        fallbackShootSeqSeen = fallbackShootCmdSub.get();
        level1ClimbSeqSeen = level1ClimbCmdSub.get();
        calibrateCANcodersSeqSeen = calibrateCANcodersCmdSub.get();
        swerveValidationSeqSeen = swerveValidationCmdSub.get();
        stopSwerveValidationSeqSeen = stopSwerveValidationCmdSub.get();
        selectAutoSeqSeen = selectAutoCmdSub.get();
        selectCameraSeqSeen = selectCameraCmdSub.get();
    }

    public void periodic(DashboardSnapshot snapshot) {
        if (snapshot.timestampSec() - lastSnapshotPublishTimestampSec >= SNAPSHOT_PUBLISH_PERIOD_SEC) {
            publishSnapshot(snapshot);
            lastSnapshotPublishTimestampSec = snapshot.timestampSec();
        }
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
        driverRawTurnInputPub.set(snapshot.driverRawTurnInput());
        driverCommandedTranslationPub.set(snapshot.driverCommandedTranslationMps());
        driverCommandedOmegaPub.set(snapshot.driverCommandedOmegaRadPerSec());
        measuredOmegaPub.set(snapshot.measuredOmegaRadPerSec());
        driverFieldRelativeEnabledPub.set(snapshot.driverFieldRelativeEnabled());
        headingHoldActivePub.set(snapshot.headingHoldActive());
        headingHoldTargetPub.set(snapshot.headingHoldTargetDeg());
        headingHoldErrorPub.set(snapshot.headingHoldErrorDeg());
        headingHoldCorrectionPub.set(snapshot.headingHoldCorrectionOmegaRadPerSec());

        shooterLeftRpsPub.set(snapshot.shooterLeftRps());
        shooterRightRpsPub.set(snapshot.shooterRightRps());
        shooterAtSpeedPub.set(snapshot.shooterAtSpeed());

        intakeHomedPub.set(snapshot.intakeHomed());
        intakeLimitPub.set(snapshot.intakeLimitSwitchPressed());
        intakeSlidePub.set(snapshot.intakeSlideIn());
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
        alignHeadingErrorPub.set(snapshot.alignHeadingErrorDeg());
        alignAimErrorPub.set(snapshot.alignAimErrorDeg());
        alignDistancePub.set(snapshot.alignDistanceM());
        alignTargetRpsPub.set(snapshot.alignTargetRps());
        alignFeedGateReadyPub.set(snapshot.alignFeedGateReady());
        alignAbortReasonPub.set(snapshot.alignAbortReason());
        alignPositionHoldActivePub.set(snapshot.alignPositionHoldActive());
        alignPositionHoldErrorPub.set(snapshot.alignPositionHoldErrorM());

        readyToScorePub.set(snapshot.readyToScore());
        readyReasonPub.set(snapshot.readyReason());

        // 2026 REBUILT: HUB shift activity
        hubActivePub.set(snapshot.hubActive());
        hubSecondsToNextShiftPub.set(snapshot.hubSecondsToNextShift());

        // System health
        batteryVoltagePub.set(snapshot.batteryVoltage());
        brownoutAlertPub.set(snapshot.brownoutAlert());
        isBrownoutPub.set(snapshot.isBrownout());

        // Auto selection & execution
        selectedAutoNamePub.set(snapshot.selectedAutoName());
        selectedAutoSourcePub.set(snapshot.selectedAutoSource());
        autoOptionsPub.set(snapshot.autoOptions());
        autoCommandRunningPub.set(snapshot.autoCommandRunning());

        // Expected auto starting pose
        autoStartXPub.set(snapshot.autoStartX_m());
        autoStartYPub.set(snapshot.autoStartY_m());
        autoStartHeadingPub.set(snapshot.autoStartHeadingDeg());

        // Match info
        matchNumberPub.set(snapshot.matchNumber());
        eventNamePub.set(snapshot.eventName());

        // Camera / vision connection (PhotonVision on Raspberry Pi 4)
        cameraConnectedPub.set(snapshot.cameraConnected());
        cameraStatusPub.set(snapshot.cameraStatus());
        cameraNamePub.set(snapshot.cameraName());
        activeCameraTypePub.set(actions.getActiveCameraTypeName());
        // Vision pose estimation
        visionTagIdPub.set(snapshot.visionTagId());
        visionHasTargetPub.set(snapshot.visionHasTarget());
        visionHeadingErrorPub.set(snapshot.visionHeadingErrorDeg());
        visionDistancePub.set(snapshot.visionDistanceM());
        visionHubTagCountPub.set(snapshot.visionHubTagCount());
        visionTargetTimestampPub.set(snapshot.visionTargetTimestampSec());

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

        swerveValidationActivePub.set(snapshot.swerveValidationActive());
        swerveValidationModuleTokenPub.set(snapshot.swerveValidationModuleToken());
        swerveValidationModuleDisplayPub.set(snapshot.swerveValidationModuleDisplayName());
        swerveValidationModeTokenPub.set(snapshot.swerveValidationModeToken());
        swerveValidationModeDisplayPub.set(snapshot.swerveValidationModeDisplayName());
        swerveValidationDrivePercentPub.set(snapshot.swerveValidationDrivePercent());
        swerveValidationSteerPercentPub.set(snapshot.swerveValidationSteerPercent());
        swerveValidationStartAnglePub.set(snapshot.swerveValidationStartAngleDeg());
        swerveValidationAngleDeltaPub.set(snapshot.swerveValidationAngleDeltaDeg());
        swerveValidationStartCANcoderPub.set(snapshot.swerveValidationStartCANcoderRot());
        swerveValidationCANcoderDeltaPub.set(snapshot.swerveValidationCANcoderDeltaRot());
    }

    private void processCommandRequests(DashboardSnapshot snapshot) {
        boolean disabled = "DISABLED".equals(snapshot.robotMode());
        boolean teleopEnabled = snapshot.enabled() && "TELEOP".equals(snapshot.robotMode());
        boolean validationEnabled = snapshot.enabled()
                && ("TELEOP".equals(snapshot.robotMode()) || "TEST".equals(snapshot.robotMode()));
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

        alignOnlySeqSeen = runCommandIfNew(
                alignOnlyCmdSub,
                alignOnlySeqSeen,
                "align_only",
                actions::scheduleAlignOnly,
                teleopEnabled,
                "Only allowed in enabled teleop",
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

        // --- CLIMBER DISABLED: level1_climb always rejected ---
        level1ClimbSeqSeen = runCommandIfNew(
                level1ClimbCmdSub,
                level1ClimbSeqSeen,
                "level1_climb",
                actions::scheduleLevel1Climb,
                false,
                "Climber disabled — no hardware installed",
                snapshot.timestampSec());

        calibrateCANcodersSeqSeen = runCommandIfNew(
                calibrateCANcodersCmdSub,
                calibrateCANcodersSeqSeen,
                "calibrate_cancoders",
                actions::scheduleCANcoderCalibration,
                true,
                "Accepted",
                snapshot.timestampSec());

        swerveValidationSeqSeen = runSwerveValidationIfNew(
                snapshot,
                swerveValidationCmdSub,
                swerveValidationModuleCmdSub,
                swerveValidationModeCmdSub,
                swerveValidationSeqSeen,
                snapshot.timestampSec(),
                validationEnabled);

        stopSwerveValidationSeqSeen = runCommandIfNew(
                stopSwerveValidationCmdSub,
                stopSwerveValidationSeqSeen,
                "stop_swerve_validation",
                actions::stopSwerveValidation,
                true,
                "Accepted",
                snapshot.timestampSec());

        selectAutoSeqSeen = runAutoSelectionIfNew(
                snapshot,
                selectAutoCmdSub,
                selectAutoNameCmdSub,
                selectAutoSeqSeen,
                snapshot.timestampSec(),
                disabled);

        selectCameraSeqSeen = runCameraSelectionIfNew(
                selectCameraCmdSub,
                selectCameraNameCmdSub,
                selectCameraSeqSeen,
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

    private long runSwerveValidationIfNew(
            DashboardSnapshot snapshot,
            IntegerSubscriber sequenceSubscriber,
            StringSubscriber moduleSubscriber,
            StringSubscriber modeSubscriber,
            long lastSeen,
            double timestampSec,
            boolean allowed) {
        long seq = sequenceSubscriber.get();
        if (seq <= lastSeen) {
            return lastSeen;
        }

        String moduleName = moduleSubscriber.get();
        String modeName = modeSubscriber.get();
        if (!allowed) {
            publishAck("swerve_validation", "REJECTED", seq,
                    "Requires enabled teleop/test", timestampSec);
            return seq;
        }
        if (moduleName == null || moduleName.isBlank() || modeName == null || modeName.isBlank()) {
            publishAck("swerve_validation", "REJECTED", seq,
                    "Module and mode are required", timestampSec);
            return seq;
        }
        if (SwerveCorner.fromToken(moduleName) == null || SwerveValidationMode.fromToken(modeName) == null) {
            publishAck("swerve_validation", "REJECTED", seq,
                    "Unknown module or mode", timestampSec);
            return seq;
        }

        actions.requestSwerveValidation(moduleName, modeName);
        publishAck("swerve_validation", "OK", seq,
                "Accepted " + moduleName + " / " + modeName, timestampSec);
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

    private long runCameraSelectionIfNew(
            IntegerSubscriber sequenceSubscriber,
            StringSubscriber nameSubscriber,
            long lastSeen,
            double timestampSec) {
        long seq = sequenceSubscriber.get();
        if (seq <= lastSeen) {
            return lastSeen;
        }

        String requestedCamera = nameSubscriber.get();
        String normalized = requestedCamera == null ? "" : requestedCamera.trim().toUpperCase();
        if (normalized.isEmpty()) {
            publishAck("select_camera", "REJECTED", seq, "No camera name provided", timestampSec);
            return seq;
        }
        if (!"PHOTONVISION".equals(normalized) && !"LIMELIGHT".equals(normalized)) {
            publishAck("select_camera", "REJECTED", seq, "Unknown camera: " + normalized, timestampSec);
            return seq;
        }

        actions.selectCamera(normalized);
        publishAck("select_camera", "OK", seq, "Switched to " + normalized, timestampSec);
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
