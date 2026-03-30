// ============================================================================
// FILE: src/main/java/frc/robot/dashboard/DashboardSnapshot.java
//
// PURPOSE: Immutable snapshot of ALL robot telemetry published to the custom
//   dashboard each cycle. Because it is a Java record, every field is final
//   and the snapshot is safe to pass between threads without synchronization.
//
// IMPORTANT: Field order matters! The record constructor is positional, so
//   every call site that creates a DashboardSnapshot must match this exact
//   field order. If you add or reorder fields, update all constructors.
// ============================================================================
package frc.robot.dashboard;

public record DashboardSnapshot(
        double timestampSec,
        String robotMode,
        boolean enabled,
        String alliance,
        double matchTimeSec,
        double poseX_m,
        double poseY_m,
        double headingDeg,
        // Raw Pigeon2 IMU telemetry
        double pigeonYawDeg,
        double pigeonPitchDeg,
        double pigeonRollDeg,
        // Driver turn debug
        double driverRawTurnInput,
        double driverCommandedTranslationMps,
        double driverCommandedOmegaRadPerSec,
        double measuredOmegaRadPerSec,
        boolean driverFieldRelativeEnabled,
        boolean headingHoldActive,
        double headingHoldTargetDeg,
        double headingHoldErrorDeg,
        double headingHoldCorrectionOmegaRadPerSec,
        double shooterLeftRps,
        double shooterRightRps,
        boolean shooterAtSpeed,
        boolean intakeHomed,
        boolean intakeLimitSwitchPressed,
        double intakeSlideIn,
        double intakeRollerCurrentAmps,
        double feederCurrentAmps,
        double hopperCurrentAmps,
        boolean climberArmed,
        double climberPositionRot,
        double climberCurrentAmps,
        // Alignment pipeline (pose-based heading to HUB center)
        String alignState,
        boolean alignCommandActive,
        boolean alignHasTarget,
        boolean alignGeometryFeasible,
        boolean alignHasShootableTarget,
        double alignHeadingErrorDeg,
        double alignAimErrorDeg,
        double alignDistanceM,
        double alignTargetRps,
        boolean alignFeedGateReady,
        String alignAbortReason,
        boolean alignPositionHoldActive,
        double alignPositionHoldErrorM,
        boolean readyToScore,
        String readyReason,
        // 2026 REBUILT: HUB shift activity
        boolean hubActive,
        double hubSecondsToNextShift,
        // System health
        double batteryVoltage,
        boolean brownoutAlert,
        boolean isBrownout,
        // Auto selection & execution
        String selectedAutoName,
        String selectedAutoSource,
        String[] autoOptions,
        boolean autoCommandRunning,
        // Expected auto starting pose (blue-side coordinates; NaN = no starting pose)
        double autoStartX_m,
        double autoStartY_m,
        double autoStartHeadingDeg,
        // Match info
        long matchNumber,
        String eventName,
        // Camera / vision connection
        boolean cameraConnected,
        String cameraStatus,
        String cameraName,
        // Vision pose estimation
        int visionTagId,
        boolean visionHasTarget,
        double visionHeadingErrorDeg,
        double visionDistanceM,
        int visionHubTagCount,
        double visionTargetTimestampSec,
        // CAN bus health
        double canBusUtilization,
        long canReceiveErrorCount,
        long canTransmitErrorCount,
        // Swerve module angles (FL, FR, BL, BR)
        double swerveFLAngleDeg,
        double swerveFRAngleDeg,
        double swerveBLAngleDeg,
        double swerveBRAngleDeg,
        // CANCoder health (per-module)
        double cancoderFLPosRot,
        double cancoderFRPosRot,
        double cancoderBLPosRot,
        double cancoderBRPosRot,
        double cancoderFLAbsRawRot,
        double cancoderFRAbsRawRot,
        double cancoderBLAbsRawRot,
        double cancoderBRAbsRawRot,
        boolean cancoderFLOk,
        boolean cancoderFROk,
        boolean cancoderBLOk,
        boolean cancoderBROk,
        // Motor temperatures (Celsius)
        double driveFLTempC,
        double driveFRTempC,
        double driveBLTempC,
        double driveBRTempC,
        double shooterLeftTempC,
        double shooterRightTempC,
        // Controller diagnostics
        String driverButtonsActive,
        String operatorButtonsActive,
        long controlEventSeq,
        double controlEventTimestampSec,
        String controlEventMessage,
        boolean swerveValidationActive,
        String swerveValidationModuleToken,
        String swerveValidationModuleDisplayName,
        String swerveValidationModeToken,
        String swerveValidationModeDisplayName,
        double swerveValidationDrivePercent,
        double swerveValidationSteerPercent,
        double swerveValidationStartAngleDeg,
        double swerveValidationAngleDeltaDeg,
        double swerveValidationStartCANcoderRot,
        double swerveValidationCANcoderDeltaRot) {
}
