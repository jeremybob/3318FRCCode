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
        double shooterLeftRps,
        double shooterRightRps,
        boolean shooterAtSpeed,
        boolean intakeHomed,
        boolean intakeLimitSwitchPressed,
        double intakeTiltDeg,
        double intakeRollerCurrentAmps,
        double feederCurrentAmps,
        double hopperCurrentAmps,
        boolean climberArmed,
        double climberPositionRot,
        double climberCurrentAmps,
        String alignState,
        boolean alignCommandActive,
        boolean alignHasTarget,
        boolean alignGeometryFeasible,
        boolean alignHasShootableTarget,
        double alignYawDeg,
        double alignPitchDeg,
        String alignAbortReason,
        boolean readyToScore,
        String readyReason,
        // System health
        double batteryVoltage,
        boolean brownoutAlert,
        boolean isBrownout,
        // Auto selection & execution
        String selectedAutoName,
        boolean autoCommandRunning,
        // Match info
        int matchNumber,
        String eventName,
        // Camera / vision connection
        boolean cameraConnected,
        // CAN bus health
        double canBusUtilization,
        int canReceiveErrorCount,
        int canTransmitErrorCount,
        // Swerve module angles (FL, FR, BL, BR)
        double swerveFLAngleDeg,
        double swerveFRAngleDeg,
        double swerveBLAngleDeg,
        double swerveBRAngleDeg,
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
        String controlEventMessage) {
}
