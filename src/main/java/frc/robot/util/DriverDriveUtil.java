package frc.robot.util;

import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

public final class DriverDriveUtil {

    private DriverDriveUtil() {}

    public record DriveRequest(
            double xVelocityMps,
            double yVelocityMps,
            double omegaRadPerSec,
            boolean fieldRelative,
            boolean precisionMode) {

        public double translationSpeedMps() {
            return Math.hypot(xVelocityMps, yVelocityMps);
        }
    }

    public static DriveRequest shapeDrive(
            double rawForward,
            double rawLeft,
            double rawTurn,
            boolean precisionMode,
            boolean fieldRelative) {
        double filteredX = MathUtil.applyDeadband(rawForward, Constants.Swerve.JOYSTICK_DEADBAND);
        double filteredY = MathUtil.applyDeadband(rawLeft, Constants.Swerve.JOYSTICK_DEADBAND);
        double filteredOmega = MathUtil.applyDeadband(rawTurn, Constants.Swerve.JOYSTICK_DEADBAND);

        double speedScale = precisionMode
                ? Constants.Swerve.PRECISION_SPEED_SCALE
                : 1.0;

        return new DriveRequest(
                filteredX * Constants.Swerve.MAX_TRANSLATION_MPS * speedScale,
                filteredY * Constants.Swerve.MAX_TRANSLATION_MPS * speedScale,
                filteredOmega * Constants.Swerve.MAX_ROTATION_RADPS * speedScale,
                fieldRelative,
                precisionMode);
    }
}
