package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;

public final class SwerveCalibrationUtil {

    private SwerveCalibrationUtil() {}

    public record CalibrationSample(
            double configuredAbsoluteRot,
            double noOffsetRot,
            double recommendedOffsetRot) {}

    public static CalibrationSample sample(double configuredAbsoluteRot, double configuredOffsetRot) {
        double noOffsetRot = wrapSignedRotations(configuredAbsoluteRot - configuredOffsetRot);
        return new CalibrationSample(
                configuredAbsoluteRot,
                noOffsetRot,
                wrapSignedRotations(-noOffsetRot));
    }

    public static double wrapSignedRotations(double rotations) {
        return MathUtil.inputModulus(rotations, -0.5, 0.5);
    }

    public static double angleDeltaDeg(double currentDeg, double startDeg) {
        return MathUtil.inputModulus(currentDeg - startDeg, -180.0, 180.0);
    }
}
