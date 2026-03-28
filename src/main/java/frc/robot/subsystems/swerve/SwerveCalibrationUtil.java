// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/swerve/SwerveCalibrationUtil.java
//
// PURPOSE: Math utilities for CANcoder offset calibration.
//
// HOW CANcoder CALIBRATION WORKS:
//   1. Point all wheels straight forward.
//   2. Read each CANcoder's absolute position (with current offset applied).
//   3. Subtract the current offset to get the "raw" reading with no offset.
//   4. Negate the raw reading to get the new offset that makes 0 = straight.
//   5. Write the new offset into Constants.Swerve.*_CANCODER_OFFSET_ROT.
//
//   The sample() method does steps 2-4 automatically.
// ============================================================================
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
