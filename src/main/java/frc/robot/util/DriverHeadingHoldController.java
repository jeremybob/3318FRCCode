// ============================================================================
// FILE: src/main/java/frc/robot/util/DriverHeadingHoldController.java
//
// PURPOSE: Prevents the robot from slowly drifting/rotating when the driver
//   is driving straight (translating without turning). Without this, slight
//   mechanical imbalances or wheel scrub cause unwanted rotation.
//
// HOW IT WORKS:
//   1. When the driver starts translating without any turn input, this
//      controller "captures" the current heading as the target.
//   2. A PID loop applies small rotational corrections to hold that heading.
//   3. When the driver commands a turn (right stick X), the controller
//      releases and lets the driver rotate freely.
//   4. When the driver stops translating, the controller resets.
//
// The Pigeon 2 gyro provides the heading measurement.
// ============================================================================
package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;

/**
 * Holds the robot's current heading while the driver is translating without
 * commanding turn. The Pigeon yaw is the feedback source.
 */
public class DriverHeadingHoldController {

    public record Output(
            boolean active,
            double targetHeadingDeg,
            double headingErrorDeg,
            double correctionOmegaRadPerSec,
            double commandedOmegaRadPerSec) {}

    private final PIDController headingPid = new PIDController(
            Constants.Swerve.HEADING_HOLD_kP,
            0.0,
            Constants.Swerve.HEADING_HOLD_kD);

    private boolean active;
    private double targetHeadingDeg = Double.NaN;

    public DriverHeadingHoldController() {
        headingPid.enableContinuousInput(-180.0, 180.0);
        headingPid.setTolerance(Constants.Swerve.HEADING_HOLD_TOLERANCE_DEG);
    }

    public Output update(
            double currentHeadingDeg,
            double translationSpeedMps,
            double requestedOmegaRadPerSec) {
        if (!Constants.Swerve.ENABLE_HEADING_HOLD_ASSIST || !Double.isFinite(currentHeadingDeg)) {
            reset(currentHeadingDeg);
            return passiveOutput(currentHeadingDeg, requestedOmegaRadPerSec);
        }

        boolean turnIntent = Math.abs(requestedOmegaRadPerSec) > 1e-6;
        boolean translationIntent = translationSpeedMps >= Constants.Swerve.HEADING_HOLD_MIN_TRANSLATION_MPS;

        if (turnIntent) {
            reset(currentHeadingDeg);
            return passiveOutput(currentHeadingDeg, requestedOmegaRadPerSec);
        }

        if (!translationIntent) {
            reset(currentHeadingDeg);
            return passiveOutput(currentHeadingDeg, 0.0);
        }

        if (!active || !Double.isFinite(targetHeadingDeg)) {
            targetHeadingDeg = currentHeadingDeg;
            headingPid.reset();
            active = true;
        }

        double headingErrorDeg = MathUtil.inputModulus(
                targetHeadingDeg - currentHeadingDeg,
                -180.0,
                180.0);
        double correctionOmegaRadPerSec = Math.abs(headingErrorDeg) <= Constants.Swerve.HEADING_HOLD_TOLERANCE_DEG
                ? 0.0
                : MathUtil.clamp(
                        headingPid.calculate(currentHeadingDeg, targetHeadingDeg),
                        -Constants.Swerve.HEADING_HOLD_MAX_OMEGA_RADPS,
                        Constants.Swerve.HEADING_HOLD_MAX_OMEGA_RADPS);

        return new Output(
                true,
                targetHeadingDeg,
                headingErrorDeg,
                correctionOmegaRadPerSec,
                correctionOmegaRadPerSec);
    }

    public void reset(double currentHeadingDeg) {
        active = false;
        targetHeadingDeg = Double.isFinite(currentHeadingDeg) ? currentHeadingDeg : Double.NaN;
        headingPid.reset();
    }

    private Output passiveOutput(double currentHeadingDeg, double commandedOmegaRadPerSec) {
        return new Output(
                false,
                Double.isFinite(targetHeadingDeg) ? targetHeadingDeg : currentHeadingDeg,
                0.0,
                0.0,
                commandedOmegaRadPerSec);
    }
}
