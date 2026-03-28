// ============================================================================
// FILE: src/main/java/frc/robot/util/ShotSolver.java
//
// PURPOSE: Physics-based shot solver that computes coupled (angle, speed)
//   pairs using projectile motion equations.
//
// PHYSICS:
//   A game piece launched at angle θ with speed v₀ follows:
//     x(t) = v₀·cos(θ)·t
//     y(t) = v₀·sin(θ)·t - ½·g·t²
//
//   To reach horizontal distance d and height Δh:
//     v₀² = g·d² / (2·cos²θ·(d·tanθ - Δh))
//
//   The minimum-speed angle (most energy-efficient trajectory) is:
//     θ_opt = 45° + arctan(Δh/d) / 2
//
//   When the optimal angle falls outside the hood's physical range, we
//   clamp to the nearest limit and compute the higher speed required for
//   that constrained angle.
// ============================================================================
package frc.robot.util;

import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

public final class ShotSolver {

    private static final double GRAVITY_MPS2 = 9.81;

    private ShotSolver() {}

    // ---- Result record ----

    public record Solution(
            double angleDeg,
            double launchSpeedMps,
            double motorRps,
            double timeOfFlightSec,
            boolean feasible) {}

    // --------------------------------------------------------------------------
    // solve()
    //
    // Given horizontal distance to the target, returns the optimal hood angle
    // and required motor RPS as a single coupled solution.
    // --------------------------------------------------------------------------
    public static Solution solve(double distanceM) {
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return fallback();
        }

        double deltaH = Constants.Shooter.HUB_SCORING_HEIGHT_M
                       - Constants.Shooter.SHOOTER_EXIT_HEIGHT_M;

        // Optimal angle for minimum launch speed
        double optimalDeg = optimalAngleDeg(distanceM, deltaH);

        // Clamp to hood physical limits
        double angleDeg = MathUtil.clamp(
                optimalDeg,
                Constants.Hood.MIN_ANGLE_DEG,
                Constants.Hood.MAX_ANGLE_DEG);

        // Compute required launch speed for this angle
        double launchSpeedMps = requiredLaunchSpeed(distanceM, deltaH, angleDeg);
        if (!Double.isFinite(launchSpeedMps) || launchSpeedMps <= 0.0) {
            return fallback();
        }

        double motorRps = launchSpeedToMotorRps(launchSpeedMps);
        if (motorRps > Constants.Shooter.MAX_SHOT_RPS) {
            // Shot is beyond mechanism capability at this angle.
            return new Solution(angleDeg, launchSpeedMps, motorRps, Double.NaN, false);
        }

        double horizontalSpeed = launchSpeedMps * Math.cos(Math.toRadians(angleDeg));
        double tof = (horizontalSpeed > 1e-6) ? distanceM / horizontalSpeed : Double.NaN;

        return new Solution(angleDeg, launchSpeedMps, motorRps, tof, true);
    }

    // --------------------------------------------------------------------------
    // Optimal angle — minimizes required launch speed
    //
    //   θ_opt = 45° + arctan(Δh / d) / 2
    //
    // Derivation: for fixed d and Δh, differentiate v₀²(θ) with respect to θ,
    // set to zero, and solve. This yields the well-known minimum-energy angle.
    // --------------------------------------------------------------------------
    static double optimalAngleDeg(double distanceM, double deltaH) {
        double alpha = Math.atan2(deltaH, distanceM);
        return Math.toDegrees(Math.PI / 4.0 + alpha / 2.0);
    }

    // --------------------------------------------------------------------------
    // Required launch speed for a given angle
    //
    //   v₀² = g·d² / (2·cos²θ·(d·tanθ - Δh))
    //
    // Returns NaN if the angle is too shallow for the ball to reach the target
    // height (denominator ≤ 0).
    // --------------------------------------------------------------------------
    static double requiredLaunchSpeed(double distanceM, double deltaH, double angleDeg) {
        double angleRad = Math.toRadians(angleDeg);
        double cosA = Math.cos(angleRad);
        double tanA = Math.tan(angleRad);

        double denominator = distanceM * tanA - deltaH;
        if (denominator <= 1e-6) {
            // Angle too shallow — trajectory can't reach target height.
            return Double.NaN;
        }

        double v0sq = GRAVITY_MPS2 * distanceM * distanceM
                    / (2.0 * cosA * cosA * denominator);
        return Math.sqrt(v0sq);
    }

    // --------------------------------------------------------------------------
    // Motor RPS ↔ launch speed conversions
    //
    // Launch speed = wheel surface speed = wheelRPS × circumference.
    // Motor RPS = wheel RPS × gear ratio.
    // --------------------------------------------------------------------------
    static double launchSpeedToMotorRps(double launchSpeedMps) {
        double wheelRps = launchSpeedMps / Constants.Shooter.WHEEL_CIRCUMFERENCE_M;
        return wheelRps * Constants.Shooter.GEAR_RATIO;
    }

    static double motorRpsToLaunchSpeed(double motorRps) {
        return motorRps / Constants.Shooter.GEAR_RATIO
                * Constants.Shooter.WHEEL_CIRCUMFERENCE_M;
    }

    // --------------------------------------------------------------------------
    // Fallback solution when distance is invalid
    // --------------------------------------------------------------------------
    private static Solution fallback() {
        double launchSpeed = motorRpsToLaunchSpeed(Constants.Shooter.TARGET_RPS);
        return new Solution(
                Constants.Hood.DEFAULT_ANGLE_DEG,
                launchSpeed,
                Constants.Shooter.TARGET_RPS,
                Double.NaN,
                false);
    }
}
