package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class ShotSolverTest {

    private static final double DELTA_H =
            Constants.Shooter.HUB_SCORING_HEIGHT_M - Constants.Shooter.SHOOTER_EXIT_HEIGHT_M;

    // ---- optimalAngleDeg ----

    @Test
    void optimalAngleIsAbove45WhenTargetIsHigher() {
        // Shooting up → optimal angle > 45°
        double angle = ShotSolver.optimalAngleDeg(3.0, DELTA_H);
        assertTrue(angle > 45.0, "Optimal angle for upward shot should exceed 45°, was " + angle);
    }

    @Test
    void optimalAngleIs45ForLevelTarget() {
        double angle = ShotSolver.optimalAngleDeg(5.0, 0.0);
        assertEquals(45.0, angle, 1e-9);
    }

    @Test
    void optimalAngleDecreasesWithDistance() {
        // As horizontal distance grows, the elevation angle α = atan(Δh/d) shrinks.
        double nearAngle = ShotSolver.optimalAngleDeg(2.0, DELTA_H);
        double farAngle = ShotSolver.optimalAngleDeg(6.0, DELTA_H);
        assertTrue(farAngle < nearAngle,
                "Optimal angle should decrease as distance grows: near="
                        + nearAngle + " far=" + farAngle);
    }

    // ---- requiredLaunchSpeed ----

    @Test
    void requiredSpeedIncreasesWithDistance() {
        double nearSpeed = ShotSolver.requiredLaunchSpeed(2.0, DELTA_H, 50.0);
        double farSpeed = ShotSolver.requiredLaunchSpeed(5.0, DELTA_H, 50.0);
        assertTrue(Double.isFinite(nearSpeed) && nearSpeed > 0.0);
        assertTrue(Double.isFinite(farSpeed) && farSpeed > 0.0);
        assertTrue(farSpeed > nearSpeed,
                "Farther shots need more speed: near=" + nearSpeed + " far=" + farSpeed);
    }

    @Test
    void requiredSpeedIsNaNForTooShallowAngle() {
        // At a very shallow angle the ball can't reach the target height
        double speed = ShotSolver.requiredLaunchSpeed(1.0, DELTA_H, 5.0);
        assertTrue(Double.isNaN(speed), "Too-shallow angle should return NaN");
    }

    // ---- solve ----

    @Test
    void solveFeasibleForReasonableDistance() {
        ShotSolver.Solution solution = ShotSolver.solve(3.0);
        assertTrue(solution.feasible(), "3m shot should be feasible");
        assertTrue(solution.motorRps() > 0.0);
        assertTrue(solution.angleDeg() >= Constants.Hood.MIN_ANGLE_DEG);
        assertTrue(solution.angleDeg() <= Constants.Hood.MAX_ANGLE_DEG);
    }

    @Test
    void solveAngleWithinHoodLimits() {
        for (double d = 1.0; d <= 6.0; d += 0.5) {
            ShotSolver.Solution solution = ShotSolver.solve(d);
            assertTrue(solution.angleDeg() >= Constants.Hood.MIN_ANGLE_DEG,
                    "Angle below min at d=" + d);
            assertTrue(solution.angleDeg() <= Constants.Hood.MAX_ANGLE_DEG,
                    "Angle above max at d=" + d);
        }
    }

    @Test
    void solveRpsWithinMotorLimitsForMidrange() {
        ShotSolver.Solution solution = ShotSolver.solve(3.0);
        assertTrue(solution.feasible());
        assertTrue(solution.motorRps() <= Constants.Shooter.MAX_SHOT_RPS,
                "Motor RPS should be within mechanism limits: " + solution.motorRps());
    }

    @Test
    void solveReturnsFallbackForInvalidDistance() {
        ShotSolver.Solution neg = ShotSolver.solve(-1.0);
        assertFalse(neg.feasible());

        ShotSolver.Solution zero = ShotSolver.solve(0.0);
        assertFalse(zero.feasible());

        ShotSolver.Solution nan = ShotSolver.solve(Double.NaN);
        assertFalse(nan.feasible());
    }

    @Test
    void solveFarShotsNeedMoreRpsThanClose() {
        ShotSolver.Solution close = ShotSolver.solve(2.0);
        ShotSolver.Solution far = ShotSolver.solve(5.0);
        assertTrue(close.feasible() && far.feasible());
        assertTrue(far.motorRps() >= close.motorRps(),
                "Far shot RPS should be >= close shot RPS");
    }

    // ---- Projectile physics round-trip verification ----

    @Test
    void solvedTrajectoryActuallyReachesTarget() {
        // Verify the projectile motion equations: a ball launched at the
        // solved (angle, speed) should land at (distance, DELTA_H).
        double distanceM = 3.5;
        ShotSolver.Solution solution = ShotSolver.solve(distanceM);
        assertTrue(solution.feasible());

        double angleRad = Math.toRadians(solution.angleDeg());
        double v0 = solution.launchSpeedMps();

        // Time to reach horizontal distance
        double vx = v0 * Math.cos(angleRad);
        double t = distanceM / vx;

        // Height at that time: y = v0*sin(θ)*t - 0.5*g*t²
        double vy = v0 * Math.sin(angleRad);
        double y = vy * t - 0.5 * 9.81 * t * t;

        assertEquals(DELTA_H, y, 0.01,
                "Projectile should reach target height. Expected=" + DELTA_H + " got=" + y);
    }

    // ---- Speed conversion round-trip ----

    @Test
    void launchSpeedConversionRoundTrip() {
        double rps = 70.0;
        double speed = ShotSolver.motorRpsToLaunchSpeed(rps);
        double backToRps = ShotSolver.launchSpeedToMotorRps(speed);
        assertEquals(rps, backToRps, 1e-9);
    }
}
