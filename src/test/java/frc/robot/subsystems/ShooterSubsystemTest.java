package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class ShooterSubsystemTest {

    @Test
    void calculateTargetRpsReturnsFallbackForInvalidDistance() {
        assertEquals(Constants.Shooter.TARGET_RPS, ShooterSubsystem.calculateTargetRPS(0.0), 1e-9);
        assertEquals(Constants.Shooter.TARGET_RPS, ShooterSubsystem.calculateTargetRPS(-1.0), 1e-9);
        assertEquals(Constants.Shooter.TARGET_RPS, ShooterSubsystem.calculateTargetRPS(Double.NaN), 1e-9);
    }

    @Test
    void calculateTargetRpsReturnsPositiveForValidDistance() {
        double rps = ShooterSubsystem.calculateTargetRPS(3.0);
        assertTrue(rps > 0.0, "RPS should be positive for valid distance");
        assertTrue(rps >= Constants.Shooter.MIN_SHOT_RPS,
                "RPS should be at least MIN_SHOT_RPS");
        assertTrue(rps <= Constants.Shooter.MAX_SHOT_RPS,
                "RPS should not exceed MAX_SHOT_RPS");
    }

    @Test
    void farShotsRequireMoreSpeedThanCloserShots() {
        double closeRps = ShooterSubsystem.calculateTargetRPS(2.0);
        double farRps = ShooterSubsystem.calculateTargetRPS(5.0);
        assertTrue(farRps >= closeRps,
                "Farther shots should require equal or greater RPS");
    }

    @Test
    void stationaryMovingShotSolutionMatchesStationary() {
        double distanceM = 3.0;
        ShooterSubsystem.ShotSolution solution = ShooterSubsystem.calculateMovingShotSolution(
                distanceM, 0.0, 0.0);

        assertTrue(solution.feasible());
        assertEquals(
                ShooterSubsystem.calculateTargetRPS(distanceM),
                solution.targetRps(),
                0.5);
    }

    @Test
    void manualStickDeadbandStopsShooter() {
        double targetRps = ShooterSubsystem.manualStickToTargetRps(
                Constants.Shooter.MANUAL_SPEED_DEADBAND - 0.01);

        assertEquals(0.0, targetRps, 1e-9);
    }

    @Test
    void manualStickOnlyCommandsForwardShooterSpeed() {
        double reverseTargetRps = ShooterSubsystem.manualStickToTargetRps(-1.0);
        double forwardTargetRps = ShooterSubsystem.manualStickToTargetRps(1.0);

        assertEquals(0.0, reverseTargetRps, 1e-9);
        assertEquals(Constants.Shooter.MANUAL_MAX_RPS, forwardTargetRps, 1e-9);
    }
}
