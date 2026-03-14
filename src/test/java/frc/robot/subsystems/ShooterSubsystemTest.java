package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class ShooterSubsystemTest {

    @Test
    void empiricalShotCurveMatchesMeasuredCloseShot() {
        double targetRps = ShooterSubsystem.calculateTargetRPS(
                Constants.Shooter.MEASURED_CLOSE_SHOT_DISTANCE_M);

        assertEquals(Constants.Shooter.FALLBACK_RPS, targetRps, 1e-9);
    }

    @Test
    void empiricalShotCurvePreservesMidrangeReferenceTarget() {
        double targetRps = ShooterSubsystem.calculateTargetRPS(
                Constants.Shooter.MIDRANGE_REFERENCE_DISTANCE_M);

        assertEquals(Constants.Shooter.MIDRANGE_REFERENCE_RPS, targetRps, 1e-9);
    }

    @Test
    void stationaryMovingShotSolutionUsesEmpiricalTarget() {
        ShooterSubsystem.ShotSolution solution = ShooterSubsystem.calculateMovingShotSolution(
                Constants.Shooter.MIDRANGE_REFERENCE_DISTANCE_M,
                0.0,
                0.0);

        assertTrue(solution.feasible());
        assertEquals(
                ShooterSubsystem.calculateTargetRPS(Constants.Shooter.MIDRANGE_REFERENCE_DISTANCE_M),
                solution.targetRps(),
                1e-9);
    }

    @Test
    void closeShotNeverDropsBelowGlobalMinimum() {
        double targetRps = ShooterSubsystem.calculateTargetRPS(0.5);

        assertEquals(Constants.Shooter.MIN_SHOT_RPS, targetRps, 1e-9);
    }

    @Test
    void shotCurveDoesNotGetSlowerAsDistanceIncreases() {
        double closeRps = ShooterSubsystem.calculateTargetRPS(Constants.Shooter.MEASURED_CLOSE_SHOT_DISTANCE_M);
        double farRps = ShooterSubsystem.calculateTargetRPS(Constants.Shooter.MIDRANGE_REFERENCE_DISTANCE_M);

        assertTrue(farRps >= closeRps);
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
