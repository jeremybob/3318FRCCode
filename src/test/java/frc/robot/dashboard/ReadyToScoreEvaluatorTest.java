package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ReadyToScoreEvaluatorTest {

    @Test
    void notReadyWhenIntakeNotHomed() {
        ReadyToScoreResult result = ReadyToScoreEvaluator.evaluate(inputs().withIntakeHomed(false).build());
        assertFalse(result.ready());
        assertEquals("Intake not homed", result.reason());
    }

    @Test
    void notReadyWhenAlignCommandIdle() {
        ReadyToScoreResult result = ReadyToScoreEvaluator.evaluate(inputs().withAlignCommandActive(false).build());
        assertFalse(result.ready());
        assertEquals("Align command idle", result.reason());
    }

    @Test
    void notReadyWhenShooterSpinningUp() {
        ReadyToScoreResult result = ReadyToScoreEvaluator.evaluate(
                inputs().withAlignPhase("SPIN_UP").withShooterAtSpeed(false).build());
        assertFalse(result.ready());
        assertEquals("Shooter spinning up", result.reason());
    }

    @Test
    void notReadyWhenNoTarget() {
        ReadyToScoreResult result = ReadyToScoreEvaluator.evaluate(
                inputs().withHasTarget(false).withHasShootableTarget(false).build());
        assertFalse(result.ready());
        assertEquals("No vision target", result.reason());
    }

    @Test
    void notReadyWhenShotGeometryInvalid() {
        ReadyToScoreResult result = ReadyToScoreEvaluator.evaluate(
                inputs().withGeometryFeasible(false).withHasShootableTarget(false).build());
        assertFalse(result.ready());
        assertEquals("Shot geometry invalid", result.reason());
    }

    @Test
    void notReadyWhenYawOutOfTolerance() {
        ReadyToScoreResult result = ReadyToScoreEvaluator.evaluate(
                inputs().withYawDeg(5.0).withYawToleranceDeg(2.0).build());
        assertFalse(result.ready());
        assertEquals("Yaw not aligned", result.reason());
    }

    @Test
    void readyWhenAllConditionsSatisfied() {
        ReadyToScoreResult result = ReadyToScoreEvaluator.evaluate(inputs().withAlignPhase("CLEAR").build());
        assertTrue(result.ready());
        assertEquals("Ready to score", result.reason());
    }

    private static InputsBuilder inputs() {
        return new InputsBuilder();
    }

    private static final class InputsBuilder {
        private boolean intakeHomed = true;
        private boolean shooterAtSpeed = true;
        private boolean alignCommandActive = true;
        private String alignPhase = "ALIGN";
        private boolean hasTarget = true;
        private boolean geometryFeasible = true;
        private boolean hasShootableTarget = true;
        private double yawDeg = 0.5;
        private double yawToleranceDeg = 2.0;

        InputsBuilder withIntakeHomed(boolean value) {
            intakeHomed = value;
            return this;
        }

        InputsBuilder withShooterAtSpeed(boolean value) {
            shooterAtSpeed = value;
            return this;
        }

        InputsBuilder withAlignCommandActive(boolean value) {
            alignCommandActive = value;
            return this;
        }

        InputsBuilder withAlignPhase(String value) {
            alignPhase = value;
            return this;
        }

        InputsBuilder withHasTarget(boolean value) {
            hasTarget = value;
            return this;
        }

        InputsBuilder withGeometryFeasible(boolean value) {
            geometryFeasible = value;
            return this;
        }

        InputsBuilder withHasShootableTarget(boolean value) {
            hasShootableTarget = value;
            return this;
        }

        InputsBuilder withYawDeg(double value) {
            yawDeg = value;
            return this;
        }

        InputsBuilder withYawToleranceDeg(double value) {
            yawToleranceDeg = value;
            return this;
        }

        ReadyToScoreEvaluator.Inputs build() {
            return new ReadyToScoreEvaluator.Inputs(
                    intakeHomed,
                    shooterAtSpeed,
                    alignCommandActive,
                    alignPhase,
                    hasTarget,
                    geometryFeasible,
                    hasShootableTarget,
                    yawDeg,
                    yawToleranceDeg);
        }
    }
}
