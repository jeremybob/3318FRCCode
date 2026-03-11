package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class AlignAndShootCommandTest {

    @Test
    void realisticHubShotPitchIsConsideredFeasible() {
        assertTrue(AlignAndShootCommand.isShotPitchFeasible(32.0));
    }

    @Test
    void extremePitchStillRejected() {
        assertFalse(AlignAndShootCommand.isShotPitchFeasible(45.0));
    }

    @Test
    void alignShootMatchesAlignOnlyYawTolerance() {
        assertEquals(Constants.Vision.YAW_TOLERANCE_DEG, Constants.AlignShoot.YAW_TOLERANCE_DEG, 1e-9);
    }

    @Test
    void alignShootMatchesAlignOnlyRotationCap() {
        assertEquals(Constants.Vision.MAX_ROT_CMD, Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS, 1e-9);
    }
}
