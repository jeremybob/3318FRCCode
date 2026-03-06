package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AlignAndShootCommandTest {

    @Test
    void realisticHubShotPitchIsConsideredFeasible() {
        assertTrue(AlignAndShootCommand.isShotPitchFeasible(32.0));
    }

    @Test
    void extremePitchStillRejected() {
        assertFalse(AlignAndShootCommand.isShotPitchFeasible(45.0));
    }
}
