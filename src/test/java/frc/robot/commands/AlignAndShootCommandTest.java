package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class AlignAndShootCommandTest {

    @Test
    void midrangeDistanceIsConsideredFeasible() {
        double midDistance =
                (Constants.Vision.MIN_SHOT_DISTANCE_M + Constants.Vision.MAX_SHOT_DISTANCE_M) * 0.5;
        assertTrue(AlignAndShootCommand.isShotDistanceFeasible(midDistance));
    }

    @Test
    void extremeDistanceStillRejected() {
        assertFalse(AlignAndShootCommand.isShotDistanceFeasible(
                Constants.Vision.MAX_SHOT_DISTANCE_M + 1.0));
    }
}
