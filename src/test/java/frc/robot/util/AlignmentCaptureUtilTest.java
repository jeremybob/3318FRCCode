package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class AlignmentCaptureUtilTest {

    @Test
    void capturesWhenAimErrorEntersToleranceBand() {
        assertTrue(AlignmentCaptureUtil.shouldCaptureOnEntryOrCrossing(
                4.2,
                3.2,
                Constants.AlignShoot.YAW_TOLERANCE_DEG,
                Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG,
                Constants.AlignShoot.CAPTURE_OVERSHOOT_DEG));
    }

    @Test
    void capturesWhenAimErrorJustCrossesPastTarget() {
        assertTrue(AlignmentCaptureUtil.shouldCaptureOnEntryOrCrossing(
                1.3,
                -0.7,
                Constants.AlignShoot.YAW_TOLERANCE_DEG,
                Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG,
                Constants.AlignShoot.CAPTURE_OVERSHOOT_DEG));
    }

    @Test
    void doesNotCaptureWhenPreviousSampleWasStillFarAway() {
        assertFalse(AlignmentCaptureUtil.shouldCaptureOnEntryOrCrossing(
                10.0,
                3.0,
                Constants.AlignShoot.YAW_TOLERANCE_DEG,
                Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG,
                Constants.AlignShoot.CAPTURE_OVERSHOOT_DEG));
    }

    @Test
    void doesNotCaptureWhenCrossingOvershootsTooFar() {
        assertFalse(AlignmentCaptureUtil.shouldCaptureOnEntryOrCrossing(
                1.4,
                -2.6,
                Constants.AlignShoot.YAW_TOLERANCE_DEG,
                Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG,
                Constants.AlignShoot.CAPTURE_OVERSHOOT_DEG));
    }

    @Test
    void doesNotCaptureWhenErrorIsGrowing() {
        assertFalse(AlignmentCaptureUtil.shouldCaptureOnEntryOrCrossing(
                3.3,
                3.8,
                Constants.AlignShoot.YAW_TOLERANCE_DEG,
                Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG,
                Constants.AlignShoot.CAPTURE_OVERSHOOT_DEG));
    }
}
