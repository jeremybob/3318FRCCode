package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class VisionSupportTest {

    @Test
    void freshnessCheckUsesTimestampAge() {
        VisionResult freshResult = new VisionResult(18, 0.0, 0.0, 2.5, 9.7, 1);
        VisionResult staleResult = new VisionResult(18, 0.0, 0.0, 2.5, 8.9, 1);

        assertTrue(VisionSupport.isResultFresh(freshResult, 10.0, 0.5));
        assertFalse(VisionSupport.isResultFresh(staleResult, 10.0, 0.5));
        assertFalse(VisionSupport.isResultFresh(null, 10.0, 0.5));
    }

    @Test
    void freshnessCheckRejectsInvalidTagId() {
        VisionResult noTag = new VisionResult(-1, 0.0, 0.0, 2.5, 9.7, 0);
        assertFalse(VisionSupport.isResultFresh(noTag, 10.0, 0.5));
    }

    @Test
    void freshnessCheckRejectsNaNTimestamp() {
        VisionResult result = new VisionResult(18, 0.0, 0.0, 2.5, 9.7, 1);
        assertFalse(VisionSupport.isResultFresh(result, Double.NaN, 0.5));
    }
}
