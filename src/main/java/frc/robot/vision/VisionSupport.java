package frc.robot.vision;

/**
 * Utility methods for vision result freshness checking.
 */
public final class VisionSupport {

    private VisionSupport() {}

    /** Returns true if the result is non-null, has a valid tag, and is recent. */
    public static boolean isResultFresh(
            VisionResult result,
            double nowSec,
            double freshnessThresholdSec) {
        if (result == null || result.tagId() < 0 || !Double.isFinite(nowSec)) {
            return false;
        }
        double ageSec = nowSec - result.timestampSec();
        return Double.isFinite(ageSec) && ageSec >= 0.0 && ageSec < freshnessThresholdSec;
    }
}
