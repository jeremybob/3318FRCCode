// ============================================================================
// FILE: src/main/java/frc/robot/vision/VisionResult.java
//
// PURPOSE: Immutable snapshot of vision data published by RioVisionThread.
//   Contains the best HUB tag detection from the most recent camera frame.
//   Read by AlignAndShootCommand via AtomicReference (thread-safe, lock-free).
// ============================================================================
package frc.robot.vision;

/**
 * Immutable result from the background vision thread.
 *
 * @param tagId           Fiducial ID of the best HUB tag, or -1 if none
 * @param yawDeg          Horizontal angle to tag center (positive = tag right of center)
 * @param pitchDeg        Vertical angle to tag center (positive = tag above center)
 * @param tagPixelHeight  Tag bounding height in pixels (distance proxy)
 * @param timestampSec    FPGA timestamp when the frame was captured
 */
public record VisionResult(
        int tagId,
        double yawDeg,
        double pitchDeg,
        double tagPixelHeight,
        double timestampSec) {

    /** Estimate distance using the pinhole camera model. */
    public double estimateDistanceM(double realTagHeightM, double focalLengthPixels) {
        if (tagPixelHeight <= 0) return Double.NaN;
        return (realTagHeightM * focalLengthPixels) / tagPixelHeight;
    }
}
