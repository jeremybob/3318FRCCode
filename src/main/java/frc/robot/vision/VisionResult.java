// ============================================================================
// FILE: src/main/java/frc/robot/vision/VisionResult.java
//
// PURPOSE: Immutable snapshot of vision data published by RioVisionThread.
//   Contains the estimated HUB center aim point from the most recent camera
//   frame, plus the dominant tag used for range/debug.
//   Read by AlignAndShootCommand via AtomicReference (thread-safe, lock-free).
// ============================================================================
package frc.robot.vision;

/**
 * Immutable result from the background vision thread.
 *
 * @param tagId             Fiducial ID of the dominant HUB tag, or -1 if none
 * @param yawDeg            Horizontal angle to estimated HUB center
 * @param pitchDeg          Vertical angle to estimated HUB center
 * @param tagPixelHeight    Dominant tag bounding height in pixels (distance proxy)
 * @param timestampSec      FPGA timestamp when the frame was captured
 * @param bestTagYawDeg     Horizontal angle to the dominant tag center
 * @param bestTagPitchDeg   Vertical angle to the dominant tag center
 * @param hubTagCount       Number of hub-tag detections used for this estimate
 * @param hubFaceCount      Number of distinct hub faces represented in-frame
 * @param hubSpanPx         Horizontal spread of visible hub-tag centers in pixels
 */
public record VisionResult(
        int tagId,
        double yawDeg,
        double pitchDeg,
        double tagPixelHeight,
        double timestampSec,
        double bestTagYawDeg,
        double bestTagPitchDeg,
        int hubTagCount,
        int hubFaceCount,
        double hubSpanPx) {

    /** Estimate distance using the pinhole camera model. */
    public double estimateDistanceM(double realTagHeightM, double focalLengthPixels) {
        if (tagPixelHeight <= 0) return Double.NaN;
        return (realTagHeightM * focalLengthPixels) / tagPixelHeight;
    }
}
