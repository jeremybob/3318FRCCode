package frc.robot.vision;

/**
 * Immutable snapshot of vision data from PhotonVision on the Raspberry Pi 4.
 * Read by alignment commands via AtomicReference (thread-safe, lock-free).
 *
 * @param tagId        Fiducial ID of the best HUB tag, or -1 if none
 * @param yawDeg       Horizontal angle to the target (from PhotonVision)
 * @param pitchDeg     Vertical angle to the target (from PhotonVision)
 * @param distanceM    Horizontal distance to the target from pose estimation
 * @param timestampSec FPGA timestamp when the result was received
 * @param hubTagCount  Number of alliance HUB tags visible this frame
 */
public record VisionResult(
        int tagId,
        double yawDeg,
        double pitchDeg,
        double distanceM,
        double timestampSec,
        int hubTagCount) {
}
