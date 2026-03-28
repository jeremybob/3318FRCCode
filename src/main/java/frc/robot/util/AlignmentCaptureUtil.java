// ============================================================================
// FILE: src/main/java/frc/robot/util/AlignmentCaptureUtil.java
//
// PURPOSE: Decides the exact moment to "capture" alignment and declare the
//   robot is aimed at the target. Used by AlignAndShootCommand and
//   AlignOnlyCommand.
//
// WHY NOT JUST USE A THRESHOLD?
//   A simple "error < tolerance" check has two problems:
//   1. It can latch too early while the robot is swinging through quickly.
//   2. It can latch on noise spikes (brief jumps below threshold).
//
//   This utility additionally checks that:
//   - The error is DECREASING (converging, not overshooting outward)
//   - The PREVIOUS error was already close (we didn't jump from far away)
//   - OR the error just crossed zero (passed through the target)
//
// This prevents false captures and makes shooting more reliable.
// ============================================================================
package frc.robot.util;

public final class AlignmentCaptureUtil {

    private AlignmentCaptureUtil() {}

    /**
     * Latch alignment once the observed aim error enters the tolerance band or
     * just crosses past the target. The previous sample must already be close
     * enough that this looks like a real pass-through, not a target jump.
     */
    public static boolean shouldCaptureOnEntryOrCrossing(
            double previousAimErrorDeg,
            double currentAimErrorDeg,
            double yawToleranceDeg,
            double yawBreakToleranceDeg,
            double captureOvershootDeg) {
        if (!Double.isFinite(previousAimErrorDeg) || !Double.isFinite(currentAimErrorDeg)) {
            return false;
        }

        double previousAbsErrorDeg = Math.abs(previousAimErrorDeg);
        double currentAbsErrorDeg = Math.abs(currentAimErrorDeg);
        if (currentAbsErrorDeg > previousAbsErrorDeg) {
            return false;
        }

        boolean wasNearTarget = previousAbsErrorDeg <= yawBreakToleranceDeg;
        if (!wasNearTarget) {
            return false;
        }

        boolean enteredTolerance = previousAbsErrorDeg > yawToleranceDeg
                && currentAbsErrorDeg <= yawToleranceDeg;
        boolean crossedTarget = Math.signum(previousAimErrorDeg) != 0.0
                && Math.signum(currentAimErrorDeg) != 0.0
                && Math.signum(previousAimErrorDeg) != Math.signum(currentAimErrorDeg)
                && currentAbsErrorDeg <= captureOvershootDeg;

        return enteredTolerance || crossedTarget;
    }
}
