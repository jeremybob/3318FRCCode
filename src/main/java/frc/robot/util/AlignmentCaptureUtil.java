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
