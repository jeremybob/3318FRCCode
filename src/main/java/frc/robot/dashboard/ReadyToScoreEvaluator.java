package frc.robot.dashboard;

public final class ReadyToScoreEvaluator {

    private ReadyToScoreEvaluator() {}

    public static ReadyToScoreResult evaluate(Inputs inputs) {
        if (!inputs.intakeHomed()) {
            return new ReadyToScoreResult(false, "Intake not homed");
        }

        if (!inputs.alignCommandActive()) {
            return new ReadyToScoreResult(false, "Align command idle");
        }

        String phase = inputs.alignPhase();
        if ("SPIN_UP".equals(phase) && !inputs.shooterAtSpeed()) {
            return new ReadyToScoreResult(false, "Shooter spinning up");
        }

        if (!inputs.hasShootableTarget()) {
            if (!inputs.hasTarget()) {
                return new ReadyToScoreResult(false, "No vision target");
            }
            if (!inputs.geometryFeasible()) {
                return new ReadyToScoreResult(false, "Shot geometry invalid");
            }
            return new ReadyToScoreResult(false, "Target lost");
        }

        if (!Double.isFinite(inputs.yawDeg())) {
            return new ReadyToScoreResult(false, "Yaw unavailable");
        }

        if (Math.abs(inputs.yawDeg()) > inputs.yawToleranceDeg()) {
            return new ReadyToScoreResult(false, "Yaw not aligned");
        }

        if ("DONE".equals(phase) || "IDLE".equals(phase)) {
            return new ReadyToScoreResult(false, "Shot cycle complete");
        }

        return new ReadyToScoreResult(true, "Ready to score");
    }

    public record Inputs(
            boolean intakeHomed,
            boolean shooterAtSpeed,
            boolean alignCommandActive,
            String alignPhase,
            boolean hasTarget,
            boolean geometryFeasible,
            boolean hasShootableTarget,
            double yawDeg,
            double yawToleranceDeg) {
    }
}
