package frc.robot.commands;

final class IntakeRollerProtection {

    enum State { RUNNING, REVERSING, LOCKED_OUT }

    record Update(
            boolean restartStallTimer,
            boolean restartReverseTimer,
            boolean enteredReversing,
            boolean enteredLockout) {
        static final Update NONE = new Update(false, false, false, false);
    }

    private State state = State.RUNNING;
    private int retryCount = 0;

    void reset() {
        state = State.RUNNING;
        retryCount = 0;
    }

    Update update(
            double currentAmps,
            double stallThresholdAmps,
            boolean stallElapsed,
            boolean reverseElapsed,
            int maxRetries) {
        switch (state) {
            case RUNNING:
                if (currentAmps <= stallThresholdAmps) {
                    return new Update(true, false, false, false);
                }
                if (!stallElapsed) {
                    return Update.NONE;
                }
                if (retryCount >= maxRetries) {
                    state = State.LOCKED_OUT;
                    return new Update(false, false, false, true);
                }
                retryCount++;
                state = State.REVERSING;
                return new Update(false, true, true, false);

            case REVERSING:
                if (!reverseElapsed) {
                    return Update.NONE;
                }
                state = State.RUNNING;
                return new Update(true, false, false, false);

            case LOCKED_OUT:
                return Update.NONE;

            default:
                throw new IllegalStateException("Unhandled state " + state);
        }
    }

    State state() {
        return state;
    }

    int retryCount() {
        return retryCount;
    }

    boolean isLockedOut() {
        return state == State.LOCKED_OUT;
    }

    double commandedPower(double forwardPower, double reversePower) {
        return switch (state) {
            case RUNNING -> forwardPower;
            case REVERSING -> reversePower;
            case LOCKED_OUT -> 0.0;
        };
    }
}
