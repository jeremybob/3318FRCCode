// ============================================================================
// FILE: src/main/java/frc/robot/commands/IntakeRollerProtection.java
//
// PURPOSE: State machine that protects the intake roller motor from stalling.
//   When a game piece jams, the motor draws excessive current ("stalls").
//   If left unchecked, this can trip a breaker or burn out the motor.
//
// STATE MACHINE:
//   RUNNING    → Normal operation. If motor current stays above the stall
//                threshold for too long, transition to REVERSING.
//   REVERSING  → Roller runs backward briefly to clear the jam.
//                After the reverse timer elapses, go back to RUNNING.
//   LOCKED_OUT → Too many consecutive stall/reverse cycles. Motor is stopped
//                to prevent damage. IntakeRollerCommand auto-resets after
//                a cooldown period.
//
// This class is intentionally separated from IntakeRollerCommand so it can
// be unit-tested without needing real hardware or WPILib subsystems.
// ============================================================================
package frc.robot.commands;

final class IntakeRollerProtection {

    // The three possible states of the protection system
    enum State { RUNNING, REVERSING, LOCKED_OUT }

    // Signals back to IntakeRollerCommand about what timers/actions to take.
    // Using a record avoids the caller needing to inspect internal state.
    record Update(
            boolean restartStallTimer,
            boolean restartReverseTimer,
            boolean enteredReversing,
            boolean enteredLockout) {
        static final Update NONE = new Update(false, false, false, false);
    }

    private State state = State.RUNNING;
    private int retryCount = 0;

    /** Reset everything back to normal — called on command start and after lockout cooldown. */
    void reset() {
        state = State.RUNNING;
        retryCount = 0;
    }

    /**
     * Called every 20ms with the latest motor current reading and timer states.
     * Returns an Update telling the caller what timers to restart.
     */
    Update update(
            double currentAmps,
            double stallThresholdAmps,
            boolean stallElapsed,
            boolean reverseElapsed,
            int maxRetries) {
        switch (state) {
            case RUNNING:
                // Current is normal — reset the stall timer so it starts fresh
                if (currentAmps <= stallThresholdAmps) {
                    return new Update(true, false, false, false);
                }
                // Current is high but hasn't been high long enough yet
                if (!stallElapsed) {
                    return Update.NONE;
                }
                // Stall confirmed! Check if we've exhausted our retries.
                if (retryCount >= maxRetries) {
                    state = State.LOCKED_OUT;
                    return new Update(false, false, false, true);
                }
                // Start a reverse cycle to try clearing the jam
                retryCount++;
                state = State.REVERSING;
                return new Update(false, true, true, false);

            case REVERSING:
                // Keep reversing until the timer runs out
                if (!reverseElapsed) {
                    return Update.NONE;
                }
                // Done reversing — try running forward again
                state = State.RUNNING;
                return new Update(true, false, false, false);

            case LOCKED_OUT:
                // Do nothing — waiting for external reset
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
