// ============================================================================
// FILE: src/main/java/frc/robot/commands/IntakeRollerCommand.java
//
// PURPOSE: Runs the intake roller with automatic stall detection and recovery.
//
//   Normal intake commands just spin the roller at a fixed power and hope for
//   the best. If a game piece jams, the motor stalls, draws excessive current,
//   and can trip a breaker or burn out the motor.
//
//   This command monitors the roller motor current. If it exceeds a threshold
//   for a sustained period (indicating a jam), the roller automatically reverses
//   briefly to clear the obstruction, then retries forward — up to a configurable
//   number of times before giving up.
//
// STATE MACHINE:
//   RUNNING   → roller spinning forward, monitoring current
//   REVERSING → stall detected, briefly running backward to clear jam
//
// USAGE:
//   - Bind to operator left trigger (replaces simple setRollerPower)
//   - Used inside buildIntakeGamePieceCommand() for auto intake
//   - Runs until interrupted (button released / command canceled)
// ============================================================================
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerCommand extends Command {

    private final IntakeSubsystem intake;
    private final double forwardPower;

    // Timer tracks how long the current has been above the stall threshold
    private final Timer stallTimer = new Timer();
    // Timer tracks how long we've been reversing
    private final Timer reverseTimer = new Timer();

    private int retryCount;

    private enum State { RUNNING, REVERSING }
    private State state;

    // --------------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   intake       - the intake subsystem (roller motor + current sensing)
    //   forwardPower - roller power during normal operation (0.0 to 1.0)
    // --------------------------------------------------------------------------
    public IntakeRollerCommand(IntakeSubsystem intake, double forwardPower) {
        this.intake = intake;
        this.forwardPower = forwardPower;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        state = State.RUNNING;
        retryCount = 0;
        stallTimer.reset();
        stallTimer.start();
        reverseTimer.reset();
        intake.setRollerPower(forwardPower);
    }

    @Override
    public void execute() {
        switch (state) {

            case RUNNING: {
                intake.setRollerPower(forwardPower);

                double current = intake.getRollerCurrentAmps();
                if (current > Constants.Intake.STALL_CURRENT_THRESHOLD_A) {
                    // Current is high — check if it's been sustained
                    if (stallTimer.hasElapsed(Constants.Intake.STALL_TIME_SEC)) {
                        if (retryCount >= Constants.Intake.STALL_MAX_RETRIES) {
                            // Max retries reached; keep running but stop reversing.
                            // The operator can release and re-press to reset.
                            System.out.println("[IntakeRoller] Max retries (" +
                                    Constants.Intake.STALL_MAX_RETRIES + ") reached.");
                        } else {
                            retryCount++;
                            System.out.println("[IntakeRoller] Stall detected — reversing " +
                                    "(attempt " + retryCount + "/" + Constants.Intake.STALL_MAX_RETRIES + ")");
                            state = State.REVERSING;
                            reverseTimer.reset();
                            reverseTimer.start();
                        }
                    }
                } else {
                    // Current is normal — reset the stall timer
                    stallTimer.reset();
                    stallTimer.start();
                }
                break;
            }

            case REVERSING: {
                intake.setRollerPower(Constants.Intake.STALL_REVERSE_POWER);

                if (reverseTimer.hasElapsed(Constants.Intake.STALL_REVERSE_TIME_SEC)) {
                    // Done reversing — try forward again
                    state = State.RUNNING;
                    stallTimer.reset();
                    stallTimer.start();
                }
                break;
            }
        }

        SmartDashboard.putString("Intake/RollerState", state.name());
        SmartDashboard.putNumber("Intake/StallRetries", retryCount);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerPower(0);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted (button released)
    }
}
