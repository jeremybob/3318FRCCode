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

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerCommand extends Command {

    private final IntakeSubsystem intake;
    private final DoubleSupplier forwardPowerSupplier;
    private final IntakeRollerProtection protection = new IntakeRollerProtection();

    // Timer tracks how long the current has been above the stall threshold
    private final Timer stallTimer = new Timer();
    // Timer tracks how long we've been reversing
    private final Timer reverseTimer = new Timer();
    // Timer tracks how long we've been in lockout before auto-resetting
    private final Timer lockoutTimer = new Timer();

    // Pause duration before auto-resetting from lockout (seconds).
    // Gives the mechanism time to settle before retrying.
    private static final double LOCKOUT_COOLDOWN_SEC = 1.0;

    // --------------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   intake       - the intake subsystem (roller motor + current sensing)
    //   forwardPower - fixed roller power during normal operation (0.0 to 1.0)
    // --------------------------------------------------------------------------
    public IntakeRollerCommand(IntakeSubsystem intake, double forwardPower) {
        this(intake, () -> forwardPower);
    }

    // --------------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   intake              - the intake subsystem (roller motor + current sensing)
    //   forwardPowerSupplier - supplies desired forward roller power (0.0 to 1.0)
    // --------------------------------------------------------------------------
    public IntakeRollerCommand(IntakeSubsystem intake, DoubleSupplier forwardPowerSupplier) {
        this.intake = intake;
        this.forwardPowerSupplier = forwardPowerSupplier;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        protection.reset();
        restartTimer(stallTimer);
        reverseTimer.stop();
        reverseTimer.reset();
        intake.setRollerPower(getForwardPowerCommand());
    }

    @Override
    public void execute() {
        double current = intake.getRollerCurrentAmps();
        IntakeRollerProtection.Update update = protection.update(
                current,
                Constants.Intake.STALL_CURRENT_THRESHOLD_A,
                stallTimer.hasElapsed(Constants.Intake.STALL_TIME_SEC),
                reverseTimer.hasElapsed(Constants.Intake.STALL_REVERSE_TIME_SEC),
                Constants.Intake.STALL_MAX_RETRIES);

        if (update.restartStallTimer()) {
            restartTimer(stallTimer);
        }
        if (update.restartReverseTimer()) {
            restartTimer(reverseTimer);
        }
        if (update.enteredReversing()) {
            System.out.println("[IntakeRoller] Stall detected — reversing "
                    + "(attempt " + protection.retryCount() + "/" + Constants.Intake.STALL_MAX_RETRIES + ")");
        }
        if (update.enteredLockout()) {
            System.out.println("[IntakeRoller] Max retries (" + Constants.Intake.STALL_MAX_RETRIES
                    + ") reached. Pausing " + LOCKOUT_COOLDOWN_SEC + "s before auto-reset.");
            restartTimer(lockoutTimer);
        }

        // Auto-reset from lockout after a cooldown period so the operator
        // doesn't have to release and re-press the trigger.
        if (protection.isLockedOut() && lockoutTimer.hasElapsed(LOCKOUT_COOLDOWN_SEC)) {
            System.out.println("[IntakeRoller] Auto-resetting after lockout cooldown.");
            protection.reset();
            restartTimer(stallTimer);
        }

        double forwardPowerCommand = getForwardPowerCommand();
        intake.setRollerPower(protection.commandedPower(
                forwardPowerCommand,
                Constants.Intake.STALL_REVERSE_POWER));

        SmartDashboard.putString("Intake/RollerState", protection.state().name());
        SmartDashboard.putNumber("Intake/StallRetries", protection.retryCount());
        SmartDashboard.putNumber("Intake/RollerForwardCommand", forwardPowerCommand);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerPower(0);
    }

    @Override
    public boolean isFinished() {
        // Never auto-finish — command runs until interrupted (button released).
        // Lockout is handled by pausing and auto-resetting above.
        return false;
    }

    private static void restartTimer(Timer timer) {
        timer.reset();
        timer.start();
    }

    private double getForwardPowerCommand() {
        return MathUtil.clamp(forwardPowerSupplier.getAsDouble(), 0.0, 1.0);
    }
}
