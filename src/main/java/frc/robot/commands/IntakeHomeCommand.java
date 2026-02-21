// ============================================================================
// FILE: src/main/java/frc/robot/commands/IntakeHomeCommand.java
//
// PURPOSE: Finds the intake arm's "home" (raised) position at startup.
//
// WHY WE NEED THIS:
//   The tilt motor uses a RELATIVE encoder. When the robot powers off, the
//   encoder loses its position. When it powers back on, the encoder starts at 0
//   regardless of where the arm actually is.
//
//   This command solves that by slowly driving the arm toward the limit switch.
//   Once the switch triggers, we know exactly where the arm is and we zero the
//   encoder. After that, commands like setTiltPosition(45°) work correctly.
//
// WHEN TO RUN:
//   - Automatically at robot startup (RobotContainer binds it to robotInit)
//   - Also available on the X button in case homing is lost during a match
//
// UPDATED: Extends Command (not CommandBase — that was deprecated in 2024)
// ============================================================================
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;  // ← use Command, not CommandBase

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHomeCommand extends Command {

    private final IntakeSubsystem intake;

    // Timer is used as a safety fallback: if the arm doesn't hit the switch
    // within HOME_TIMEOUT_SEC, we stop trying to prevent mechanical damage.
    private final Timer timer = new Timer();

    // Flag: did we successfully find the home position?
    private boolean homed = false;

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public IntakeHomeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        // addRequirements tells the scheduler that while this command is running,
        // no other command may use the intake subsystem.
        addRequirements(intake);
    }

    // --------------------------------------------------------------------------
    // initialize()
    //
    // Called once when the command first starts.
    // Reset state and start moving the arm toward home.
    // --------------------------------------------------------------------------
    @Override
    public void initialize() {
        homed = false;
        timer.reset();
        timer.start();
        System.out.println("[IntakeHomeCommand] Starting homing sequence...");
    }

    // --------------------------------------------------------------------------
    // execute()
    //
    // Called every 20ms while the command is running.
    // Either the switch triggers (success) or we keep moving toward it.
    // --------------------------------------------------------------------------
    @Override
    public void execute() {
        if (intake.getLimitSwitchPressed()) {
            // Limit switch triggered! The arm is now at the home position.
            intake.setTiltPower(0);         // stop moving
            intake.resetEncoderToHome();    // set this position as 0 degrees
            homed = true;
        } else {
            // Keep driving slowly toward the limit switch
            intake.setTiltPower(Constants.Intake.HOME_POWER);
        }
    }

    // --------------------------------------------------------------------------
    // isFinished()
    //
    // The command ends when:
    //   a) We found home (homed == true), OR
    //   b) The timeout expired (arm didn't reach switch in time)
    // --------------------------------------------------------------------------
    @Override
    public boolean isFinished() {
        return homed || timer.hasElapsed(Constants.Intake.HOME_TIMEOUT_SEC);
    }

    // --------------------------------------------------------------------------
    // end()
    //
    // Called once when the command ends, either normally or interrupted.
    // Always stops the motor for safety.
    // --------------------------------------------------------------------------
    @Override
    public void end(boolean interrupted) {
        intake.setTiltPower(0);

        if (homed) {
            System.out.println("[IntakeHomeCommand] Homing complete.");
        } else if (interrupted) {
            System.out.println("[IntakeHomeCommand] Homing was interrupted!");
        } else {
            // Timeout expired without hitting the switch
            System.out.println("[IntakeHomeCommand] HOMING FAILED (timeout). Check limit switch wiring!");
            // Last-chance check: if the switch IS pressed right now even though
            // we never caught it in execute() (shouldn't happen, but just in case)
            if (!intake.isHomed() && intake.getLimitSwitchPressed()) {
                intake.resetEncoderToHome();
            }
        }
    }
}
