// ============================================================================
// FILE: src/main/java/frc/robot/commands/IntakeHomeCommand.java
//
// PURPOSE: Finds the intake linear slide's "home" (retracted) position at startup.
//
// WHY WE NEED THIS:
//   The slide motor uses a RELATIVE encoder. When the robot powers off, the
//   encoder loses its position. When it powers back on, the encoder starts at 0
//   regardless of where the slide actually is.
//
//   This command solves that by slowly driving the slide toward the Hall effect
//   sensor. Once the sensor triggers, we know exactly where the slide is and we
//   zero the encoder. After that, commands like setSlidePosition(6.0) work correctly.
//
// WHEN TO RUN:
//   - Automatically at robot startup (RobotContainer binds it to robotInit)
//   - Also available on the X button in case homing is lost during a match
// ============================================================================
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHomeCommand extends Command {

    private final IntakeSubsystem intake;

    // Timer is used as a safety fallback: if the slide doesn't hit the sensor
    // within HOME_TIMEOUT_SEC, we stop trying to prevent mechanical damage.
    private final Timer timer = new Timer();

    // Flag: did we successfully find the home position?
    private boolean homed = false;

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public IntakeHomeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    // --------------------------------------------------------------------------
    // initialize()
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
    // --------------------------------------------------------------------------
    @Override
    public void execute() {
        if (intake.getLimitSwitchPressed()) {
            // Hall effect sensor triggered! The slide is now at the home position.
            intake.setSlidePowerHoming(0);   // stop moving
            intake.resetEncoderToHome();     // set this position as 0 inches
            homed = true;
        } else {
            // Keep driving slowly toward the Hall effect sensor
            intake.setSlidePowerHoming(Constants.Intake.HOME_POWER);
        }
    }

    // --------------------------------------------------------------------------
    // isFinished()
    // --------------------------------------------------------------------------
    @Override
    public boolean isFinished() {
        return homed || timer.hasElapsed(Constants.Intake.HOME_TIMEOUT_SEC);
    }

    // --------------------------------------------------------------------------
    // end()
    // --------------------------------------------------------------------------
    @Override
    public void end(boolean interrupted) {
        intake.setSlidePowerHoming(0);

        if (homed) {
            System.out.println("[IntakeHomeCommand] Homing complete.");
        } else if (interrupted) {
            System.out.println("[IntakeHomeCommand] Homing was interrupted!");
        } else {
            System.out.println("[IntakeHomeCommand] HOMING FAILED (timeout). Check Hall effect sensor wiring!");
            if (!intake.isHomed() && intake.getLimitSwitchPressed()) {
                intake.resetEncoderToHome();
            }
        }
    }
}
