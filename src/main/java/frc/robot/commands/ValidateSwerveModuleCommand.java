// ============================================================================
// FILE: src/main/java/frc/robot/commands/ValidateSwerveModuleCommand.java
//
// PURPOSE: Runs a single swerve module in a specific test mode (drive forward,
//   drive reverse, steer positive, steer negative) for bring-up validation.
//
// WHEN TO USE:
//   During pit setup, use the dashboard's swerve validation panel to select
//   a corner (FL/FR/BL/BR) and a mode, then observe that the wheel moves
//   as expected. This helps catch wiring issues, inverted motors, or bad
//   CANcoder offsets before driving the robot.
//
//   The command has a 30-second safety timeout to prevent unattended spinning.
// ============================================================================
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveValidationMode;

public class ValidateSwerveModuleCommand extends Command {

    // Safety timeout: stop the validation if nobody cancels it after 30 seconds.
    // Prevents a module from spinning indefinitely if the stop command is lost.
    private static final double SAFETY_TIMEOUT_SEC = 30.0;

    private final SwerveSubsystem swerve;
    private final SwerveCorner corner;
    private final SwerveValidationMode mode;
    private final Timer timer = new Timer();

    public ValidateSwerveModuleCommand(
            SwerveSubsystem swerve,
            SwerveCorner corner,
            SwerveValidationMode mode) {
        this.swerve = swerve;
        this.corner = corner;
        this.mode = mode;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        swerve.beginValidation(corner, mode);
    }

    @Override
    public void execute() {
        swerve.runValidationStep();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopValidation();
        if (timer.hasElapsed(SAFETY_TIMEOUT_SEC)) {
            System.out.println("[ValidateSwerve] Safety timeout reached (" + SAFETY_TIMEOUT_SEC
                    + "s). Module validation stopped automatically.");
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(SAFETY_TIMEOUT_SEC);
    }
}
