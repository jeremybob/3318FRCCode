package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveValidationMode;

public class ValidateSwerveModuleCommand extends Command {

    private final SwerveSubsystem swerve;
    private final SwerveCorner corner;
    private final SwerveValidationMode mode;

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
        swerve.beginValidation(corner, mode);
    }

    @Override
    public void execute() {
        swerve.runValidationStep();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopValidation();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
