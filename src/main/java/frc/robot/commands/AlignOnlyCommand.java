// ============================================================================
// FILE: src/main/java/frc/robot/commands/AlignOnlyCommand.java
//
// PURPOSE: Rotates the robot to face the alliance HUB using pose-based heading,
//   WITHOUT running any shooter, feeder, or intake motors. This is a testing
//   command for alignment bring-up — use it to verify that the robot can lock
//   onto a target before trying the full AlignAndShootCommand.
//
//   The command finishes when alignment is stable or when the convergence
//   timeout expires. It does NOT fire.
// ============================================================================
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AlignmentCaptureUtil;


/**
 * Rotates the robot to face the alliance HUB using pose-based heading without
 * running any shooter/feed/intake motors. Intended for alignment bring-up.
 */
public class AlignOnlyCommand extends Command {
    private static final double NO_TARGET_TIMEOUT_SEC = 3.0;
    private static final double ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0;
    private static final double STABLE_ALIGNMENT_TIME_SEC = Constants.AlignShoot.SETTLE_TIME_SEC;

    private final SwerveSubsystem swerve;
    private final PIDController turnPID = new PIDController(
            Constants.AlignShoot.TURN_kP,
            0.0,
            Constants.AlignShoot.TURN_kD);

    private final Timer noTargetTimer = new Timer();
    private final Timer alignTimer = new Timer();
    private final Timer alignedHoldTimer = new Timer();
    private double filteredHeadingErrorDeg = Double.NaN;
    private double previousAimErrorDeg = Double.NaN;

    private boolean finished;
    private String finishReason = "";

    public AlignOnlyCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
        turnPID.setTolerance(Constants.AlignShoot.YAW_TOLERANCE_DEG);
    }

    @Override
    public void initialize() {
        finished = false;
        finishReason = "";

        noTargetTimer.restart();
        alignTimer.restart();
        alignedHoldTimer.stop();
        alignedHoldTimer.reset();
        filteredHeadingErrorDeg = Double.NaN;
        previousAimErrorDeg = Double.NaN;

        SmartDashboard.putString("AlignOnly/State", "ALIGN");
        SmartDashboard.putString("AlignOnly/FinishReason", "");
        SmartDashboard.putNumber("AlignOnly/HeadingError", Double.NaN);
        SmartDashboard.putNumber("AlignOnly/FilteredError", Double.NaN);
        SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);
    }

    @Override
    public void execute() {
        Translation2d hubCenter = RobotContainer.getAllianceHubCenter();
        boolean hasVision = swerve.isVisionActive();

        if (!hasVision) {
            swerve.drive(0, 0, 0, false);
            alignedHoldTimer.stop();
            alignedHoldTimer.reset();
            filteredHeadingErrorDeg = Double.NaN;
            previousAimErrorDeg = Double.NaN;

            SmartDashboard.putString("AlignOnly/State", "NO_TARGET");
            SmartDashboard.putNumber("AlignOnly/HeadingError", Double.NaN);
            SmartDashboard.putNumber("AlignOnly/FilteredError", Double.NaN);
            SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);

            if (noTargetTimer.hasElapsed(NO_TARGET_TIMEOUT_SEC)) {
                finishReason = "No vision-corrected pose";
                finished = true;
            }
            return;
        }

        noTargetTimer.reset();

        double headingErrorDeg = swerve.getHeadingErrorDegTo(hubCenter);
        double filteredError = filterHeadingError(headingErrorDeg);
        double aimErrorDeg = filteredError;

        SmartDashboard.putNumber("AlignOnly/HeadingError", headingErrorDeg);
        SmartDashboard.putNumber("AlignOnly/FilteredError", aimErrorDeg);
        SmartDashboard.putNumber("AlignOnly/DistanceM", swerve.getDistanceTo(hubCenter));

        if (AlignmentCaptureUtil.shouldCaptureOnEntryOrCrossing(
                previousAimErrorDeg,
                aimErrorDeg,
                Constants.AlignShoot.YAW_TOLERANCE_DEG,
                Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG,
                Constants.AlignShoot.CAPTURE_OVERSHOOT_DEG)) {
            swerve.drive(0, 0, 0, false);
            SmartDashboard.putString("AlignOnly/State", "ALIGNED");
            SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);
            finishReason = "Aligned";
            finished = true;
            previousAimErrorDeg = aimErrorDeg;
            return;
        }
        if (shouldHoldAlignment(aimErrorDeg)) {
            swerve.drive(0, 0, 0, false);
            SmartDashboard.putString("AlignOnly/State", "ALIGNED");
            SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);

            if (!alignedHoldTimer.isRunning()) {
                alignedHoldTimer.restart();
            }
            if (alignedHoldTimer.hasElapsed(STABLE_ALIGNMENT_TIME_SEC)) {
                finishReason = "Aligned";
                finished = true;
            }
            previousAimErrorDeg = aimErrorDeg;
            return;
        }

        alignedHoldTimer.stop();
        alignedHoldTimer.reset();
        // Negate: positive heading error → positive omega (CCW toward target).
        double rotCmd = MathUtil.clamp(
                -turnPID.calculate(filteredError, 0.0),
                -Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS,
                Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS);
        swerve.drive(0, 0, rotCmd, false);
        SmartDashboard.putString("AlignOnly/State", "ALIGNING");
        SmartDashboard.putNumber("AlignOnly/RotCmd", rotCmd);
        previousAimErrorDeg = aimErrorDeg;

        if (alignTimer.hasElapsed(ALIGN_CONVERGENCE_TIMEOUT_SEC)) {
            finishReason = "Alignment convergence timeout";
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        SmartDashboard.putString("AlignOnly/State", "IDLE");

        if (interrupted) {
            finishReason = "Interrupted";
        } else if (finishReason.isBlank()) {
            finishReason = "Finished";
        }
        SmartDashboard.putString("AlignOnly/FinishReason", finishReason);
        System.out.println("[AlignOnly] Ended: " + finishReason);
    }

    private double filterHeadingError(double rawErrorDeg) {
        if (!Double.isFinite(rawErrorDeg)) {
            filteredHeadingErrorDeg = Double.NaN;
            return Double.NaN;
        }
        if (!Double.isFinite(filteredHeadingErrorDeg)) {
            filteredHeadingErrorDeg = rawErrorDeg;
        } else {
            filteredHeadingErrorDeg = Constants.AlignShoot.YAW_FILTER_ALPHA * filteredHeadingErrorDeg
                    + (1.0 - Constants.AlignShoot.YAW_FILTER_ALPHA) * rawErrorDeg;
        }
        return filteredHeadingErrorDeg;
    }

    private boolean shouldHoldAlignment(double errorDeg) {
        if (!Double.isFinite(errorDeg)) {
            return false;
        }
        double absError = Math.abs(errorDeg);
        return absError <= Constants.AlignShoot.YAW_TOLERANCE_DEG
                || (alignedHoldTimer.isRunning()
                        && absError <= Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG);
    }
}
