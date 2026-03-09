package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionResult;

/**
 * Rotates the robot to center the active vision target yaw without running any
 * shooter/feed/intake motors. Intended for alignment-direction bring-up.
 */
public class AlignOnlyCommand extends Command {
    private static final double NO_TARGET_TIMEOUT_SEC = 3.0;
    private static final double ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0;
    private static final double STABLE_ALIGNMENT_TIME_SEC = 0.20;

    private final SwerveSubsystem swerve;
    private final AtomicReference<VisionResult> visionRef;
    private final PIDController turnPID = new PIDController(
            Constants.Vision.TURN_kP,
            0.0,
            Constants.Vision.TURN_kD);

    private final Timer noTargetTimer = new Timer();
    private final Timer alignTimer = new Timer();
    private final Timer alignedHoldTimer = new Timer();

    private boolean finished;
    private String finishReason = "";

    public AlignOnlyCommand(SwerveSubsystem swerve, AtomicReference<VisionResult> visionRef) {
        this.swerve = swerve;
        this.visionRef = visionRef;

        addRequirements(swerve);
        turnPID.setTolerance(Constants.Vision.YAW_TOLERANCE_DEG);
    }

    @Override
    public void initialize() {
        finished = false;
        finishReason = "";

        noTargetTimer.restart();
        alignTimer.restart();
        alignedHoldTimer.stop();
        alignedHoldTimer.reset();

        SmartDashboard.putString("AlignOnly/State", "ALIGN");
        SmartDashboard.putString("AlignOnly/FinishReason", "");
        SmartDashboard.putNumber("AlignOnly/YawError", Double.NaN);
        SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);
    }

    @Override
    public void execute() {
        VisionResult result = visionRef.get();
        if (!isResultFresh(result)) {
            swerve.drive(0, 0, 0, false);
            alignedHoldTimer.stop();
            alignedHoldTimer.reset();

            SmartDashboard.putString("AlignOnly/State", "NO_TARGET");
            SmartDashboard.putNumber("AlignOnly/YawError", Double.NaN);
            SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);

            if (noTargetTimer.hasElapsed(NO_TARGET_TIMEOUT_SEC)) {
                finishReason = "No fresh vision target";
                finished = true;
            }
            return;
        }

        noTargetTimer.reset();

        double yawDeg = result.yawDeg();
        double rotCmd = MathUtil.clamp(
                turnPID.calculate(yawDeg, 0.0),
                -Constants.Vision.MAX_ROT_CMD,
                Constants.Vision.MAX_ROT_CMD);

        SmartDashboard.putNumber("AlignOnly/TargetTagId", result.tagId());
        SmartDashboard.putNumber("AlignOnly/YawError", yawDeg);
        SmartDashboard.putNumber("AlignOnly/RotCmd", rotCmd);

        if (turnPID.atSetpoint()) {
            swerve.drive(0, 0, 0, false);
            SmartDashboard.putString("AlignOnly/State", "ALIGNED");

            if (!alignedHoldTimer.isRunning()) {
                alignedHoldTimer.restart();
            }
            if (alignedHoldTimer.hasElapsed(STABLE_ALIGNMENT_TIME_SEC)) {
                finishReason = "Aligned";
                finished = true;
            }
            return;
        }

        alignedHoldTimer.stop();
        alignedHoldTimer.reset();
        swerve.drive(0, 0, rotCmd, false);
        SmartDashboard.putString("AlignOnly/State", "ALIGNING");

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

    private boolean isResultFresh(VisionResult result) {
        if (result == null || result.tagId() < 0) {
            return false;
        }
        double ageSec = Timer.getFPGATimestamp() - result.timestampSec();
        return ageSec < Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }
}
