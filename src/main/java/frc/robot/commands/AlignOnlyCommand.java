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
import frc.robot.vision.VisionSupport;

/**
 * Rotates the robot to center the active vision target yaw without running any
 * shooter/feed/intake motors. Intended for alignment-direction bring-up.
 */
public class AlignOnlyCommand extends Command {
    private static final double NO_TARGET_TIMEOUT_SEC = 3.0;
    private static final double ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0;
    private static final double STABLE_ALIGNMENT_TIME_SEC = Constants.AlignShoot.SETTLE_TIME_SEC;

    private final SwerveSubsystem swerve;
    private final AtomicReference<VisionResult> visionRef;
    private final PIDController turnPID = new PIDController(
            Constants.AlignShoot.TURN_kP,
            0.0,
            Constants.AlignShoot.TURN_kD);

    private final Timer noTargetTimer = new Timer();
    private final Timer alignTimer = new Timer();
    private final Timer alignedHoldTimer = new Timer();
    private double filteredYawDeg = Double.NaN;

    private boolean finished;
    private String finishReason = "";

    public AlignOnlyCommand(SwerveSubsystem swerve, AtomicReference<VisionResult> visionRef) {
        this.swerve = swerve;
        this.visionRef = visionRef;

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
        filteredYawDeg = Double.NaN;

        SmartDashboard.putString("AlignOnly/State", "ALIGN");
        SmartDashboard.putString("AlignOnly/FinishReason", "");
        SmartDashboard.putNumber("AlignOnly/YawError", Double.NaN);
        SmartDashboard.putNumber("AlignOnly/FilteredYawError", Double.NaN);
        SmartDashboard.putNumber("AlignOnly/YawSetpointDeg", Double.NaN);
        SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);
    }

    @Override
    public void execute() {
        VisionResult result = visionRef.get();
        if (!isResultFresh(result)) {
            swerve.drive(0, 0, 0, false);
            alignedHoldTimer.stop();
            alignedHoldTimer.reset();
            filteredYawDeg = Double.NaN;

            SmartDashboard.putString("AlignOnly/State", "NO_TARGET");
            SmartDashboard.putNumber("AlignOnly/YawError", Double.NaN);
            SmartDashboard.putNumber("AlignOnly/FilteredYawError", Double.NaN);
            SmartDashboard.putNumber("AlignOnly/YawSetpointDeg", Double.NaN);
            SmartDashboard.putNumber("AlignOnly/RotCmd", 0.0);

            if (noTargetTimer.hasElapsed(NO_TARGET_TIMEOUT_SEC)) {
                finishReason = "No fresh vision target";
                finished = true;
            }
            return;
        }

        noTargetTimer.reset();

        double yawDeg = result.yawDeg();
        double filteredYawDeg = filterYaw(yawDeg);
        double distanceM = estimateDistanceM(result);
        double yawSetpointDeg = calculateCameraYawSetpointDeg(distanceM);
        double aimErrorDeg = filteredYawDeg - yawSetpointDeg;

        SmartDashboard.putNumber("AlignOnly/TargetTagId", result.tagId());
        SmartDashboard.putNumber("AlignOnly/TargetYawDeg", yawDeg);
        SmartDashboard.putNumber("AlignOnly/YawSetpointDeg", yawSetpointDeg);
        SmartDashboard.putNumber("AlignOnly/YawError", aimErrorDeg);
        SmartDashboard.putNumber("AlignOnly/FilteredYawError", aimErrorDeg);
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
            return;
        }

        alignedHoldTimer.stop();
        alignedHoldTimer.reset();
        double rotCmd = MathUtil.clamp(
                turnPID.calculate(filteredYawDeg, yawSetpointDeg),
                -Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS,
                Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS);
        swerve.drive(0, 0, rotCmd, false);
        SmartDashboard.putString("AlignOnly/State", "ALIGNING");
        SmartDashboard.putNumber("AlignOnly/RotCmd", rotCmd);

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

    private double filterYaw(double rawYawDeg) {
        if (!Double.isFinite(rawYawDeg)) {
            filteredYawDeg = Double.NaN;
            return Double.NaN;
        }
        if (!Double.isFinite(filteredYawDeg)) {
            filteredYawDeg = rawYawDeg;
        } else {
            filteredYawDeg = Constants.AlignShoot.YAW_FILTER_ALPHA * filteredYawDeg
                    + (1.0 - Constants.AlignShoot.YAW_FILTER_ALPHA) * rawYawDeg;
        }
        return filteredYawDeg;
    }

    private boolean shouldHoldAlignment(double filteredYawDeg) {
        if (!Double.isFinite(filteredYawDeg)) {
            return false;
        }
        double absYawDeg = Math.abs(filteredYawDeg);
        return absYawDeg <= Constants.AlignShoot.YAW_TOLERANCE_DEG
                || (alignedHoldTimer.isRunning()
                        && absYawDeg <= Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG);
    }

    private double calculateCameraYawSetpointDeg(double distanceM) {
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return 0.0;
        }
        return Math.toDegrees(Math.atan2(-Constants.Vision.CAMERA_LATERAL_OFFSET_M, distanceM));
    }

    private double estimateDistanceM(VisionResult result) {
        double distanceM = result.estimateDistanceM(
                Constants.Vision.TAG_HEIGHT_M,
                Constants.Vision.FOCAL_LENGTH_PIXELS);
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            distanceM = estimateDistanceFromPitch(result.pitchDeg());
        }
        if (Double.isFinite(distanceM)) {
            distanceM = VisionSupport.calibrateDistanceM(distanceM);
        }
        return distanceM;
    }

    private double estimateDistanceFromPitch(double targetPitchDeg) {
        double totalPitchRad = Constants.Vision.CAMERA_PITCH_RAD
                + Math.toRadians(targetPitchDeg);
        double heightDiff = Constants.Shooter.HUB_SCORING_HEIGHT_M - Constants.Vision.CAMERA_UP_M;
        double tanPitch = Math.tan(totalPitchRad);
        if (Math.abs(tanPitch) < 1e-6) {
            return Double.NaN;
        }
        return heightDiff / tanPitch;
    }
}
