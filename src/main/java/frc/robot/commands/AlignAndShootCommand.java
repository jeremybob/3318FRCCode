// ============================================================================
// FILE: src/main/java/frc/robot/commands/AlignAndShootCommand.java
//
// PURPOSE: Rotates the robot to aim at a vision target, then fires.
//
// SEQUENCE:
//   1. SPIN UP  — Start shooter wheels spinning to target speed
//   2. ALIGN    — Rotate robot until vision target is centered in camera
//   3. CLEAR    — Brief feeder reverse to prevent double-feeding
//   4. FEED     — Push game piece through feeder + hopper + intake roller
//   5. DONE     — Command finishes, everything stops
//
// VISION SOURCE: Reads VisionResult from the background RioVisionThread via
//   an AtomicReference.  No PhotonVision dependency.
// ============================================================================
package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.HubActivityTracker;
import frc.robot.subsystems.*;
import frc.robot.vision.VisionResult;

public class AlignAndShootCommand extends Command {

    // --------------------------------------------------------------------------
    // Static telemetry snapshot
    // --------------------------------------------------------------------------
    private static volatile String telemetryState = "IDLE";
    private static volatile boolean telemetryCommandActive = false;
    private static volatile boolean telemetryHasTarget = false;
    private static volatile boolean telemetryGeometryFeasible = false;
    private static volatile boolean telemetryHasShootableTarget = false;
    private static volatile double telemetryYawDeg = Double.NaN;
    private static volatile double telemetryPitchDeg = Double.NaN;
    private static volatile double telemetryTargetRps = Double.NaN;
    private static volatile String telemetryLastAbortReason = "";

    // All the subsystems this command needs to control
    private final SwerveSubsystem  swerve;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem  feeder;
    private final HopperSubsystem  hopper;
    private final IntakeSubsystem  intake;

    // Vision data from the background RioVisionThread
    private final AtomicReference<VisionResult> visionRef;

    // PD controller for rotating toward the target.
    private final PIDController turnPID = new PIDController(
            Constants.Vision.TURN_kP,
            0,
            Constants.Vision.TURN_kD);

    // ---- State machine ----
    private enum State { SPIN_UP, ALIGN, CLEAR, FEED, DONE }
    private State state;

    private final Timer stateTimer = new Timer();
    private double lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
    private double calculatedRPS = Constants.Shooter.TARGET_RPS;

    private static final double ALIGN_TIMEOUT_SEC = 3.0;

    // Height of the HUB tags (for pitch-based distance estimation)
    private static final double HUB_TAG_HEIGHT_M = 1.124;

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public AlignAndShootCommand(SwerveSubsystem swerve,
                                 ShooterSubsystem shooter,
                                 FeederSubsystem feeder,
                                 HopperSubsystem hopper,
                                 IntakeSubsystem intake,
                                 AtomicReference<VisionResult> visionRef) {
        this.swerve  = swerve;
        this.shooter = shooter;
        this.feeder  = feeder;
        this.hopper  = hopper;
        this.intake  = intake;
        this.visionRef = visionRef;

        addRequirements(swerve, shooter, feeder, hopper, intake);
        turnPID.setTolerance(Constants.Vision.YAW_TOLERANCE_DEG);
    }

    // --------------------------------------------------------------------------
    // initialize()
    // --------------------------------------------------------------------------
    @Override
    public void initialize() {
        state = State.SPIN_UP;
        stateTimer.reset();
        stateTimer.start();
        lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
        telemetryState = State.SPIN_UP.name();
        telemetryCommandActive = true;
        telemetryHasTarget = false;
        telemetryGeometryFeasible = false;
        telemetryHasShootableTarget = false;
        telemetryYawDeg = Double.NaN;
        telemetryPitchDeg = Double.NaN;
        telemetryTargetRps = Double.NaN;
        telemetryLastAbortReason = "";

        if (!HubActivityTracker.isOurHubActive()) {
            System.out.println("[AlignAndShoot] WARNING: Alliance HUB is currently INACTIVE!");
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", true);
        } else {
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", false);
        }

        calculatedRPS = Constants.Shooter.TARGET_RPS;
        telemetryTargetRps = calculatedRPS;
        shooter.setShooterVelocity(calculatedRPS);
        SmartDashboard.putString("AlignShoot/State", "SPIN_UP");
    }

    // --------------------------------------------------------------------------
    // execute()
    // --------------------------------------------------------------------------
    @Override
    public void execute() {
        switch (state) {

            case SPIN_UP: {
                swerve.drive(0, 0, 0, false);
                if (shooter.isAtSpeed(calculatedRPS)
                        || stateTimer.hasElapsed(Constants.Shooter.AT_SPEED_TIMEOUT_SEC)) {
                    transitionTo(State.ALIGN);
                }
                break;
            }

            case ALIGN: {
                VisionResult result = visionRef.get();
                boolean hasResult = isResultFresh(result);
                telemetryHasTarget = hasResult;

                if (!hasResult) {
                    swerve.drive(0, 0, 0, false);
                    telemetryGeometryFeasible = false;
                    telemetryHasShootableTarget = false;
                    telemetryYawDeg = Double.NaN;
                    telemetryPitchDeg = Double.NaN;

                    if (stateTimer.hasElapsed(ALIGN_TIMEOUT_SEC)) {
                        System.out.println("[AlignAndShoot] No alliance HUB tag found, aborting.");
                        telemetryLastAbortReason = "No alliance HUB tag found";
                        transitionTo(State.DONE);
                    }
                    break;
                }

                if (!isShotGeometryFeasible(result)) {
                    System.out.println("[AlignAndShoot] Shot geometry not feasible, aborting.");
                    telemetryHasShootableTarget = false;
                    telemetryLastAbortReason = "Shot geometry not feasible";
                    transitionTo(State.DONE);
                    break;
                }
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                telemetryHasShootableTarget = true;
                telemetryGeometryFeasible = true;

                double yawDeg = result.yawDeg();
                SmartDashboard.putNumber("AlignShoot/YawError", yawDeg);
                SmartDashboard.putNumber("AlignShoot/TargetTagId", result.tagId());
                telemetryYawDeg = yawDeg;

                // Distance estimation: prefer pixel-height method, fall back to pitch
                double distanceM = result.estimateDistanceM(
                        Constants.Vision.TAG_HEIGHT_M,
                        Constants.Vision.FOCAL_LENGTH_PIXELS);
                if (!Double.isFinite(distanceM) || distanceM <= 0) {
                    distanceM = estimateDistanceFromPitch(result.pitchDeg());
                }
                calculatedRPS = ShooterSubsystem.calculateTargetRPS(distanceM);
                telemetryTargetRps = calculatedRPS;
                shooter.setShooterVelocity(calculatedRPS);
                SmartDashboard.putNumber("AlignShoot/CalculatedRPS", calculatedRPS);
                SmartDashboard.putNumber("AlignShoot/EstDistanceM", distanceM);

                double rotCmd = turnPID.calculate(yawDeg, 0);
                rotCmd = MathUtil.clamp(rotCmd,
                        -Constants.Vision.MAX_ROT_CMD, Constants.Vision.MAX_ROT_CMD);

                if (turnPID.atSetpoint() && shooter.isAtSpeed(calculatedRPS)) {
                    swerve.drive(0, 0, 0, false);
                    transitionTo(State.CLEAR);
                } else if (turnPID.atSetpoint()) {
                    swerve.drive(0, 0, 0, false);
                } else {
                    swerve.drive(0, 0, rotCmd, false);
                }
                break;
            }

            case CLEAR: {
                if (!hasShootableTarget()) {
                    System.out.println("[AlignAndShoot] Vision lost or geometry invalid before feed, aborting.");
                    telemetryLastAbortReason = "Vision lost before feed";
                    transitionTo(State.DONE);
                    break;
                }
                feeder.setPower(Constants.Shooter.CLEAR_POWER);
                if (stateTimer.hasElapsed(Constants.Shooter.CLEAR_TIME_SEC)) {
                    feeder.stop();
                    transitionTo(State.FEED);
                }
                break;
            }

            case FEED: {
                if (!hasShootableTarget()) {
                    System.out.println("[AlignAndShoot] Vision lost or geometry invalid during feed, aborting.");
                    telemetryLastAbortReason = "Vision lost during feed";
                    transitionTo(State.DONE);
                    break;
                }
                feeder.setPower(Constants.Shooter.FEED_POWER);
                hopper.setPower(Constants.Shooter.FEED_POWER);
                intake.setRollerPower(Constants.Shooter.FEED_POWER);
                if (stateTimer.hasElapsed(Constants.Shooter.FEED_TIME_SEC)) {
                    transitionTo(State.DONE);
                }
                break;
            }

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        shooter.stop();
        feeder.stop();
        hopper.stop();
        intake.setRollerPower(0);
        telemetryState = "IDLE";
        telemetryCommandActive = false;
        telemetryHasShootableTarget = false;
        telemetryTargetRps = Double.NaN;
        SmartDashboard.putString("AlignShoot/State", "IDLE");
        if (interrupted) {
            System.out.println("[AlignAndShoot] Command was interrupted.");
            telemetryLastAbortReason = "Interrupted";
        }
    }

    // --------------------------------------------------------------------------
    // Helpers
    // --------------------------------------------------------------------------

    private void transitionTo(State newState) {
        state = newState;
        stateTimer.reset();
        stateTimer.start();
        telemetryState = newState.toString();
        SmartDashboard.putString("AlignShoot/State", newState.toString());
    }

    /** Check if a VisionResult is recent enough to use (not stale). */
    private boolean isResultFresh(VisionResult result) {
        if (result == null || result.tagId() < 0) return false;
        double age = Timer.getFPGATimestamp() - result.timestampSec();
        // Accept results up to 0.5 s old (accounts for lower frame rate)
        return age < Constants.Vision.TARGET_LOSS_TOLERANCE_SEC + 0.15;
    }

    private boolean hasShootableTarget() {
        VisionResult result = visionRef.get();
        boolean hasResult = isResultFresh(result);
        telemetryHasTarget = hasResult;

        if (hasResult) {
            boolean geometryFeasible = isShotGeometryFeasible(result);
            telemetryGeometryFeasible = geometryFeasible;
            if (geometryFeasible) {
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                telemetryHasShootableTarget = true;
                return true;
            }
            telemetryHasShootableTarget = false;
            return false;
        }
        telemetryGeometryFeasible = false;
        telemetryHasShootableTarget = hasRecentValidTarget();
        return hasRecentValidTarget();
    }

    private boolean hasRecentValidTarget() {
        return Timer.getFPGATimestamp() - lastValidTargetSeenSec
                <= Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }

    private boolean isShotGeometryFeasible(VisionResult result) {
        double pitchDeg = result.pitchDeg();
        SmartDashboard.putNumber("AlignShoot/TargetPitchDeg", pitchDeg);
        telemetryPitchDeg = pitchDeg;

        return isShotPitchFeasible(pitchDeg);
    }

    /**
     * Fallback distance estimation using pitch angle + camera mount geometry.
     * distance = (tagHeight - cameraHeight) / tan(cameraPitch + targetPitch)
     */
    private double estimateDistanceFromPitch(double targetPitchDeg) {
        double totalPitchRad = Constants.Vision.CAMERA_PITCH_RAD
                + Math.toRadians(targetPitchDeg);
        double heightDiff = HUB_TAG_HEIGHT_M - Constants.Vision.CAMERA_UP_M;
        double tanPitch = Math.tan(totalPitchRad);
        if (Math.abs(tanPitch) < 1e-6) return Double.NaN;
        return heightDiff / tanPitch;
    }

    // --------------------------------------------------------------------------
    // Static telemetry accessors (for dashboard)
    // --------------------------------------------------------------------------
    public static String getTelemetryState() { return telemetryState; }
    public static boolean isTelemetryCommandActive() { return telemetryCommandActive; }
    public static boolean telemetryHasTarget() { return telemetryHasTarget; }
    public static boolean telemetryGeometryFeasible() { return telemetryGeometryFeasible; }
    public static boolean telemetryHasShootableTarget() { return telemetryHasShootableTarget; }
    public static double getTelemetryYawDeg() { return telemetryYawDeg; }
    public static double getTelemetryPitchDeg() { return telemetryPitchDeg; }
    public static double getTelemetryTargetRps() { return telemetryTargetRps; }
    public static String getTelemetryLastAbortReason() { return telemetryLastAbortReason; }

    static boolean isShotPitchFeasible(double pitchDeg) {
        return Double.isFinite(pitchDeg)
                && pitchDeg >= Constants.Vision.MIN_SHOT_PITCH_DEG
                && pitchDeg <= Constants.Vision.MAX_SHOT_PITCH_DEG;
    }
}
