// ============================================================================
// FILE: src/main/java/frc/robot/commands/AlignAndShootCommand.java
//
// PURPOSE: Rotates the robot to aim at a vision target, then fires.
//
// SEQUENCE:
//   1. ALIGN    — Start shooter wheels AND rotate toward vision target simultaneously
//   2. CLEAR    — Brief feeder reverse to prevent double-feeding (with active alignment)
//   3. FEED     — Push game piece through feeder + hopper + intake roller (committed, no abort)
//   4. DONE     — Command finishes, everything stops
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
    // Atomic telemetry snapshot — published as one immutable object so dashboard
    // readers always see a consistent set of fields (fixes issue #10).
    // --------------------------------------------------------------------------
    public record TelemetrySnapshot(
            String state,
            boolean commandActive,
            boolean hasTarget,
            boolean geometryFeasible,
            boolean hasShootableTarget,
            double yawDeg,
            double pitchDeg,
            double targetRps,
            String lastAbortReason) {}

    private static final TelemetrySnapshot IDLE_SNAPSHOT = new TelemetrySnapshot(
            "IDLE", false, false, false, false, Double.NaN, Double.NaN, Double.NaN, "");

    private static volatile TelemetrySnapshot telemetrySnapshot = IDLE_SNAPSHOT;

    // Mutable working fields — written only from the command scheduler thread,
    // then published atomically via updateTelemetry().
    private String workState = "IDLE";
    private boolean workCommandActive = false;
    private boolean workHasTarget = false;
    private boolean workGeometryFeasible = false;
    private boolean workHasShootableTarget = false;
    private double workYawDeg = Double.NaN;
    private double workPitchDeg = Double.NaN;
    private double workTargetRps = Double.NaN;
    private String workLastAbortReason = "";

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
    private enum State { ALIGN, CLEAR, FEED, DONE }
    private State state;

    private final Timer stateTimer = new Timer();
    // Overall timer for the ALIGN state — covers both "no target" and "can't converge".
    private final Timer alignOverallTimer = new Timer();
    private double lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
    private double calculatedRPS = Constants.Shooter.TARGET_RPS;

    private static final double ALIGN_TIMEOUT_SEC = 3.0;
    // Maximum time in ALIGN with a target visible but PID/shooter not converging.
    private static final double ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0;

    // Height of the HUB tags above ground (for pitch-based distance estimation).
    // This is the tag's vertical POSITION on the field, NOT its physical size.
    // Uses Constants.Shooter.HUB_SCORING_HEIGHT_M to avoid duplicate constants.

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
        state = State.ALIGN;
        stateTimer.reset();
        stateTimer.start();
        alignOverallTimer.reset();
        alignOverallTimer.start();
        lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;

        workState = State.ALIGN.name();
        workCommandActive = true;
        workHasTarget = false;
        workGeometryFeasible = false;
        workHasShootableTarget = false;
        workYawDeg = Double.NaN;
        workPitchDeg = Double.NaN;
        workLastAbortReason = "";

        if (!HubActivityTracker.isOurHubActive()) {
            double secsToShift = HubActivityTracker.secondsUntilNextShiftChange();
            System.out.println("[AlignAndShoot] WARNING: Alliance HUB is INACTIVE! "
                    + String.format("%.1fs", secsToShift) + " until next shift change.");
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", true);
        } else {
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", false);
        }

        calculatedRPS = Constants.Shooter.TARGET_RPS;
        workTargetRps = calculatedRPS;
        shooter.setShooterVelocity(calculatedRPS);
        SmartDashboard.putString("AlignShoot/State", "ALIGN");
        updateTelemetry();
    }

    // --------------------------------------------------------------------------
    // execute()
    // --------------------------------------------------------------------------
    @Override
    public void execute() {
        switch (state) {

            case ALIGN: {
                VisionResult result = visionRef.get();
                boolean hasResult = isResultFresh(result);
                workHasTarget = hasResult;

                if (!hasResult) {
                    swerve.drive(0, 0, 0, false);
                    workGeometryFeasible = false;
                    workHasShootableTarget = false;
                    workYawDeg = Double.NaN;
                    workPitchDeg = Double.NaN;

                    if (stateTimer.hasElapsed(ALIGN_TIMEOUT_SEC)) {
                        System.out.println("[AlignAndShoot] No alliance HUB tag found, aborting.");
                        workLastAbortReason = "No alliance HUB tag found";
                        transitionTo(State.DONE);
                    }
                    break;
                }

                if (!isShotGeometryFeasible(result)) {
                    System.out.println("[AlignAndShoot] Shot geometry not feasible, aborting.");
                    workHasShootableTarget = false;
                    workLastAbortReason = "Shot geometry not feasible";
                    transitionTo(State.DONE);
                    break;
                }
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                workHasShootableTarget = true;
                workGeometryFeasible = true;

                updateShooterFromVision(result);

                double rotCmd = calculateAlignmentRotation(result);

                if (turnPID.atSetpoint() && shooter.isAtSpeed(calculatedRPS)) {
                    swerve.drive(0, 0, 0, false);
                    transitionTo(State.CLEAR);
                } else if (alignOverallTimer.hasElapsed(ALIGN_CONVERGENCE_TIMEOUT_SEC)) {
                    System.out.println("[AlignAndShoot] ALIGN convergence timeout, aborting.");
                    workLastAbortReason = "Alignment convergence timeout";
                    transitionTo(State.DONE);
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
                    workLastAbortReason = "Vision lost before feed";
                    transitionTo(State.DONE);
                    break;
                }
                // Maintain alignment while clearing
                alignIfVisionAvailable();
                feeder.setPower(Constants.Shooter.CLEAR_POWER);
                if (stateTimer.hasElapsed(Constants.Shooter.CLEAR_TIME_SEC)) {
                    feeder.stop();
                    transitionTo(State.FEED);
                }
                break;
            }

            case FEED: {
                // Once committed to feeding, finish the feed to avoid jamming.
                // A game piece partially in the shooter must be pushed through.
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
        updateTelemetry();
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
        SmartDashboard.putString("AlignShoot/State", "IDLE");
        if (interrupted) {
            System.out.println("[AlignAndShoot] Command was interrupted.");
            workLastAbortReason = "Interrupted";
        }
        workState = "IDLE";
        workCommandActive = false;
        workHasShootableTarget = false;
        workTargetRps = Double.NaN;
        updateTelemetry();
    }

    // --------------------------------------------------------------------------
    // Helpers
    // --------------------------------------------------------------------------

    private void transitionTo(State newState) {
        state = newState;
        stateTimer.reset();
        stateTimer.start();
        workState = newState.toString();
        SmartDashboard.putString("AlignShoot/State", newState.toString());
    }

    /** Publish a consistent telemetry snapshot atomically. */
    private void updateTelemetry() {
        telemetrySnapshot = new TelemetrySnapshot(
                workState, workCommandActive, workHasTarget,
                workGeometryFeasible, workHasShootableTarget,
                workYawDeg, workPitchDeg, workTargetRps, workLastAbortReason);
    }

    /** Update shooter velocity from the current vision result. */
    private void updateShooterFromVision(VisionResult result) {
        double yawDeg = result.yawDeg();
        SmartDashboard.putNumber("AlignShoot/YawError", yawDeg);
        SmartDashboard.putNumber("AlignShoot/TargetTagId", result.tagId());
        workYawDeg = yawDeg;

        double distanceM = result.estimateDistanceM(
                Constants.Vision.TAG_HEIGHT_M,
                Constants.Vision.FOCAL_LENGTH_PIXELS);
        if (!Double.isFinite(distanceM) || distanceM <= 0) {
            double pitchFallback = estimateDistanceFromPitch(result.pitchDeg());
            System.out.println("[AlignAndShoot] Primary distance invalid ("
                    + String.format("%.2f", distanceM)
                    + "m), using pitch-based fallback: "
                    + String.format("%.2f", pitchFallback) + "m");
            distanceM = pitchFallback;
        }
        calculatedRPS = ShooterSubsystem.calculateTargetRPS(distanceM);
        workTargetRps = calculatedRPS;
        shooter.setShooterVelocity(calculatedRPS);
        SmartDashboard.putNumber("AlignShoot/CalculatedRPS", calculatedRPS);
        SmartDashboard.putNumber("AlignShoot/EstDistanceM", distanceM);
    }

    /** Calculate the rotation command to align toward the vision target. */
    private double calculateAlignmentRotation(VisionResult result) {
        double yawDeg = result.yawDeg();
        double rotCmd = turnPID.calculate(yawDeg, 0);
        return MathUtil.clamp(rotCmd,
                -Constants.Vision.MAX_ROT_CMD, Constants.Vision.MAX_ROT_CMD);
    }

    /** Actively align toward the target if vision data is available. */
    private void alignIfVisionAvailable() {
        VisionResult result = visionRef.get();
        if (isResultFresh(result)) {
            double rotCmd = calculateAlignmentRotation(result);
            if (!turnPID.atSetpoint()) {
                swerve.drive(0, 0, rotCmd, false);
                return;
            }
        }
        swerve.drive(0, 0, 0, false);
    }

    /** Check if a VisionResult is recent enough to use (not stale). */
    private boolean isResultFresh(VisionResult result) {
        if (result == null || result.tagId() < 0) return false;
        double age = Timer.getFPGATimestamp() - result.timestampSec();
        return age < Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }

    private boolean hasShootableTarget() {
        VisionResult result = visionRef.get();
        boolean hasResult = isResultFresh(result);
        workHasTarget = hasResult;

        if (hasResult) {
            boolean geometryFeasible = isShotGeometryFeasible(result);
            workGeometryFeasible = geometryFeasible;
            if (geometryFeasible) {
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                workHasShootableTarget = true;
                return true;
            }
            workHasShootableTarget = false;
            return false;
        }
        workGeometryFeasible = false;
        boolean recent = hasRecentValidTarget();
        workHasShootableTarget = recent;
        return recent;
    }

    private boolean hasRecentValidTarget() {
        return Timer.getFPGATimestamp() - lastValidTargetSeenSec
                <= Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }

    private boolean isShotGeometryFeasible(VisionResult result) {
        double pitchDeg = result.pitchDeg();
        double yawDeg = result.yawDeg();
        SmartDashboard.putNumber("AlignShoot/TargetPitchDeg", pitchDeg);
        SmartDashboard.putNumber("AlignShoot/YawGeometryCheck", yawDeg);
        workPitchDeg = pitchDeg;

        // Reject shots where the robot is facing far off-target.
        if (Math.abs(yawDeg) > Constants.Vision.YAW_TOLERANCE_DEG * 10) {
            return false;
        }

        return isShotPitchFeasible(pitchDeg);
    }

    /**
     * Fallback distance estimation using pitch angle + camera mount geometry.
     * distance = (tagHeight - cameraHeight) / tan(cameraPitch + targetPitch)
     */
    private double estimateDistanceFromPitch(double targetPitchDeg) {
        double totalPitchRad = Constants.Vision.CAMERA_PITCH_RAD
                + Math.toRadians(targetPitchDeg);
        double heightDiff = Constants.Shooter.HUB_SCORING_HEIGHT_M - Constants.Vision.CAMERA_UP_M;
        double tanPitch = Math.tan(totalPitchRad);
        if (Math.abs(tanPitch) < 1e-6) return Double.NaN;
        return heightDiff / tanPitch;
    }

    // --------------------------------------------------------------------------
    // Static telemetry accessors (for dashboard)
    // --------------------------------------------------------------------------
    public static TelemetrySnapshot getTelemetrySnapshot() { return telemetrySnapshot; }

    public static String getTelemetryState() { return telemetrySnapshot.state(); }
    public static boolean isTelemetryCommandActive() { return telemetrySnapshot.commandActive(); }
    public static boolean telemetryHasTarget() { return telemetrySnapshot.hasTarget(); }
    public static boolean telemetryGeometryFeasible() { return telemetrySnapshot.geometryFeasible(); }
    public static boolean telemetryHasShootableTarget() { return telemetrySnapshot.hasShootableTarget(); }
    public static double getTelemetryYawDeg() { return telemetrySnapshot.yawDeg(); }
    public static double getTelemetryPitchDeg() { return telemetrySnapshot.pitchDeg(); }
    public static double getTelemetryTargetRps() { return telemetrySnapshot.targetRps(); }
    public static String getTelemetryLastAbortReason() { return telemetrySnapshot.lastAbortReason(); }

    static boolean isShotPitchFeasible(double pitchDeg) {
        return Double.isFinite(pitchDeg)
                && pitchDeg >= Constants.Vision.MIN_SHOT_PITCH_DEG
                && pitchDeg <= Constants.Vision.MAX_SHOT_PITCH_DEG;
    }
}
