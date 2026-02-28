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
// KEY FIXES FROM v1:
//   1. PhotonCamera is now INJECTED (passed in), not created inside the command.
//      Creating it inside caused a new network connection on every command start.
//   2. ALIGN state now has its OWN timeout separate from the at-speed timeout.
//      Previously, the robot would fire even if it never aligned to a target.
//   3. Extends Command (not deprecated CommandBase).
//
// NOTE: The camera is owned by RobotContainer and shared with this command.
//   Only one command should use the camera at a time.
// ============================================================================
package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;  // ← not CommandBase

import frc.robot.Constants;
import frc.robot.HubActivityTracker;
import frc.robot.subsystems.*;

public class AlignAndShootCommand extends Command {

    // --------------------------------------------------------------------------
    // Static telemetry snapshot
    //
    // Exposes the latest align-and-shoot state for dashboard and tests.
    // Values are updated by whichever AlignAndShootCommand instance runs most
    // recently (teleop or auto).
    // --------------------------------------------------------------------------
    private static volatile String telemetryState = "IDLE";
    private static volatile boolean telemetryCommandActive = false;
    private static volatile boolean telemetryHasTarget = false;
    private static volatile boolean telemetryGeometryFeasible = false;
    private static volatile boolean telemetryHasShootableTarget = false;
    private static volatile double telemetryYawDeg = Double.NaN;
    private static volatile double telemetryPitchDeg = Double.NaN;
    private static volatile String telemetryLastAbortReason = "";

    // All the subsystems this command needs to control
    private final SwerveSubsystem  swerve;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem  feeder;
    private final HopperSubsystem  hopper;
    private final IntakeSubsystem  intake;

    // FIXED: Camera is injected from outside rather than created here.
    // RobotContainer creates ONE camera and passes it in.
    private final PhotonCamera camera;

    // PD controller for rotating toward the target.
    // P: how hard to turn per degree of error
    // D: damping to prevent overshooting
    private final PIDController turnPID = new PIDController(
            Constants.Vision.TURN_kP,
            0,  // no integral
            Constants.Vision.TURN_kD);

    // ---- State machine ----
    // Instead of a giant if-else chain, we track which phase we're in.
    private enum State {
        SPIN_UP,  // wait for shooter wheels to reach speed
        ALIGN,    // rotate toward vision target
        CLEAR,    // brief reverse pulse on feeder
        FEED,     // push game piece into shooter
        DONE      // command is finished
    }
    private State state;

    // Timer tracks how long we've been in each state
    private final Timer stateTimer = new Timer();
    // Last time we had a valid target with feasible shot geometry.
    private double lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
    // Cached camera result so we can use the latest unread packet API without losing continuity.
    private PhotonPipelineResult lastCameraResult = new PhotonPipelineResult();
    private double nextCameraWarningSec = 0.0;
    // Distance-based shooter speed — recalculated each cycle in ALIGN
    private double calculatedRPS = Constants.Shooter.TARGET_RPS;

    // FIXED: Separate timeout for alignment (3 seconds).
    // If the target is never found within this time, skip to DONE.
    // Previously, the at-speed timeout (1.5s) was incorrectly used for
    // alignment, meaning the robot would fire without ever aligning.
    private static final double ALIGN_TIMEOUT_SEC = 3.0;

    // --------------------------------------------------------------------------
    // Constructor
    //
    // Parameters: all subsystems + the shared camera from RobotContainer
    // --------------------------------------------------------------------------
    public AlignAndShootCommand(SwerveSubsystem swerve,
                                 ShooterSubsystem shooter,
                                 FeederSubsystem feeder,
                                 HopperSubsystem hopper,
                                 IntakeSubsystem intake,
                                 PhotonCamera camera) {
        this.swerve  = swerve;
        this.shooter = shooter;
        this.feeder  = feeder;
        this.hopper  = hopper;
        this.intake  = intake;
        this.camera  = camera;

        // This command "owns" all of these subsystems while running.
        // No other command can interrupt them.
        addRequirements(swerve, shooter, feeder, hopper, intake);

        // atSetpoint() is triggered when yaw error is within YAW_TOLERANCE_DEG
        turnPID.setTolerance(Constants.Vision.YAW_TOLERANCE_DEG);
    }

    // --------------------------------------------------------------------------
    // initialize() — called once when the command starts
    // --------------------------------------------------------------------------
    @Override
    public void initialize() {
        state = State.SPIN_UP;
        stateTimer.reset();
        stateTimer.start();
        lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
        lastCameraResult = new PhotonPipelineResult();
        nextCameraWarningSec = 0.0;
        telemetryState = State.SPIN_UP.name();
        telemetryCommandActive = true;
        telemetryHasTarget = false;
        telemetryGeometryFeasible = false;
        telemetryHasShootableTarget = false;
        telemetryYawDeg = Double.NaN;
        telemetryPitchDeg = Double.NaN;
        telemetryLastAbortReason = "";

        // 2026 REBUILT: Warn the operator if the HUB is currently inactive.
        // Shooting at an inactive HUB wastes FUEL (scores 0 points).
        // The command still proceeds — the operator may be pre-staging for the
        // next active window, or it may become active during feed.
        if (!HubActivityTracker.isOurHubActive()) {
            System.out.println("[AlignAndShoot] WARNING: Alliance HUB is currently INACTIVE!");
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", true);
        } else {
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", false);
        }

        // Start spinning shooter wheels at default warmup speed.
        // The actual speed will be recalculated based on distance once we see a HUB tag.
        calculatedRPS = Constants.Shooter.TARGET_RPS;
        shooter.setShooterVelocity(calculatedRPS);

        SmartDashboard.putString("AlignShoot/State", "SPIN_UP");
    }

    // --------------------------------------------------------------------------
    // execute() — called every 20ms while command is running
    // --------------------------------------------------------------------------
    @Override
    public void execute() {
        switch (state) {

            // ---- PHASE 1: Wait for shooter to reach warmup speed ----
            case SPIN_UP: {
                swerve.drive(0, 0, 0, false);
                // Advance once wheels are at speed OR we've waited long enough.
                // Speed will be refined in ALIGN once we have a distance estimate.
                if (shooter.isAtSpeed(calculatedRPS)
                        || stateTimer.hasElapsed(Constants.Shooter.AT_SPEED_TIMEOUT_SEC)) {
                    transitionTo(State.ALIGN);
                }
                break;
            }

            // ---- PHASE 2: Rotate to face the alliance HUB ----
            case ALIGN: {
                PhotonPipelineResult result = getLatestCameraResult();

                // Filter for only our alliance's HUB tags — ignore Trench, Tower,
                // Outpost, and opponent HUB tags. Without this filter, the robot
                // could aim at any visible AprilTag and shoot the wrong direction.
                PhotonTrackedTarget hubTarget = findBestHubTarget(result);
                telemetryHasTarget = (hubTarget != null);

                if (hubTarget == null) {
                    // No alliance HUB tag visible — stop rotating and wait
                    swerve.drive(0, 0, 0, false);
                    telemetryGeometryFeasible = false;
                    telemetryHasShootableTarget = false;
                    telemetryYawDeg = Double.NaN;
                    telemetryPitchDeg = Double.NaN;

                    // FIXED: Bail out if we can't find a target after ALIGN_TIMEOUT_SEC.
                    // In v1, the code would never escape this state without a target.
                    if (stateTimer.hasElapsed(ALIGN_TIMEOUT_SEC)) {
                        System.out.println("[AlignAndShoot] No alliance HUB tag found, aborting.");
                        telemetryLastAbortReason = "No alliance HUB tag found";
                        transitionTo(State.DONE);
                    }
                    break;
                }

                if (!isShotGeometryFeasible(hubTarget)) {
                    System.out.println("[AlignAndShoot] Shot geometry not feasible, aborting.");
                    telemetryHasShootableTarget = false;
                    telemetryLastAbortReason = "Shot geometry not feasible";
                    transitionTo(State.DONE);
                    break;
                }
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                telemetryHasShootableTarget = true;
                telemetryGeometryFeasible = true;

                // HUB target found — get the horizontal angle error in degrees
                double yawDeg = hubTarget.getYaw();
                SmartDashboard.putNumber("AlignShoot/YawError", yawDeg);
                SmartDashboard.putNumber("AlignShoot/TargetTagId", hubTarget.getFiducialId());
                telemetryYawDeg = yawDeg;

                // Recalculate shooter speed based on distance to HUB each cycle.
                // Projectile physics: v = f(distance, launch_angle, height_delta).
                double distanceM = estimateDistanceM(hubTarget.getPitch());
                calculatedRPS = ShooterSubsystem.calculateTargetRPS(distanceM);
                shooter.setShooterVelocity(calculatedRPS);
                SmartDashboard.putNumber("AlignShoot/CalculatedRPS", calculatedRPS);
                SmartDashboard.putNumber("AlignShoot/EstDistanceM", distanceM);

                // Calculate rotation correction using PD controller
                // PID input = current yaw error, setpoint = 0 (target centered)
                double rotCmd = turnPID.calculate(yawDeg, 0);
                rotCmd = MathUtil.clamp(rotCmd,
                        -Constants.Vision.MAX_ROT_CMD, Constants.Vision.MAX_ROT_CMD);

                if (turnPID.atSetpoint() && shooter.isAtSpeed(calculatedRPS)) {
                    // Aligned AND shooter at distance-based speed — advance to CLEAR.
                    swerve.drive(0, 0, 0, false);
                    transitionTo(State.CLEAR);
                } else if (turnPID.atSetpoint()) {
                    // Aligned but shooter still adjusting — hold position, wait for speed
                    swerve.drive(0, 0, 0, false);
                } else {
                    // Not aligned yet — keep rotating
                    swerve.drive(0, 0, rotCmd, false);
                }
                break;
            }

            // ---- PHASE 3: Clear the feeder path ----
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

            // ---- PHASE 4: Feed game piece into shooter ----
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
                // Nothing to do — isFinished() will return true and end() will be called
                break;
        }
    }

    // --------------------------------------------------------------------------
    // isFinished() — returns true when the command should stop
    // --------------------------------------------------------------------------
    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    // --------------------------------------------------------------------------
    // end() — called once when command finishes OR is interrupted
    // --------------------------------------------------------------------------
    @Override
    public void end(boolean interrupted) {
        // Always clean up, whether we finished normally or were interrupted
        swerve.stop();
        shooter.stop();
        feeder.stop();
        hopper.stop();
        intake.setRollerPower(0);
        telemetryState = "IDLE";
        telemetryCommandActive = false;
        telemetryHasShootableTarget = false;
        SmartDashboard.putString("AlignShoot/State", "IDLE");
        if (interrupted) {
            System.out.println("[AlignAndShoot] Command was interrupted.");
            telemetryLastAbortReason = "Interrupted";
        }
    }

    // --------------------------------------------------------------------------
    // transitionTo() — helper to switch states and reset the timer
    // --------------------------------------------------------------------------
    private void transitionTo(State newState) {
        state = newState;
        stateTimer.reset();
        stateTimer.start();
        telemetryState = newState.toString();
        SmartDashboard.putString("AlignShoot/State", newState.toString());
    }

    private boolean hasShootableTarget() {
        PhotonPipelineResult result = getLatestCameraResult();
        PhotonTrackedTarget hubTarget = findBestHubTarget(result);
        telemetryHasTarget = (hubTarget != null);

        if (hubTarget != null) {
            boolean geometryFeasible = isShotGeometryFeasible(hubTarget);
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

    // --------------------------------------------------------------------------
    // isShotGeometryFeasible()
    //
    // Checks whether the target's pitch angle is within the valid band for a
    // shot.  Pitch is a proxy for distance — too high = too close, too low =
    // too far.
    // --------------------------------------------------------------------------
    private boolean isShotGeometryFeasible(PhotonTrackedTarget target) {
        double pitchDeg = target.getPitch();
        SmartDashboard.putNumber("AlignShoot/TargetPitchDeg", pitchDeg);
        telemetryPitchDeg = pitchDeg;

        if (!Double.isFinite(pitchDeg)) {
            return false;
        }

        return pitchDeg >= Constants.Vision.MIN_SHOT_PITCH_DEG
                && pitchDeg <= Constants.Vision.MAX_SHOT_PITCH_DEG;
    }

    // --------------------------------------------------------------------------
    // estimateDistanceM()
    //
    // Rough distance estimate using the camera-to-tag vertical angle.
    //   distance = (tagHeight - cameraHeight) / tan(cameraPitch + targetPitch)
    //
    // This is approximate — it assumes a flat field and no robot tilt.
    // Use it for telemetry and future speed-by-distance lookup tables.
    // HUB tags are at 1.124 m (44.25 in) per the 2026 REBUILT game manual.
    // --------------------------------------------------------------------------
    private static final double HUB_TAG_HEIGHT_M = 1.124;

    private double estimateDistanceM(double targetPitchDeg) {
        double totalPitchRad = Constants.Vision.CAMERA_PITCH_RAD
                + Math.toRadians(targetPitchDeg);
        double heightDiff = HUB_TAG_HEIGHT_M - Constants.Vision.CAMERA_UP_M;
        double tanPitch = Math.tan(totalPitchRad);
        if (Math.abs(tanPitch) < 1e-6) return Double.NaN;
        return heightDiff / tanPitch;
    }

    // --------------------------------------------------------------------------
    // findBestHubTarget()
    //
    // Filters the camera result to only include AprilTags on our alliance's HUB.
    // Returns the best (largest area = closest) matching target, or null if none
    // of our HUB tags are visible.
    //
    // Without this filter, the robot could aim at Trench, Tower, Outpost, or
    // opponent HUB tags — and shoot in completely the wrong direction.
    // --------------------------------------------------------------------------
    private PhotonTrackedTarget findBestHubTarget(PhotonPipelineResult result) {
        if (!result.hasTargets()) return null;

        int[] hubTagIds = getAllianceHubTagIds();
        if (hubTagIds == null) return null;  // alliance unknown — can't filter

        PhotonTrackedTarget best = null;
        double bestArea = 0;
        for (var target : result.getTargets()) {
            if (isHubTag(target.getFiducialId(), hubTagIds) && target.getArea() > bestArea) {
                best = target;
                bestArea = target.getArea();
            }
        }
        return best;
    }

    // --------------------------------------------------------------------------
    // getAllianceHubTagIds()
    //
    // Returns the HUB tag ID array for our current alliance, or null if the
    // alliance is not yet known (e.g., practice mode with no FMS connection).
    // --------------------------------------------------------------------------
    private int[] getAllianceHubTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return null;
        return alliance.get() == DriverStation.Alliance.Red
                ? Constants.Vision.RED_HUB_TAG_IDS
                : Constants.Vision.BLUE_HUB_TAG_IDS;
    }

    private static boolean isHubTag(int fiducialId, int[] hubTagIds) {
        for (int id : hubTagIds) {
            if (id == fiducialId) return true;
        }
        return false;
    }

    private PhotonPipelineResult getLatestCameraResult() {
        if (!Constants.Vision.ENABLE_PHOTON) {
            return lastCameraResult;
        }

        if (!camera.isConnected()) {
            throttledCameraWarning("PhotonVision is not connected; vision targeting unavailable.");
            return lastCameraResult;
        }

        try {
            var unreadResults = camera.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                lastCameraResult = unreadResults.get(unreadResults.size() - 1);
            }
        } catch (RuntimeException e) {
            throttledCameraWarning("PhotonVision read failed: " + e.getMessage());
        }

        return lastCameraResult;
    }

    private void throttledCameraWarning(String message) {
        double now = Timer.getFPGATimestamp();
        if (now < nextCameraWarningSec) {
            return;
        }
        nextCameraWarningSec = now + Constants.Vision.VISION_WARN_INTERVAL_SEC;
        System.err.println("[AlignAndShoot] " + message);
    }

    public static String getTelemetryState() {
        return telemetryState;
    }

    public static boolean isTelemetryCommandActive() {
        return telemetryCommandActive;
    }

    public static boolean telemetryHasTarget() {
        return telemetryHasTarget;
    }

    public static boolean telemetryGeometryFeasible() {
        return telemetryGeometryFeasible;
    }

    public static boolean telemetryHasShootableTarget() {
        return telemetryHasShootableTarget;
    }

    public static double getTelemetryYawDeg() {
        return telemetryYawDeg;
    }

    public static double getTelemetryPitchDeg() {
        return telemetryPitchDeg;
    }

    public static String getTelemetryLastAbortReason() {
        return telemetryLastAbortReason;
    }
}
