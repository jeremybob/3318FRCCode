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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;  // ← not CommandBase

import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AlignAndShootCommand extends Command {

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

        // Start spinning shooter wheels immediately so they're ready sooner
        shooter.setShooterVelocity(Constants.Shooter.TARGET_RPS);

        SmartDashboard.putString("AlignShoot/State", "SPIN_UP");
    }

    // --------------------------------------------------------------------------
    // execute() — called every 20ms while command is running
    // --------------------------------------------------------------------------
    @Override
    public void execute() {
        switch (state) {

            // ---- PHASE 1: Wait for shooter to reach speed ----
            case SPIN_UP: {
                swerve.drive(0, 0, 0, false);
                // Advance once wheels are at speed OR we've waited long enough
                if (shooter.isAtSpeed(Constants.Shooter.TARGET_RPS)
                        || stateTimer.hasElapsed(Constants.Shooter.AT_SPEED_TIMEOUT_SEC)) {
                    transitionTo(State.ALIGN);
                }
                break;
            }

            // ---- PHASE 2: Rotate to face the vision target ----
            case ALIGN: {
                PhotonPipelineResult result = camera.getLatestResult();

                if (!result.hasTargets()) {
                    // No target visible — stop rotating and wait
                    swerve.drive(0, 0, 0, false);

                    // FIXED: Bail out if we can't find a target after ALIGN_TIMEOUT_SEC.
                    // In v1, the code would never escape this state without a target.
                    if (stateTimer.hasElapsed(ALIGN_TIMEOUT_SEC)) {
                        System.out.println("[AlignAndShoot] No target found, aborting.");
                        transitionTo(State.DONE);
                    }
                    break;
                }

                if (!isShotGeometryFeasible(result)) {
                    System.out.println("[AlignAndShoot] Shot geometry not feasible, aborting.");
                    transitionTo(State.DONE);
                    break;
                }
                lastValidTargetSeenSec = Timer.getFPGATimestamp();

                // Target found — get the horizontal angle error in degrees
                double yawDeg = result.getBestTarget().getYaw();
                SmartDashboard.putNumber("AlignShoot/YawError", yawDeg);

                // Calculate rotation correction using PD controller
                // PID input = current yaw error, setpoint = 0 (target centered)
                double rotCmd = turnPID.calculate(yawDeg, 0);
                rotCmd = MathUtil.clamp(rotCmd,
                        -Constants.Vision.MAX_ROT_CMD, Constants.Vision.MAX_ROT_CMD);

                if (turnPID.atSetpoint()) {
                    // Aligned! Stop rotating and advance to CLEAR phase.
                    swerve.drive(0, 0, 0, false);
                    transitionTo(State.CLEAR);
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
        SmartDashboard.putString("AlignShoot/State", "IDLE");
        if (interrupted) {
            System.out.println("[AlignAndShoot] Command was interrupted.");
        }
    }

    // --------------------------------------------------------------------------
    // transitionTo() — helper to switch states and reset the timer
    // --------------------------------------------------------------------------
    private void transitionTo(State newState) {
        state = newState;
        stateTimer.reset();
        stateTimer.start();
        SmartDashboard.putString("AlignShoot/State", newState.toString());
    }

    private boolean hasShootableTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            if (isShotGeometryFeasible(result)) {
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                return true;
            }
            return false;
        }

        return hasRecentValidTarget();
    }

    private boolean hasRecentValidTarget() {
        return Timer.getFPGATimestamp() - lastValidTargetSeenSec
                <= Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }

    private boolean isShotGeometryFeasible(PhotonPipelineResult result) {
        double pitchDeg = result.getBestTarget().getPitch();
        SmartDashboard.putNumber("AlignShoot/TargetPitchDeg", pitchDeg);

        if (!Double.isFinite(pitchDeg)) {
            return false;
        }

        return pitchDeg >= Constants.Vision.MIN_SHOT_PITCH_DEG
                && pitchDeg <= Constants.Vision.MAX_SHOT_PITCH_DEG;
    }
}
