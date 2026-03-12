// ============================================================================
// FILE: src/main/java/frc/robot/commands/AlignAndShootCommand.java
//
// PURPOSE: Auto-aligns to the vision target in place, then fires while
// stationary.
//
// SEQUENCE:
//   1. ALIGN    - Spin shooter and rotate in place until yaw + RPM are ready
//   2. CLEAR    - Keep aiming in place while the feeder clears the note
//   3. FEED     - Continue aiming in place while feeding until interrupted
//   4. DONE     - Command finishes, everything stops
// ============================================================================
package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.HubActivityTracker;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionResult;

public class AlignAndShootCommand extends Command {

    public record TelemetrySnapshot(
            String state,
            boolean commandActive,
            boolean hasTarget,
            boolean geometryFeasible,
            boolean hasShootableTarget,
            double yawDeg,
            double aimErrorDeg,
            double leadYawDeg,
            double pitchDeg,
            double targetRps,
            double radialVelocityMps,
            double lateralVelocityMps,
            double commandedXVelocityMps,
            double commandedYVelocityMps,
            double activeTranslationCapMps,
            double timeOfFlightSec,
            boolean feedGateReady,
            String lastAbortReason) {}

    private static final TelemetrySnapshot IDLE_SNAPSHOT = new TelemetrySnapshot(
            "IDLE",
            false,
            false,
            false,
            false,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            false,
            "");

    private static volatile TelemetrySnapshot telemetrySnapshot = IDLE_SNAPSHOT;

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;
    private final IntakeSubsystem intake;
    private final AtomicReference<VisionResult> visionRef;
    private final boolean continuousFeedUntilInterrupted;
    private final PIDController turnPID = new PIDController(
            Constants.Vision.TURN_kP,
            0.0,
            Constants.Vision.TURN_kD);

    private enum State { ALIGN, CLEAR, FEED, DONE }

    private static final double ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0;

    private State state;
    private final Timer stateTimer = new Timer();
    private final Timer alignOverallTimer = new Timer();
    private final Timer feedGateTimer = new Timer();
    private final Timer continuousLossTimer = new Timer();
    private final Timer alignmentLockTimer = new Timer();
    private final Timer alignTargetLossTimer = new Timer();
    private double searchRotationSign = 1.0;
    private boolean seenTargetThisRun = false;
    private double filteredYawDeg = Double.NaN;
    private boolean alignmentLocked = false;

    private String workState = "IDLE";
    private boolean workCommandActive = false;
    private boolean workHasTarget = false;
    private boolean workGeometryFeasible = false;
    private boolean workHasShootableTarget = false;
    private double workYawDeg = Double.NaN;
    private double workAimErrorDeg = Double.NaN;
    private double workLeadYawDeg = Double.NaN;
    private double workPitchDeg = Double.NaN;
    private double workTargetRps = Double.NaN;
    private double workRadialVelocityMps = Double.NaN;
    private double workLateralVelocityMps = Double.NaN;
    private double workCommandedXVelocityMps = Double.NaN;
    private double workCommandedYVelocityMps = Double.NaN;
    private double workActiveTranslationCapMps = Double.NaN;
    private double workTimeOfFlightSec = Double.NaN;
    private boolean workFeedGateReady = false;
    private String workLastAbortReason = "";

    public AlignAndShootCommand(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            AtomicReference<VisionResult> visionRef,
            boolean continuousFeedUntilInterrupted) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.visionRef = visionRef;
        this.continuousFeedUntilInterrupted = continuousFeedUntilInterrupted;

        addRequirements(swerve, shooter, feeder, hopper, intake);
        turnPID.setTolerance(Constants.Vision.YAW_TOLERANCE_DEG);
    }

    @Override
    public void initialize() {
        state = State.ALIGN;
        stateTimer.restart();
        alignOverallTimer.restart();
        turnPID.reset();
        resetFeedGateTimer();
        resetContinuousLossTimer();
        resetAlignmentLockTimer();
        resetAlignTargetLossTimer();
        searchRotationSign = 1.0;
        seenTargetThisRun = false;
        filteredYawDeg = Double.NaN;
        alignmentLocked = false;

        workState = State.ALIGN.name();
        workCommandActive = true;
        workHasTarget = false;
        workGeometryFeasible = false;
        workHasShootableTarget = false;
        workYawDeg = Double.NaN;
        workAimErrorDeg = Double.NaN;
        workLeadYawDeg = 0.0;
        workPitchDeg = Double.NaN;
        workTargetRps = Constants.Shooter.TARGET_RPS;
        workRadialVelocityMps = 0.0;
        workLateralVelocityMps = 0.0;
        workCommandedXVelocityMps = 0.0;
        workCommandedYVelocityMps = 0.0;
        workActiveTranslationCapMps = 0.0;
        workTimeOfFlightSec = Double.NaN;
        workFeedGateReady = false;
        workLastAbortReason = "";

        if (!HubActivityTracker.isOurHubActive()) {
            double secsToShift = HubActivityTracker.secondsUntilNextShiftChange();
            System.out.println("[AlignAndShoot] WARNING: Alliance HUB is INACTIVE! "
                    + String.format("%.1fs", secsToShift) + " until next shift change.");
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", true);
        } else {
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", false);
        }

        shooter.stop();
        SmartDashboard.putString("AlignShoot/State", "ALIGN");
        updateTelemetry();
        publishTelemetry();
    }

    @Override
    public void execute() {
        switch (state) {
            case ALIGN -> executeAlign();
            case CLEAR -> executeClear();
            case FEED -> executeFeed();
            case DONE -> { }
        }

        updateTelemetry();
        publishTelemetry();
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
        workFeedGateReady = false;
        workTargetRps = Double.NaN;
        workTimeOfFlightSec = Double.NaN;
        alignmentLocked = false;
        resetAlignmentLockTimer();
        updateTelemetry();
        publishTelemetry();
    }

    private void executeAlign() {
        VisionResult result = visionRef.get();
        boolean hasFreshTarget = isResultFresh(result);
        workHasTarget = hasFreshTarget;

        if (!hasFreshTarget) {
            shooter.stop();
            workGeometryFeasible = false;
            workHasShootableTarget = false;
            workYawDeg = Double.NaN;
            workAimErrorDeg = Double.NaN;
            workLeadYawDeg = 0.0;
            workPitchDeg = Double.NaN;
            workRadialVelocityMps = 0.0;
            workLateralVelocityMps = 0.0;
            workCommandedXVelocityMps = 0.0;
            workCommandedYVelocityMps = 0.0;
            workActiveTranslationCapMps = 0.0;
            workTimeOfFlightSec = Double.NaN;
            workFeedGateReady = false;
            resetFeedGateTimer();
            filteredYawDeg = Double.NaN;
            alignmentLocked = false;
            resetAlignmentLockTimer();

            if (seenTargetThisRun) {
                if (shouldResumeSearchAfterTargetLoss()) {
                    workState = "RESEEK_TARGET";
                    SmartDashboard.putString("AlignShoot/State", "RESEEK_TARGET");
                    driveSearchPattern();
                } else {
                    workState = "WAIT_TARGET";
                    swerve.drive(0, 0, 0, false);
                    SmartDashboard.putString("AlignShoot/State", "WAIT_TARGET");
                }
            } else {
                workState = State.ALIGN.name();
                driveSearchPattern();
            }

            if (alignOverallTimer.hasElapsed(ALIGN_CONVERGENCE_TIMEOUT_SEC)) {
                abort("No alliance HUB tag found");
            }
            return;
        }

        seenTargetThisRun = true;
        resetAlignTargetLossTimer();
        workState = State.ALIGN.name();
        SmartDashboard.putString("AlignShoot/State", "ALIGN");

        workPitchDeg = result.pitchDeg();
        workYawDeg = result.yawDeg();
        double filteredYawDeg = filterYaw(result.yawDeg());
        if (!isShotPitchFeasible(result.pitchDeg())) {
            shooter.stop();
            workHasShootableTarget = false;
            workFeedGateReady = false;
            resetFeedGateTimer();
            abort("Shot pitch out of range");
            return;
        }

        if (!isWithinTrackingYaw(filteredYawDeg)) {
            alignmentLocked = false;
            resetAlignmentLockTimer();
            shooter.stop();
            workGeometryFeasible = false;
            workHasShootableTarget = false;
            workAimErrorDeg = filteredYawDeg;
            workLeadYawDeg = 0.0;
            workRadialVelocityMps = 0.0;
            workLateralVelocityMps = 0.0;
            workTimeOfFlightSec = Double.NaN;
            workFeedGateReady = false;
            resetFeedGateTimer();
            updateSearchDirectionFromYaw(filteredYawDeg);
            driveAcquireTarget(filteredYawDeg);
            if (alignOverallTimer.hasElapsed(ALIGN_CONVERGENCE_TIMEOUT_SEC)) {
                abort("Target yaw acquisition timeout");
            }
            return;
        }

        updateAlignmentLock(filteredYawDeg);
        ShotTracking tracking = buildStationaryTracking(result, filteredYawDeg);
        if (!tracking.solution().feasible()) {
            shooter.stop();
            workHasShootableTarget = false;
            workFeedGateReady = false;
            resetFeedGateTimer();
            abort("Shot solution invalid");
            return;
        }

        applyTracking(tracking);
        driveTracking(tracking);

        if (tracking.feedGateReady()) {
            transitionTo(State.CLEAR);
        } else if (alignOverallTimer.hasElapsed(ALIGN_CONVERGENCE_TIMEOUT_SEC)) {
            abort("Alignment convergence timeout");
        }
    }

    private void executeClear() {
        VisionResult result = visionRef.get();
        if (!hasShootableTarget(result)) {
            if (continuousFeedUntilInterrupted) {
                if (shouldHoldContinuousFeed()) {
                    holdStationaryWhileReacquiring();
                    return;
                }
                stopFeedPath();
                shooter.stop();
                transitionTo(State.ALIGN);
            } else {
                abort("Vision lost before feed");
            }
            return;
        }

        double filteredYawDeg = filterYaw(result.yawDeg());
        if (!maintainLockedAlignment(filteredYawDeg)) {
            if (continuousFeedUntilInterrupted) {
                if (shouldHoldContinuousFeed()) {
                    holdStationaryWhileReacquiring();
                    return;
                }
                stopFeedPath();
                shooter.stop();
                transitionTo(State.ALIGN);
            } else {
                abort("Alignment lock lost before feed");
            }
            return;
        }

        ShotTracking tracking = buildStationaryTracking(result, filteredYawDeg);
        if (!tracking.solution().feasible()) {
            if (continuousFeedUntilInterrupted) {
                stopFeedPath();
                shooter.stop();
                alignmentLocked = false;
                resetAlignmentLockTimer();
                transitionTo(State.ALIGN);
            } else {
                abort("Feed gate lost before feed");
            }
            return;
        }

        resetContinuousLossTimer();
        applyTracking(tracking);
        driveTracking(tracking);
        feeder.setPower(Constants.Shooter.CLEAR_POWER);

        if (stateTimer.hasElapsed(Constants.Shooter.CLEAR_TIME_SEC)) {
            feeder.stop();
            transitionTo(State.FEED);
        }
    }

    private void executeFeed() {
        VisionResult result = visionRef.get();
        if (!hasShootableTarget(result)) {
            if (continuousFeedUntilInterrupted) {
                if (shouldHoldContinuousFeed()) {
                    holdStationaryWhileReacquiring();
                    return;
                }
                stopFeedPath();
                shooter.stop();
                transitionTo(State.ALIGN);
            } else {
                abort("Vision lost during feed");
            }
            return;
        }

        double filteredYawDeg = filterYaw(result.yawDeg());
        if (!maintainLockedAlignment(filteredYawDeg)) {
            if (continuousFeedUntilInterrupted) {
                if (shouldHoldContinuousFeed()) {
                    holdStationaryWhileReacquiring();
                    return;
                }
                stopFeedPath();
                shooter.stop();
                transitionTo(State.ALIGN);
            } else {
                abort("Alignment lock lost during feed");
            }
            return;
        }

        ShotTracking tracking = buildStationaryTracking(result, filteredYawDeg);
        if (!tracking.solution().feasible()) {
            if (continuousFeedUntilInterrupted) {
                stopFeedPath();
                shooter.stop();
                alignmentLocked = false;
                resetAlignmentLockTimer();
                transitionTo(State.ALIGN);
            } else {
                abort("Feed gate lost during feed");
            }
            return;
        }

        resetContinuousLossTimer();
        applyTracking(tracking);
        driveTracking(tracking);
        feeder.setPower(Constants.Shooter.FEED_POWER);
        hopper.setPower(Constants.Shooter.FEED_POWER);
        intake.setRollerPower(Constants.Shooter.FEED_POWER);

        if (!continuousFeedUntilInterrupted
                && stateTimer.hasElapsed(Constants.Shooter.FEED_TIME_SEC)) {
            transitionTo(State.DONE);
        }
    }

    private void transitionTo(State newState) {
        state = newState;
        stateTimer.restart();
        resetContinuousLossTimer();
        if (newState == State.ALIGN || newState == State.DONE) {
            alignmentLocked = false;
            resetAlignmentLockTimer();
        }
        if (newState != State.ALIGN) {
            resetFeedGateTimer();
        }
        workState = newState.toString();
        SmartDashboard.putString("AlignShoot/State", newState.toString());
    }

    private void abort(String reason) {
        System.out.println("[AlignAndShoot] " + reason + ", aborting.");
        workLastAbortReason = reason;
        transitionTo(State.DONE);
    }

    private void updateTelemetry() {
        telemetrySnapshot = new TelemetrySnapshot(
                workState,
                workCommandActive,
                workHasTarget,
                workGeometryFeasible,
                workHasShootableTarget,
                workYawDeg,
                workAimErrorDeg,
                workLeadYawDeg,
                workPitchDeg,
                workTargetRps,
                workRadialVelocityMps,
                workLateralVelocityMps,
                workCommandedXVelocityMps,
                workCommandedYVelocityMps,
                workActiveTranslationCapMps,
                workTimeOfFlightSec,
                workFeedGateReady,
                workLastAbortReason);
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber("AlignShoot/YawError", workYawDeg);
        SmartDashboard.putNumber("AlignShoot/AimErrorDeg", workAimErrorDeg);
        SmartDashboard.putNumber("AlignShoot/LeadYawDeg", workLeadYawDeg);
        SmartDashboard.putNumber("AlignShoot/TargetPitchDeg", workPitchDeg);
        SmartDashboard.putNumber("AlignShoot/CalculatedRPS", workTargetRps);
        SmartDashboard.putNumber("AlignShoot/RadialVelocityMps", workRadialVelocityMps);
        SmartDashboard.putNumber("AlignShoot/LateralVelocityMps", workLateralVelocityMps);
        SmartDashboard.putNumber("AlignShoot/CommandedXVelocityMps", workCommandedXVelocityMps);
        SmartDashboard.putNumber("AlignShoot/CommandedYVelocityMps", workCommandedYVelocityMps);
        SmartDashboard.putNumber("AlignShoot/ActiveTranslationCapMps", workActiveTranslationCapMps);
        SmartDashboard.putNumber("AlignShoot/TimeOfFlightSec", workTimeOfFlightSec);
        SmartDashboard.putBoolean("AlignShoot/FeedGateReady", workFeedGateReady);
    }

    private ShotTracking buildStationaryTracking(VisionResult result, double filteredYawDeg) {
        double rawYawDeg = result.yawDeg();
        double distanceM = estimateDistanceM(result);
        double pitchDeg = result.pitchDeg();
        ShooterSubsystem.ShotSolution solution = ShooterSubsystem.calculateMovingShotSolution(distanceM, 0.0, 0.0);

        boolean holdingAlignment = shouldHoldAlignment(filteredYawDeg);
        double rotCmd = 0.0;
        if (!holdingAlignment) {
            double pidOutput = turnPID.calculate(filteredYawDeg, 0.0);
            rotCmd = MathUtil.clamp(
                    pidOutput,
                    -Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS,
                    Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS);
        }

        boolean feedGateReady = solution.feasible()
                && alignmentLocked
                && isShooterReady(solution.targetRps());

        return new ShotTracking(
                rawYawDeg,
                filteredYawDeg,
                0.0,
                pitchDeg,
                distanceM,
                0.0,
                0.0,
                new ChassisSpeeds(0.0, 0.0, 0.0),
                0.0,
                solution,
                rotCmd,
                feedGateReady);
    }

    private void applyTracking(ShotTracking tracking) {
        workHasTarget = true;
        workGeometryFeasible = true;
        workHasShootableTarget = true;
        workYawDeg = tracking.rawYawDeg();
        workAimErrorDeg = tracking.aimErrorDeg();
        workLeadYawDeg = tracking.leadYawDeg();
        workPitchDeg = tracking.pitchDeg();
        workRadialVelocityMps = tracking.radialVelocityMps();
        workLateralVelocityMps = tracking.lateralVelocityMps();
        workCommandedXVelocityMps = tracking.translationCmd().vxMetersPerSecond;
        workCommandedYVelocityMps = tracking.translationCmd().vyMetersPerSecond;
        workActiveTranslationCapMps = tracking.translationCapMps();
        workFeedGateReady = tracking.feedGateReady();
        workTimeOfFlightSec = tracking.solution().timeOfFlightSec();
        workTargetRps = tracking.solution().targetRps();
        shooter.setShooterVelocity(workTargetRps);

        SmartDashboard.putNumber("AlignShoot/EstDistanceM", tracking.distanceM());
    }

    private void driveTracking(ShotTracking tracking) {
        swerve.driveRobotRelative(new ChassisSpeeds(
                tracking.translationCmd().vxMetersPerSecond,
                tracking.translationCmd().vyMetersPerSecond,
                tracking.rotCmdRadPerSec()));
    }

    private void driveSearchPattern() {
        workCommandedXVelocityMps = 0.0;
        workCommandedYVelocityMps = 0.0;
        workActiveTranslationCapMps = 0.0;
        swerve.driveRobotRelative(new ChassisSpeeds(
                0.0,
                0.0,
                searchRotationSign * Constants.AlignShoot.SEARCH_OMEGA_RADPS));
    }

    private void driveAcquireTarget(double yawDeg) {
        double pidOutput = turnPID.calculate(yawDeg, 0.0);
        double rotCmd = turnPID.atSetpoint()
                ? 0.0
                : MathUtil.clamp(
                        pidOutput,
                        -Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS,
                        Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS);

        workCommandedXVelocityMps = 0.0;
        workCommandedYVelocityMps = 0.0;
        workActiveTranslationCapMps = 0.0;
        swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, rotCmd));
    }

    private double estimateDistanceM(VisionResult result) {
        double distanceM = result.estimateDistanceM(
                Constants.Vision.TAG_HEIGHT_M,
                Constants.Vision.FOCAL_LENGTH_PIXELS);
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            double pitchFallbackM = estimateDistanceFromPitch(result.pitchDeg());
            System.out.println("[AlignAndShoot] Primary distance invalid ("
                    + String.format("%.2f", distanceM)
                    + "m), using pitch-based fallback: "
                    + String.format("%.2f", pitchFallbackM) + "m");
            distanceM = pitchFallbackM;
        }
        return distanceM;
    }

    private boolean isResultFresh(VisionResult result) {
        if (result == null || result.tagId() < 0) {
            return false;
        }
        double ageSec = Timer.getFPGATimestamp() - result.timestampSec();
        return ageSec < Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }

    private boolean hasShootableTarget(VisionResult result) {
        boolean hasTarget = isResultFresh(result);
        workHasTarget = hasTarget;
        if (!hasTarget) {
            workGeometryFeasible = false;
            workHasShootableTarget = false;
            return false;
        }

        boolean geometryFeasible = isShotGeometryFeasible(result);
        workGeometryFeasible = geometryFeasible;
        if (!geometryFeasible) {
            workHasShootableTarget = false;
            return false;
        }

        workHasShootableTarget = true;
        return true;
    }

    private boolean isShotGeometryFeasible(VisionResult result) {
        double pitchDeg = result.pitchDeg();
        double yawDeg = result.yawDeg();
        workPitchDeg = pitchDeg;

        SmartDashboard.putNumber("AlignShoot/TargetTagId", result.tagId());
        SmartDashboard.putNumber("AlignShoot/YawGeometryCheck", yawDeg);

        return isShotPitchFeasible(pitchDeg) && isWithinTrackingYaw(yawDeg);
    }

    private boolean isShooterReady(double targetRps) {
        double leftError = Math.abs(Math.abs(shooter.getLeftRPS()) - targetRps);
        double rightError = Math.abs(Math.abs(shooter.getRightRPS()) - targetRps);
        return leftError <= Constants.AlignShoot.RPS_TOLERANCE_RPS
                && rightError <= Constants.AlignShoot.RPS_TOLERANCE_RPS;
    }

    private void resetFeedGateTimer() {
        feedGateTimer.stop();
        feedGateTimer.reset();
    }

    private void resetContinuousLossTimer() {
        continuousLossTimer.stop();
        continuousLossTimer.reset();
    }

    private void resetAlignmentLockTimer() {
        alignmentLockTimer.stop();
        alignmentLockTimer.reset();
    }

    private void resetAlignTargetLossTimer() {
        alignTargetLossTimer.stop();
        alignTargetLossTimer.reset();
    }

    private boolean shouldResumeSearchAfterTargetLoss() {
        if (!alignTargetLossTimer.isRunning()) {
            alignTargetLossTimer.restart();
            return false;
        }
        return alignTargetLossTimer.hasElapsed(
                Constants.AlignShoot.TARGET_LOSS_WAIT_BEFORE_RESEEK_SEC);
    }

    private void stopFeedPath() {
        feeder.stop();
        hopper.stop();
        intake.setRollerPower(0);
    }

    private void updateAlignmentLock(double filteredYawDeg) {
        if (!Double.isFinite(filteredYawDeg)) {
            alignmentLocked = false;
            resetAlignmentLockTimer();
            return;
        }
        if (alignmentLocked) {
            if (Math.abs(filteredYawDeg) > Constants.Vision.YAW_BREAK_TOLERANCE_DEG) {
                alignmentLocked = false;
                resetAlignmentLockTimer();
            }
            return;
        }
        if (shouldHoldAlignment(filteredYawDeg)) {
            if (!alignmentLockTimer.isRunning()) {
                alignmentLockTimer.restart();
            }
            if (alignmentLockTimer.hasElapsed(Constants.AlignShoot.SETTLE_TIME_SEC)) {
                alignmentLocked = true;
            }
        } else {
            resetAlignmentLockTimer();
        }
    }

    private boolean maintainLockedAlignment(double filteredYawDeg) {
        if (!alignmentLocked) {
            return false;
        }
        if (Double.isFinite(filteredYawDeg)
                && Math.abs(filteredYawDeg) <= Constants.Vision.YAW_BREAK_TOLERANCE_DEG) {
            return true;
        }
        alignmentLocked = false;
        resetAlignmentLockTimer();
        return false;
    }

    private boolean shouldHoldAlignment(double filteredYawDeg) {
        if (!Double.isFinite(filteredYawDeg)) {
            return false;
        }
        double absYawDeg = Math.abs(filteredYawDeg);
        return absYawDeg <= Constants.AlignShoot.YAW_TOLERANCE_DEG
                || (alignmentLockTimer.isRunning()
                        && absYawDeg <= Constants.Vision.YAW_BREAK_TOLERANCE_DEG)
                || alignmentLocked;
    }

    private boolean shouldHoldContinuousFeed() {
        if (!continuousLossTimer.isRunning()) {
            continuousLossTimer.restart();
        }
        return !continuousLossTimer.hasElapsed(Constants.AlignShoot.CONTINUOUS_FEED_REACQUIRE_SEC);
    }

    private void holdStationaryWhileReacquiring() {
        workFeedGateReady = false;
        if (state == State.CLEAR) {
            feeder.setPower(Constants.Shooter.CLEAR_POWER);
        } else if (state == State.FEED) {
            feeder.setPower(Constants.Shooter.FEED_POWER);
            hopper.setPower(Constants.Shooter.FEED_POWER);
            intake.setRollerPower(Constants.Shooter.FEED_POWER);
        }
        swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private double filterYaw(double rawYawDeg) {
        if (!Double.isFinite(rawYawDeg)) {
            filteredYawDeg = Double.NaN;
            return Double.NaN;
        }
        if (!Double.isFinite(filteredYawDeg)) {
            filteredYawDeg = rawYawDeg;
        } else {
            filteredYawDeg = Constants.Vision.YAW_FILTER_ALPHA * filteredYawDeg
                    + (1.0 - Constants.Vision.YAW_FILTER_ALPHA) * rawYawDeg;
        }
        return filteredYawDeg;
    }

    private void updateSearchDirectionFromYaw(double yawDeg) {
        double desiredSign = -Math.signum(yawDeg);
        if (desiredSign != 0.0) {
            searchRotationSign = desiredSign;
        }
    }

    private static boolean isWithinTrackingYaw(double yawDeg) {
        return Double.isFinite(yawDeg)
                && Math.abs(yawDeg) <= Constants.AlignShoot.ACQUIRE_YAW_MAX_DEG;
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

    public static TelemetrySnapshot getTelemetrySnapshot() { return telemetrySnapshot; }

    public static String getTelemetryState() { return telemetrySnapshot.state(); }
    public static boolean isTelemetryCommandActive() { return telemetrySnapshot.commandActive(); }
    public static boolean telemetryHasTarget() { return telemetrySnapshot.hasTarget(); }
    public static boolean telemetryGeometryFeasible() { return telemetrySnapshot.geometryFeasible(); }
    public static boolean telemetryHasShootableTarget() { return telemetrySnapshot.hasShootableTarget(); }
    public static double getTelemetryYawDeg() { return telemetrySnapshot.yawDeg(); }
    public static double getTelemetryAimErrorDeg() { return telemetrySnapshot.aimErrorDeg(); }
    public static double getTelemetryLeadYawDeg() { return telemetrySnapshot.leadYawDeg(); }
    public static double getTelemetryPitchDeg() { return telemetrySnapshot.pitchDeg(); }
    public static double getTelemetryTargetRps() { return telemetrySnapshot.targetRps(); }
    public static double getTelemetryRadialVelocityMps() { return telemetrySnapshot.radialVelocityMps(); }
    public static double getTelemetryLateralVelocityMps() { return telemetrySnapshot.lateralVelocityMps(); }
    public static double getTelemetryCommandedXVelocityMps() {
        return telemetrySnapshot.commandedXVelocityMps();
    }
    public static double getTelemetryCommandedYVelocityMps() {
        return telemetrySnapshot.commandedYVelocityMps();
    }
    public static double getTelemetryActiveTranslationCapMps() {
        return telemetrySnapshot.activeTranslationCapMps();
    }
    public static double getTelemetryTimeOfFlightSec() { return telemetrySnapshot.timeOfFlightSec(); }
    public static boolean telemetryFeedGateReady() { return telemetrySnapshot.feedGateReady(); }
    public static String getTelemetryLastAbortReason() { return telemetrySnapshot.lastAbortReason(); }

    static boolean isShotPitchFeasible(double pitchDeg) {
        return Double.isFinite(pitchDeg)
                && pitchDeg >= Constants.Vision.MIN_SHOT_PITCH_DEG
                && pitchDeg <= Constants.Vision.MAX_SHOT_PITCH_DEG;
    }

    private record ShotTracking(
            double rawYawDeg,
            double aimErrorDeg,
            double leadYawDeg,
            double pitchDeg,
            double distanceM,
            double radialVelocityMps,
            double lateralVelocityMps,
            ChassisSpeeds translationCmd,
            double translationCapMps,
            ShooterSubsystem.ShotSolution solution,
            double rotCmdRadPerSec,
            boolean feedGateReady) {}
}
