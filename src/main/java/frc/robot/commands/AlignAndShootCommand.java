// ============================================================================
// FILE: src/main/java/frc/robot/commands/AlignAndShootCommand.java
//
// PURPOSE: Auto-aligns to the HUB using pose-based heading, then fires while
// stationary. Vision tags correct the robot's pose estimate; alignment is
// calculated from the corrected pose to the known HUB center — not from
// raw camera tag yaw.
//
// SEQUENCE:
//   1. ALIGN    - Spin shooter and rotate in place until heading + RPM are ready
//   2. CLEAR    - Optional short clear pulse before feed (non-continuous mode)
//   3. FEED     - Continue aiming in place while feeding (or skip CLEAR in
//                 continuous hold-to-shoot mode)
//   4. DONE     - Command finishes, everything stops
// ============================================================================
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.HubActivityTracker;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AlignmentCaptureUtil;


public class AlignAndShootCommand extends Command {

    public record TelemetrySnapshot(
            String state,
            boolean commandActive,
            boolean hasTarget,
            boolean geometryFeasible,
            boolean hasShootableTarget,
            double headingErrorDeg,
            double aimErrorDeg,
            double distanceM,
            double targetRps,
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
            false,
            "");

    private static volatile TelemetrySnapshot telemetrySnapshot = IDLE_SNAPSHOT;

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;
    private final IntakeSubsystem intake;
    private final boolean continuousFeedUntilInterrupted;
    private final PIDController turnPID = new PIDController(
            Constants.AlignShoot.TURN_kP,
            0.0,
            Constants.AlignShoot.TURN_kD);

    private enum State { ALIGN, CLEAR, FEED, DONE }

    private static final double ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0;

    private State state;
    private final Timer stateTimer = new Timer();
    private final Timer alignOverallTimer = new Timer();
    private final Timer feedGateTimer = new Timer();
    private final Timer continuousLossTimer = new Timer();
    private final Timer alignmentLockTimer = new Timer();
    private final Timer alignmentBreakTimer = new Timer();
    private final Timer alignTargetLossTimer = new Timer();
    private double searchRotationSign = 1.0;
    private boolean seenTargetThisRun = false;
    private double filteredHeadingErrorDeg = Double.NaN;
    private double previousAimErrorDeg = Double.NaN;
    private boolean alignmentLocked = false;
    private boolean hadAlignmentLockThisRun = false;

    private String workState = "IDLE";
    private boolean workCommandActive = false;
    private boolean workHasTarget = false;
    private boolean workGeometryFeasible = false;
    private boolean workHasShootableTarget = false;
    private double workHeadingErrorDeg = Double.NaN;
    private double workAimErrorDeg = Double.NaN;
    private double workDistanceM = Double.NaN;
    private double workTargetRps = Double.NaN;
    private boolean workFeedGateReady = false;
    private String workLastAbortReason = "";

    public AlignAndShootCommand(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            FeederSubsystem feeder,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            boolean continuousFeedUntilInterrupted) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.continuousFeedUntilInterrupted = continuousFeedUntilInterrupted;

        addRequirements(swerve, shooter, hood, feeder, hopper, intake);
        turnPID.setTolerance(Constants.AlignShoot.YAW_TOLERANCE_DEG);
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
        resetAlignmentBreakTimer();
        resetAlignTargetLossTimer();
        searchRotationSign = 1.0;
        seenTargetThisRun = false;
        filteredHeadingErrorDeg = Double.NaN;
        previousAimErrorDeg = Double.NaN;
        alignmentLocked = false;
        hadAlignmentLockThisRun = false;

        workState = State.ALIGN.name();
        workCommandActive = true;
        workHasTarget = false;
        workGeometryFeasible = false;
        workHasShootableTarget = false;
        workHeadingErrorDeg = Double.NaN;
        workAimErrorDeg = Double.NaN;
        workDistanceM = Double.NaN;
        workTargetRps = Constants.Shooter.TARGET_RPS;
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
        alignmentLocked = false;
        hadAlignmentLockThisRun = false;
        resetAlignmentLockTimer();
        resetAlignmentBreakTimer();
        updateTelemetry();
        publishTelemetry();
    }

    // =========================================================================
    // State implementations
    // =========================================================================

    private void executeAlign() {
        Translation2d hubCenter = RobotContainer.getAllianceHubCenter();
        boolean hasVision = swerve.isVisionActive();
        double distanceM = swerve.getDistanceTo(hubCenter);
        double headingErrorDeg = swerve.getHeadingErrorDegTo(hubCenter);
        workHasTarget = hasVision;

        if (!hasVision) {
            workGeometryFeasible = false;
            workHasShootableTarget = false;
            workHeadingErrorDeg = Double.NaN;
            workAimErrorDeg = Double.NaN;
            workDistanceM = Double.NaN;
            workFeedGateReady = false;
            resetFeedGateTimer();
            filteredHeadingErrorDeg = Double.NaN;
            previousAimErrorDeg = Double.NaN;
            alignmentLocked = false;
            resetAlignmentLockTimer();

            if (seenTargetThisRun) {
                boolean holdHeadingDuringLoss = hadAlignmentLockThisRun || continuousFeedUntilInterrupted;
                if (!holdHeadingDuringLoss && shouldResumeSearchAfterTargetLoss()) {
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

            if (hasAlignConvergenceTimedOut()) {
                abort("No vision-corrected pose available");
            }
            return;
        }

        seenTargetThisRun = true;
        resetAlignTargetLossTimer();
        workState = State.ALIGN.name();
        SmartDashboard.putString("AlignShoot/State", "ALIGN");

        workDistanceM = distanceM;
        workHeadingErrorDeg = headingErrorDeg;
        double filteredError = filterHeadingError(headingErrorDeg);
        double aimErrorDeg = filteredError;

        if (!isShotDistanceFeasible(distanceM)) {
            shooter.stop();
            workHasShootableTarget = false;
            workFeedGateReady = false;
            resetFeedGateTimer();
            abort("Shot distance out of range");
            return;
        }

        // Pre-spin the shooter and set hood angle as soon as we have a valid
        // target so spin-up and hood travel happen in parallel with heading
        // alignment instead of after it.
        double targetRps = ShooterSubsystem.calculateTargetRPS(distanceM);
        if (Double.isFinite(targetRps) && targetRps > 0.0) {
            shooter.setShooterVelocity(targetRps);
            hood.setAngle(HoodSubsystem.calculateTargetAngle(distanceM));
            workTargetRps = targetRps;
        }

        if (!isWithinTrackingHeading(aimErrorDeg)) {
            alignmentLocked = false;
            resetAlignmentLockTimer();
            workGeometryFeasible = false;
            workHasShootableTarget = false;
            workAimErrorDeg = aimErrorDeg;
            workFeedGateReady = false;
            resetFeedGateTimer();
            updateSearchDirectionFromError(aimErrorDeg);
            driveTowardsHub(filteredError);
            if (hasAlignConvergenceTimedOut()) {
                abort("Heading acquisition timeout");
            }
            return;
        }

        updateAlignmentLock(aimErrorDeg);
        ShotTracking tracking = buildStationaryTracking(filteredError, distanceM);
        if (!tracking.feasible()) {
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
            transitionTo(continuousFeedUntilInterrupted ? State.FEED : State.CLEAR);
        } else if (hasAlignConvergenceTimedOut()) {
            abort("Alignment convergence timeout");
        }
    }

    private void executeClear() {
        Translation2d hubCenter = RobotContainer.getAllianceHubCenter();
        if (!hasShootableTarget(hubCenter)) {
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

        double distanceM = swerve.getDistanceTo(hubCenter);
        double headingErrorDeg = swerve.getHeadingErrorDegTo(hubCenter);
        double filteredError = filterHeadingError(headingErrorDeg);
        double aimErrorDeg = filteredError;

        if (!maintainLockedAlignment(aimErrorDeg)) {
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

        ShotTracking tracking = buildStationaryTracking(filteredError, distanceM);
        if (!tracking.feasible()) {
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
        Translation2d hubCenter = RobotContainer.getAllianceHubCenter();
        if (!hasShootableTarget(hubCenter)) {
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

        double distanceM = swerve.getDistanceTo(hubCenter);
        double headingErrorDeg = swerve.getHeadingErrorDegTo(hubCenter);
        double filteredError = filterHeadingError(headingErrorDeg);
        double aimErrorDeg = filteredError;

        if (!maintainLockedAlignment(aimErrorDeg)) {
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

        ShotTracking tracking = buildStationaryTracking(filteredError, distanceM);
        if (!tracking.feasible()) {
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

    // =========================================================================
    // State management
    // =========================================================================

    private void transitionTo(State newState) {
        state = newState;
        stateTimer.restart();
        resetContinuousLossTimer();
        if (newState == State.ALIGN || newState == State.DONE) {
            previousAimErrorDeg = Double.NaN;
            alignmentLocked = false;
            resetAlignmentLockTimer();
            resetAlignmentBreakTimer();
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

    // =========================================================================
    // Shot tracking — stationary only (no moving shots)
    // =========================================================================

    private ShotTracking buildStationaryTracking(double filteredErrorDeg, double distanceM) {
        double aimErrorDeg = filteredErrorDeg;
        double targetRps = ShooterSubsystem.calculateTargetRPS(distanceM);
        boolean feasible = Double.isFinite(targetRps) && targetRps > 0.0
                && isShotDistanceFeasible(distanceM);

        boolean holdingAlignment = shouldHoldAlignment(aimErrorDeg);
        double rotCmd = 0.0;
        if (!holdingAlignment) {
            double pidOutput = turnPID.calculate(filteredErrorDeg, 0.0);
            rotCmd = MathUtil.clamp(
                    pidOutput,
                    -Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS,
                    Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS);
        }

        boolean feedGateSettled = updateFeedGateTimer(
                feasible
                        && holdingAlignment
                        && isShooterReady(targetRps));
        boolean feedGateReady = feasible
                && alignmentLocked
                && feedGateSettled;

        return new ShotTracking(
                aimErrorDeg,
                distanceM,
                targetRps,
                new ChassisSpeeds(0.0, 0.0, 0.0),
                rotCmd,
                feasible,
                feedGateReady);
    }

    private void applyTracking(ShotTracking tracking) {
        workHasTarget = true;
        workGeometryFeasible = true;
        workHasShootableTarget = true;
        workHeadingErrorDeg = tracking.aimErrorDeg();
        workAimErrorDeg = tracking.aimErrorDeg();
        workDistanceM = tracking.distanceM();
        workFeedGateReady = tracking.feedGateReady();
        workTargetRps = tracking.targetRps();
        shooter.setShooterVelocity(workTargetRps);
        hood.setAngle(HoodSubsystem.calculateTargetAngle(tracking.distanceM()));

        SmartDashboard.putNumber("AlignShoot/EstDistanceM", tracking.distanceM());
    }

    private void driveTracking(ShotTracking tracking) {
        swerve.driveRobotRelative(new ChassisSpeeds(
                tracking.translationCmd().vxMetersPerSecond,
                tracking.translationCmd().vyMetersPerSecond,
                tracking.rotCmdRadPerSec()));
    }

    // =========================================================================
    // Drive helpers
    // =========================================================================

    private void driveSearchPattern() {
        swerve.driveRobotRelative(new ChassisSpeeds(
                0.0,
                0.0,
                searchRotationSign * Constants.AlignShoot.SEARCH_OMEGA_RADPS));
    }

    private void driveTowardsHub(double filteredErrorDeg) {
        double pidOutput = turnPID.calculate(filteredErrorDeg, 0.0);
        double rotCmd = turnPID.atSetpoint()
                ? 0.0
                : MathUtil.clamp(
                        pidOutput,
                        -Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS,
                        Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS);

        swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, rotCmd));
    }

    // =========================================================================
    // Heading filter — smooth pose-estimator heading error updates
    // =========================================================================

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

    // =========================================================================
    // Target presence and feasibility
    // =========================================================================

    private boolean hasShootableTarget(Translation2d hubCenter) {
        boolean hasVision = swerve.isVisionActive();
        workHasTarget = hasVision;
        if (!hasVision) {
            workGeometryFeasible = false;
            workHasShootableTarget = false;
            return false;
        }

        double distanceM = swerve.getDistanceTo(hubCenter);
        double headingErrorDeg = swerve.getHeadingErrorDegTo(hubCenter);
        boolean geometryFeasible = isShotDistanceFeasible(distanceM)
                && isWithinTrackingHeading(headingErrorDeg);
        workGeometryFeasible = geometryFeasible;
        workDistanceM = distanceM;

        if (!geometryFeasible) {
            workHasShootableTarget = false;
            return false;
        }

        workHasShootableTarget = true;
        return true;
    }

    static boolean isShotDistanceFeasible(double distanceM) {
        return Double.isFinite(distanceM)
                && distanceM >= Constants.Vision.MIN_SHOT_DISTANCE_M
                && distanceM <= Constants.Vision.MAX_SHOT_DISTANCE_M;
    }

    private boolean isShooterReady(double targetRps) {
        double leftError = Math.abs(Math.abs(shooter.getLeftRPS()) - targetRps);
        double rightError = Math.abs(Math.abs(shooter.getRightRPS()) - targetRps);
        return leftError <= Constants.AlignShoot.RPS_TOLERANCE_RPS
                && rightError <= Constants.AlignShoot.RPS_TOLERANCE_RPS;
    }

    // =========================================================================
    // Timer helpers
    // =========================================================================

    private void resetFeedGateTimer() {
        feedGateTimer.stop();
        feedGateTimer.reset();
    }

    private boolean updateFeedGateTimer(boolean readyNow) {
        if (!readyNow) {
            resetFeedGateTimer();
            return false;
        }
        if (!feedGateTimer.isRunning()) {
            feedGateTimer.restart();
        }
        return feedGateTimer.hasElapsed(Constants.AlignShoot.SETTLE_TIME_SEC);
    }

    private void resetContinuousLossTimer() {
        continuousLossTimer.stop();
        continuousLossTimer.reset();
    }

    private void resetAlignmentLockTimer() {
        alignmentLockTimer.stop();
        alignmentLockTimer.reset();
    }

    private void resetAlignmentBreakTimer() {
        alignmentBreakTimer.stop();
        alignmentBreakTimer.reset();
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

    // =========================================================================
    // Alignment lock — require stable heading before allowing feed
    // =========================================================================

    private void updateAlignmentLock(double aimErrorDeg) {
        if (!Double.isFinite(aimErrorDeg)) {
            previousAimErrorDeg = Double.NaN;
            alignmentLocked = false;
            resetAlignmentLockTimer();
            resetAlignmentBreakTimer();
            return;
        }
        if (alignmentLocked) {
            if (shouldBreakAlignmentLock(aimErrorDeg)) {
                alignmentLocked = false;
                resetAlignmentLockTimer();
                resetAlignmentBreakTimer();
            }
            previousAimErrorDeg = aimErrorDeg;
            return;
        }
        if (AlignmentCaptureUtil.shouldCaptureOnEntryOrCrossing(
                previousAimErrorDeg,
                aimErrorDeg,
                Constants.AlignShoot.YAW_TOLERANCE_DEG,
                Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG,
                Constants.AlignShoot.CAPTURE_OVERSHOOT_DEG)) {
            alignmentLocked = true;
            hadAlignmentLockThisRun = true;
            resetAlignmentLockTimer();
            resetAlignmentBreakTimer();
            previousAimErrorDeg = aimErrorDeg;
            return;
        }
        resetAlignmentBreakTimer();
        if (shouldHoldAlignment(aimErrorDeg)) {
            if (!alignmentLockTimer.isRunning()) {
                alignmentLockTimer.restart();
            }
            if (alignmentLockTimer.hasElapsed(Constants.AlignShoot.SETTLE_TIME_SEC)) {
                alignmentLocked = true;
                hadAlignmentLockThisRun = true;
            }
        } else {
            resetAlignmentLockTimer();
        }
        previousAimErrorDeg = aimErrorDeg;
    }

    private boolean maintainLockedAlignment(double aimErrorDeg) {
        if (!alignmentLocked) {
            return false;
        }
        if (!shouldBreakAlignmentLock(aimErrorDeg)) {
            return true;
        }
        alignmentLocked = false;
        resetAlignmentLockTimer();
        resetAlignmentBreakTimer();
        return false;
    }

    private boolean shouldBreakAlignmentLock(double aimErrorDeg) {
        if (!Double.isFinite(aimErrorDeg)) {
            resetAlignmentBreakTimer();
            return true;
        }
        if (Math.abs(aimErrorDeg) <= Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG) {
            resetAlignmentBreakTimer();
            return false;
        }
        if (!alignmentBreakTimer.isRunning()) {
            alignmentBreakTimer.restart();
            return false;
        }
        return alignmentBreakTimer.hasElapsed(Constants.AlignShoot.LOCK_BREAK_DEBOUNCE_SEC);
    }

    private boolean shouldHoldAlignment(double aimErrorDeg) {
        if (!Double.isFinite(aimErrorDeg)) {
            return false;
        }
        double absError = Math.abs(aimErrorDeg);
        return absError <= Constants.AlignShoot.YAW_TOLERANCE_DEG
                || (alignmentLockTimer.isRunning()
                        && absError <= Constants.AlignShoot.YAW_BREAK_TOLERANCE_DEG)
                || alignmentLocked;
    }

    private boolean shouldHoldContinuousFeed() {
        if (!continuousLossTimer.isRunning()) {
            continuousLossTimer.restart();
        }
        return !continuousLossTimer.hasElapsed(Constants.AlignShoot.CONTINUOUS_FEED_REACQUIRE_SEC);
    }

    private boolean hasAlignConvergenceTimedOut() {
        return !continuousFeedUntilInterrupted
                && alignOverallTimer.hasElapsed(ALIGN_CONVERGENCE_TIMEOUT_SEC);
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

    private void updateSearchDirectionFromError(double errorDeg) {
        double desiredSign = -Math.signum(errorDeg);
        if (desiredSign != 0.0) {
            searchRotationSign = desiredSign;
        }
    }

    private static boolean isWithinTrackingHeading(double errorDeg) {
        return Double.isFinite(errorDeg)
                && Math.abs(errorDeg) <= Constants.AlignShoot.ACQUIRE_YAW_MAX_DEG;
    }

    // =========================================================================
    // Telemetry
    // =========================================================================

    private void updateTelemetry() {
        telemetrySnapshot = new TelemetrySnapshot(
                workState,
                workCommandActive,
                workHasTarget,
                workGeometryFeasible,
                workHasShootableTarget,
                workHeadingErrorDeg,
                workAimErrorDeg,
                workDistanceM,
                workTargetRps,
                workFeedGateReady,
                workLastAbortReason);
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber("AlignShoot/HeadingErrorDeg", workHeadingErrorDeg);
        SmartDashboard.putNumber("AlignShoot/AimErrorDeg", workAimErrorDeg);
        SmartDashboard.putNumber("AlignShoot/DistanceM", workDistanceM);
        SmartDashboard.putNumber("AlignShoot/CalculatedRPS", workTargetRps);
        SmartDashboard.putBoolean("AlignShoot/FeedGateReady", workFeedGateReady);
    }

    public static TelemetrySnapshot getTelemetrySnapshot() { return telemetrySnapshot; }

    public static String getTelemetryState() { return telemetrySnapshot.state(); }
    public static boolean isTelemetryCommandActive() { return telemetrySnapshot.commandActive(); }
    public static boolean telemetryHasTarget() { return telemetrySnapshot.hasTarget(); }
    public static boolean telemetryGeometryFeasible() { return telemetrySnapshot.geometryFeasible(); }
    public static boolean telemetryHasShootableTarget() { return telemetrySnapshot.hasShootableTarget(); }
    public static double getTelemetryHeadingErrorDeg() { return telemetrySnapshot.headingErrorDeg(); }
    public static double getTelemetryAimErrorDeg() { return telemetrySnapshot.aimErrorDeg(); }
    public static double getTelemetryDistanceM() { return telemetrySnapshot.distanceM(); }
    public static double getTelemetryTargetRps() { return telemetrySnapshot.targetRps(); }
    public static boolean telemetryFeedGateReady() { return telemetrySnapshot.feedGateReady(); }
    public static String getTelemetryLastAbortReason() { return telemetrySnapshot.lastAbortReason(); }

    private record ShotTracking(
            double aimErrorDeg,
            double distanceM,
            double targetRps,
            ChassisSpeeds translationCmd,
            double rotCmdRadPerSec,
            boolean feasible,
            boolean feedGateReady) {}
}
