// ============================================================================
// FILE: src/main/java/frc/robot/commands/AlignAndShootCommand.java
//
// PURPOSE: Lets the driver translate while the robot auto-aims and fires.
//
// SEQUENCE:
//   1. ALIGN    - Spin shooter, auto-aim, and allow capped driver translation
//   2. CLEAR    - Keep aiming/translating while the feeder clears the note
//   3. FEED     - Continue the moving shot with a tighter translation cap
//   4. DONE     - Command finishes, everything stops
// ============================================================================
package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import frc.robot.util.DriverDriveUtil;
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
    private final DoubleSupplier driverForwardSupplier;
    private final DoubleSupplier driverLeftSupplier;
    private final BooleanSupplier driverPrecisionSupplier;
    private final BooleanSupplier driverFieldRelativeSupplier;

    private final PIDController turnPID = new PIDController(
            Constants.Vision.TURN_kP,
            0.0,
            Constants.Vision.TURN_kD);

    private enum State { ALIGN, CLEAR, FEED, DONE }

    private State state;
    private final Timer stateTimer = new Timer();
    private final Timer alignOverallTimer = new Timer();
    private final Timer feedGateTimer = new Timer();
    private double lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
    private double calculatedRps = Constants.Shooter.TARGET_RPS;

    private static final double ALIGN_TIMEOUT_SEC = 3.0;
    private static final double ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0;

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
            DoubleSupplier driverForwardSupplier,
            DoubleSupplier driverLeftSupplier,
            BooleanSupplier driverPrecisionSupplier,
            BooleanSupplier driverFieldRelativeSupplier) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.visionRef = visionRef;
        this.driverForwardSupplier = driverForwardSupplier;
        this.driverLeftSupplier = driverLeftSupplier;
        this.driverPrecisionSupplier = driverPrecisionSupplier;
        this.driverFieldRelativeSupplier = driverFieldRelativeSupplier;

        addRequirements(swerve, shooter, feeder, hopper, intake);
        turnPID.setTolerance(Constants.AlignShoot.YAW_TOLERANCE_DEG);
    }

    @Override
    public void initialize() {
        state = State.ALIGN;
        stateTimer.restart();
        alignOverallTimer.restart();
        resetFeedGateTimer();
        lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;

        workState = State.ALIGN.name();
        workCommandActive = true;
        workHasTarget = false;
        workGeometryFeasible = false;
        workHasShootableTarget = false;
        workYawDeg = Double.NaN;
        workAimErrorDeg = Double.NaN;
        workLeadYawDeg = Double.NaN;
        workPitchDeg = Double.NaN;
        workTargetRps = Constants.Shooter.TARGET_RPS;
        workRadialVelocityMps = Double.NaN;
        workLateralVelocityMps = Double.NaN;
        workCommandedXVelocityMps = 0.0;
        workCommandedYVelocityMps = 0.0;
        workActiveTranslationCapMps = Constants.AlignShoot.MAX_TRANS_MPS;
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

        calculatedRps = Constants.Shooter.TARGET_RPS;
        shooter.setShooterVelocity(calculatedRps);
        SmartDashboard.putString("AlignShoot/State", "ALIGN");
        updateTelemetry();
        publishTelemetry();
    }

    @Override
    public void execute() {
        DriverDriveUtil.DriveRequest driverRequest = DriverDriveUtil.shapeDrive(
                driverForwardSupplier.getAsDouble(),
                driverLeftSupplier.getAsDouble(),
                0.0,
                driverPrecisionSupplier.getAsBoolean(),
                driverFieldRelativeSupplier.getAsBoolean());

        switch (state) {
            case ALIGN -> executeAlign(driverRequest);
            case CLEAR -> executeClear(driverRequest);
            case FEED -> executeFeed(driverRequest);
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
        updateTelemetry();
        publishTelemetry();
    }

    private void executeAlign(DriverDriveUtil.DriveRequest driverRequest) {
        VisionResult result = visionRef.get();
        boolean hasFreshTarget = isResultFresh(result);
        workHasTarget = hasFreshTarget;

        if (!hasFreshTarget) {
            workGeometryFeasible = false;
            workHasShootableTarget = false;
            workYawDeg = Double.NaN;
            workAimErrorDeg = Double.NaN;
            workLeadYawDeg = Double.NaN;
            workPitchDeg = Double.NaN;
            workRadialVelocityMps = Double.NaN;
            workLateralVelocityMps = Double.NaN;
            workTimeOfFlightSec = Double.NaN;
            workFeedGateReady = false;
            resetFeedGateTimer();
            driveTranslationWithoutTarget(driverRequest, Constants.AlignShoot.MAX_TRANS_MPS);

            if (stateTimer.hasElapsed(ALIGN_TIMEOUT_SEC)) {
                abort("No alliance HUB tag found");
            }
            return;
        }

        if (!isShotGeometryFeasible(result)) {
            workHasShootableTarget = false;
            workFeedGateReady = false;
            resetFeedGateTimer();
            abort("Shot geometry not feasible");
            return;
        }

        ShotTracking tracking = buildShotTracking(result, driverRequest, Constants.AlignShoot.MAX_TRANS_MPS);
        if (!tracking.solution().feasible()) {
            workHasShootableTarget = false;
            workFeedGateReady = false;
            resetFeedGateTimer();
            abort("Moving-shot solution invalid");
            return;
        }

        applyTracking(tracking);
        if (tracking.feedGateReady()) {
            if (!feedGateTimer.isRunning()) {
                feedGateTimer.restart();
            }
        } else {
            resetFeedGateTimer();
        }

        driveTracking(tracking);

        if (tracking.feedGateReady()
                && feedGateTimer.hasElapsed(Constants.AlignShoot.SETTLE_TIME_SEC)) {
            transitionTo(State.CLEAR);
        } else if (alignOverallTimer.hasElapsed(ALIGN_CONVERGENCE_TIMEOUT_SEC)) {
            abort("Alignment convergence timeout");
        }
    }

    private void executeClear(DriverDriveUtil.DriveRequest driverRequest) {
        VisionResult result = visionRef.get();
        if (!hasShootableTarget(result)) {
            abort("Vision lost before feed");
            return;
        }

        ShotTracking tracking = buildShotTracking(result, driverRequest, Constants.AlignShoot.MAX_TRANS_MPS);
        if (!tracking.solution().feasible() || !tracking.feedGateReady()) {
            abort("Feed gate lost before feed");
            return;
        }

        applyTracking(tracking);
        driveTracking(tracking);
        feeder.setPower(Constants.Shooter.CLEAR_POWER);

        if (stateTimer.hasElapsed(Constants.Shooter.CLEAR_TIME_SEC)) {
            feeder.stop();
            transitionTo(State.FEED);
        }
    }

    private void executeFeed(DriverDriveUtil.DriveRequest driverRequest) {
        VisionResult result = visionRef.get();
        if (!hasShootableTarget(result)) {
            abort("Vision lost during feed");
            return;
        }

        ShotTracking tracking = buildShotTracking(result, driverRequest, Constants.AlignShoot.MAX_FEED_TRANS_MPS);
        if (!tracking.solution().feasible() || !tracking.feedGateReady()) {
            abort("Feed gate lost during feed");
            return;
        }

        applyTracking(tracking);
        driveTracking(tracking);
        feeder.setPower(Constants.Shooter.FEED_POWER);
        hopper.setPower(Constants.Shooter.FEED_POWER);
        intake.setRollerPower(Constants.Shooter.FEED_POWER);

        if (stateTimer.hasElapsed(Constants.Shooter.FEED_TIME_SEC)) {
            transitionTo(State.DONE);
        }
    }

    private void transitionTo(State newState) {
        state = newState;
        stateTimer.restart();
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

    private ShotTracking buildShotTracking(
            VisionResult result,
            DriverDriveUtil.DriveRequest driverRequest,
            double translationCapMps) {
        double rawYawDeg = result.yawDeg();
        double distanceM = estimateDistanceM(result);
        double pitchDeg = result.pitchDeg();

        double yawRad = Math.toRadians(-rawYawDeg);
        double shotLineX = Math.cos(yawRad);
        double shotLineY = Math.sin(yawRad);
        double lateralAxisX = -shotLineY;
        double lateralAxisY = shotLineX;

        ChassisSpeeds actualSpeeds = swerve.getRobotRelativeSpeeds();
        double radialVelocityMps = dot(actualSpeeds.vxMetersPerSecond, actualSpeeds.vyMetersPerSecond,
                shotLineX, shotLineY);
        double lateralVelocityMps = dot(actualSpeeds.vxMetersPerSecond, actualSpeeds.vyMetersPerSecond,
                lateralAxisX, lateralAxisY);

        ShooterSubsystem.ShotSolution solution = ShooterSubsystem.calculateMovingShotSolution(
                distanceM,
                radialVelocityMps,
                lateralVelocityMps);
        double leadYawDeg = calculateLeadYawDeg(solution.horizontalSpeedMps(), lateralVelocityMps);
        double aimErrorDeg = rawYawDeg - leadYawDeg;

        double rotCmd = Math.abs(aimErrorDeg) <= Constants.AlignShoot.YAW_TOLERANCE_DEG
                ? 0.0
                : MathUtil.clamp(
                        turnPID.calculate(rawYawDeg, leadYawDeg),
                        -Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS,
                        Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS);

        ChassisSpeeds translationCmd = clampDriverTranslation(
                driverRequest,
                shotLineX,
                shotLineY,
                lateralAxisX,
                lateralAxisY,
                translationCapMps);

        boolean feedGateReady = solution.feasible()
                && Math.abs(aimErrorDeg) <= Constants.AlignShoot.YAW_TOLERANCE_DEG
                && isShooterReady(solution.targetRps())
                && Math.hypot(actualSpeeds.vxMetersPerSecond, actualSpeeds.vyMetersPerSecond)
                        <= Constants.AlignShoot.MAX_SPEED_FOR_FEED_MPS;

        return new ShotTracking(
                rawYawDeg,
                aimErrorDeg,
                leadYawDeg,
                pitchDeg,
                distanceM,
                radialVelocityMps,
                lateralVelocityMps,
                translationCmd,
                translationCapMps,
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

        lastValidTargetSeenSec = Timer.getFPGATimestamp();
        calculatedRps = tracking.solution().targetRps();
        workTargetRps = calculatedRps;
        shooter.setShooterVelocity(calculatedRps);

        SmartDashboard.putNumber("AlignShoot/EstDistanceM", tracking.distanceM());
    }

    private void driveTracking(ShotTracking tracking) {
        swerve.driveRobotRelative(new ChassisSpeeds(
                tracking.translationCmd().vxMetersPerSecond,
                tracking.translationCmd().vyMetersPerSecond,
                tracking.rotCmdRadPerSec()));
    }

    private void driveTranslationWithoutTarget(
            DriverDriveUtil.DriveRequest driverRequest,
            double translationCapMps) {
        ChassisSpeeds translationCmd = toRobotRelativeTranslation(driverRequest);
        double speedMps = Math.hypot(
                translationCmd.vxMetersPerSecond,
                translationCmd.vyMetersPerSecond);
        if (speedMps > translationCapMps && speedMps > 1e-6) {
            double scale = translationCapMps / speedMps;
            translationCmd = new ChassisSpeeds(
                    translationCmd.vxMetersPerSecond * scale,
                    translationCmd.vyMetersPerSecond * scale,
                    0.0);
        }

        workCommandedXVelocityMps = translationCmd.vxMetersPerSecond;
        workCommandedYVelocityMps = translationCmd.vyMetersPerSecond;
        workActiveTranslationCapMps = translationCapMps;
        swerve.driveRobotRelative(translationCmd);
    }

    private ChassisSpeeds clampDriverTranslation(
            DriverDriveUtil.DriveRequest driverRequest,
            double shotLineX,
            double shotLineY,
            double lateralAxisX,
            double lateralAxisY,
            double translationCapMps) {
        ChassisSpeeds requested = toRobotRelativeTranslation(driverRequest);

        double radialCmdMps = dot(requested.vxMetersPerSecond, requested.vyMetersPerSecond, shotLineX, shotLineY);
        double lateralCmdMps = dot(requested.vxMetersPerSecond, requested.vyMetersPerSecond,
                lateralAxisX, lateralAxisY);

        radialCmdMps = MathUtil.clamp(
                radialCmdMps,
                -Constants.AlignShoot.MAX_APPROACH_MPS,
                Constants.AlignShoot.MAX_APPROACH_MPS);
        lateralCmdMps = MathUtil.clamp(
                lateralCmdMps,
                -Constants.AlignShoot.MAX_LATERAL_MPS,
                Constants.AlignShoot.MAX_LATERAL_MPS);

        double clampedX = radialCmdMps * shotLineX + lateralCmdMps * lateralAxisX;
        double clampedY = radialCmdMps * shotLineY + lateralCmdMps * lateralAxisY;

        double speedMps = Math.hypot(clampedX, clampedY);
        if (speedMps > translationCapMps && speedMps > 1e-6) {
            double scale = translationCapMps / speedMps;
            clampedX *= scale;
            clampedY *= scale;
        }

        return new ChassisSpeeds(clampedX, clampedY, 0.0);
    }

    private ChassisSpeeds toRobotRelativeTranslation(DriverDriveUtil.DriveRequest driverRequest) {
        if (driverRequest.fieldRelative()) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    driverRequest.xVelocityMps(),
                    driverRequest.yVelocityMps(),
                    0.0,
                    swerve.getHeading());
        }
        return new ChassisSpeeds(
                driverRequest.xVelocityMps(),
                driverRequest.yVelocityMps(),
                0.0);
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

    private double calculateLeadYawDeg(double horizontalSpeedMps, double lateralVelocityMps) {
        if (!Double.isFinite(horizontalSpeedMps) || horizontalSpeedMps <= 1e-6) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.asin(MathUtil.clamp(
                lateralVelocityMps / horizontalSpeedMps,
                -1.0,
                1.0)));
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

        lastValidTargetSeenSec = Timer.getFPGATimestamp();
        workHasShootableTarget = true;
        return true;
    }

    private boolean isShotGeometryFeasible(VisionResult result) {
        double pitchDeg = result.pitchDeg();
        double yawDeg = result.yawDeg();
        workPitchDeg = pitchDeg;

        SmartDashboard.putNumber("AlignShoot/TargetTagId", result.tagId());
        SmartDashboard.putNumber("AlignShoot/YawGeometryCheck", yawDeg);

        if (Math.abs(yawDeg) > Constants.AlignShoot.YAW_TOLERANCE_DEG * 10.0) {
            return false;
        }

        return isShotPitchFeasible(pitchDeg);
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

    private static double dot(double ax, double ay, double bx, double by) {
        return ax * bx + ay * by;
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
