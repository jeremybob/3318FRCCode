// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/SwerveSubsystem.java
//
// PURPOSE: Manages all four swerve modules together as one drivetrain.
//   Responsibilities:
//     1. Accept driver input (x/y speed + rotation) and distribute to modules
//     2. Track where the robot is on the field using odometry
//     3. Provide pose/speed information so PathPlanner can run autonomous paths
//     4. Support field-oriented driving (joystick "up" always = away from driver)
//
// VISION:
//   Arducam OV9281 on Raspberry Pi 4 with PhotonVision. Pose estimation
//   uses wheel encoders + Pigeon 2 gyro. PhotonVision provides tag yaw/
//   distance to AlignAndShootCommand via RobotContainer polling.
// ============================================================================
package frc.robot.subsystems;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveCalibrationUtil;
import frc.robot.subsystems.swerve.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveValidationMode;

public class SwerveSubsystem extends SubsystemBase {
    private static final int CTRE_CONFIG_RETRIES = 5;

    // ---- Four swerve modules, one per corner ----
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.CAN.FRONT_LEFT_DRIVE,
            Constants.CAN.FRONT_LEFT_STEER,
            Constants.CAN.FRONT_LEFT_CANCODER,
            Constants.Swerve.FL_CANCODER_OFFSET_ROT,
            Constants.Swerve.FL_CANCODER_CLOCKWISE_POSITIVE,
            Constants.Swerve.FL_DRIVE_INVERTED,
            Constants.Swerve.FL_STEER_INVERTED,
            "FL");

    private final SwerveModule frontRight = new SwerveModule(
            Constants.CAN.FRONT_RIGHT_DRIVE,
            Constants.CAN.FRONT_RIGHT_STEER,
            Constants.CAN.FRONT_RIGHT_CANCODER,
            Constants.Swerve.FR_CANCODER_OFFSET_ROT,
            Constants.Swerve.FR_CANCODER_CLOCKWISE_POSITIVE,
            Constants.Swerve.FR_DRIVE_INVERTED,
            Constants.Swerve.FR_STEER_INVERTED,
            "FR");

    private final SwerveModule backLeft = new SwerveModule(
            Constants.CAN.BACK_LEFT_DRIVE,
            Constants.CAN.BACK_LEFT_STEER,
            Constants.CAN.BACK_LEFT_CANCODER,
            Constants.Swerve.BL_CANCODER_OFFSET_ROT,
            Constants.Swerve.BL_CANCODER_CLOCKWISE_POSITIVE,
            Constants.Swerve.BL_DRIVE_INVERTED,
            Constants.Swerve.BL_STEER_INVERTED,
            "BL");

    private final SwerveModule backRight = new SwerveModule(
            Constants.CAN.BACK_RIGHT_DRIVE,
            Constants.CAN.BACK_RIGHT_STEER,
            Constants.CAN.BACK_RIGHT_CANCODER,
            Constants.Swerve.BR_CANCODER_OFFSET_ROT,
            Constants.Swerve.BR_CANCODER_CLOCKWISE_POSITIVE,
            Constants.Swerve.BR_DRIVE_INVERTED,
            Constants.Swerve.BR_STEER_INVERTED,
            "BR");

    private final SwerveModule[] modules = new SwerveModule[] {
        frontLeft, frontRight, backLeft, backRight
    };

    // ---- Gyro (Pigeon 2) ----
    // The Pigeon 2 measures the robot's yaw (rotation angle on the field).
    // We use it to make driving "field-oriented" so joystick up = away from driver.
    private final Pigeon2 pigeon = new Pigeon2(Constants.CAN.PIGEON, new CANBus(Constants.CAN.RIO_CAN_BUS));

    // ---- Kinematics ----
    // SwerveDriveKinematics converts a desired chassis speed (x, y, omega)
    // into individual speeds and angles for each of the four modules.
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Constants.Swerve.FRONT_LEFT_LOCATION,
            Constants.Swerve.FRONT_RIGHT_LOCATION,
            Constants.Swerve.BACK_LEFT_LOCATION,
            Constants.Swerve.BACK_RIGHT_LOCATION);

    // ---- Pose Estimator (odometry) ----
    // Tracks where the robot is on the field using:
    //   - The gyro angle (heading)
    //   - How far each wheel has traveled (module positions)
    // No vision pose correction in USB camera fallback mode.
    private final SwerveDrivePoseEstimator poseEstimator;

    // ---- Vision pose tracking ----
    private double lastVisionUpdateSec = Double.NaN;

    // ---- Field visualization (appears in Shuffleboard / SmartDashboard) ----
    private final Field2d field = new Field2d();
    private ChassisSpeeds cachedRobotRelativeSpeeds = new ChassisSpeeds();

    // Throttle counter for low-priority diagnostics (CANcoder health, angles).
    // Only publish every 5th loop (100ms) to reduce CAN reads and dashboard traffic.
    private int diagnosticLoopCounter = 0;

    // Re-sync counter: check steer encoder vs CANcoder every 50 loops (~1 sec).
    private int resyncLoopCounter = 0;

    private boolean validationActive = false;
    private SwerveCorner validationCorner;
    private SwerveValidationMode validationMode;
    private double validationStartAngleDeg = Double.NaN;
    private double validationStartCANcoderRot = Double.NaN;

    public record ValidationStatus(
            boolean active,
            String moduleToken,
            String moduleDisplayName,
            String modeToken,
            String modeDisplayName,
            double drivePercent,
            double steerPercent,
            double startAngleDeg,
            double angleDeltaDeg,
            double startCANcoderRot,
            double cancoderDeltaRot) {}

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public SwerveSubsystem() {

        // Zero the gyro so "forward" is whatever direction the robot is facing
        // at power-on. If you want the robot to know field orientation from the
        // start of a match, call resetPose() with the actual starting pose instead.
        pigeon.reset();

        // Optimize Pigeon 2 CAN frame rates — yaw is critical for field-relative
        // driving, but pitch/roll are only used for telemetry. With the Pigeon on
        // the dedicated CANivore bus, 100 Hz yaw is sustainable and improves
        // field-relative drive responsiveness.
        pigeon.getYaw().setUpdateFrequency(100);
        pigeon.getPitch().setUpdateFrequency(4);
        pigeon.getRoll().setUpdateFrequency(4);
        applyWithRetry(
                pigeon::optimizeBusUtilization,
                "Pigeon2 bus optimization (id=" + Constants.CAN.PIGEON + ")");

        // Initialize odometry.
        // We start at the origin (0, 0) facing 0 degrees.
        // In actual matches, this gets reset to the known auto starting position.
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getGyroYaw(),           // current heading from gyro
                getModulePositions(),   // current encoder readings from all 4 wheels
                new Pose2d()            // initial pose = (0, 0, 0°)
        );

        // Register the field widget so it shows up in SmartDashboard
        SmartDashboard.putData("Field", field);
    }

    // --------------------------------------------------------------------------
    // periodic()
    //
    // Called every 20ms (50 Hz) automatically by the WPILib scheduler.
    // Use this to update sensors and publish telemetry — NOT for driving commands.
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        // Copy the latest Phoenix status values from the background CAN stream.
        for (SwerveModule mod : modules) {
            mod.refreshFastSignals();
            mod.refreshTelemetrySignals();
        }

        // Update the odometry estimate with the latest gyro and wheel data
        poseEstimator.update(getGyroYaw(), getModulePositions());
        cachedRobotRelativeSpeeds = kinematics.toChassisSpeeds(getModuleStates());

        // Update the field visualization (robot position on the dashboard)
        field.setRobotPose(getPose());

        // Publish useful debugging data to SmartDashboard
        ChassisSpeeds measuredSpeeds = cachedRobotRelativeSpeeds;
        SmartDashboard.putNumber("Swerve/HeadingDeg",        getHeading().getDegrees());
        SmartDashboard.putNumber("Swerve/PigeonYawDeg",      getPigeonYawDeg());
        SmartDashboard.putNumber("Swerve/PigeonPitchDeg",    getPigeonPitchDeg());
        SmartDashboard.putNumber("Swerve/PigeonRollDeg",     getPigeonRollDeg());
        SmartDashboard.putNumber("Swerve/PoseX_m",           getPose().getX());
        SmartDashboard.putNumber("Swerve/PoseY_m",           getPose().getY());
        SmartDashboard.putNumber("Swerve/MeasuredOmegaRadPerSec", measuredSpeeds.omegaRadiansPerSecond);

        // ---- Throttled diagnostics (every 100ms) ----
        // Module angles and CANCoder health are low-priority telemetry.
        // Publishing every 5th loop keeps dashboard chatter low.
        diagnosticLoopCounter++;
        if (diagnosticLoopCounter >= 5) {
            diagnosticLoopCounter = 0;

            SmartDashboard.putNumber("Swerve/FL_AngleDeg",   frontLeft.getSteerAngle().getDegrees());
            SmartDashboard.putNumber("Swerve/FR_AngleDeg",   frontRight.getSteerAngle().getDegrees());
            SmartDashboard.putNumber("Swerve/BL_AngleDeg",   backLeft.getSteerAngle().getDegrees());
            SmartDashboard.putNumber("Swerve/BR_AngleDeg",   backRight.getSteerAngle().getDegrees());

            // CANCoder health — visible on dashboard so CAN issues are obvious
            for (SwerveModule mod : modules) {
                String prefix = "Swerve/" + mod.getName() + "_CC_";
                SmartDashboard.putNumber(prefix + "PosRot",    mod.getCANcoderPositionRot());
                SmartDashboard.putNumber(prefix + "AbsRaw",    mod.getCANcoderAbsoluteRaw());
                SmartDashboard.putBoolean(prefix + "OK",       mod.isCANcoderOk());
            }
        }

        // ---- Periodic steer encoder re-sync (~1 Hz) ----
        // Compares internal encoder against CANcoder to detect belt skips.
        resyncLoopCounter++;
        if (resyncLoopCounter >= 50) {
            resyncLoopCounter = 0;
            for (SwerveModule mod : modules) {
                boolean skipped = mod.checkSteerSync();
                SmartDashboard.putBoolean("Swerve/" + mod.getName() + "_BeltSkip", skipped);
            }
        }
    }

    // ==========================================================================
    // DRIVING METHODS
    // ==========================================================================

    public void drive(double xVelocity, double yVelocity,
                      double rotationalVelocity, boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity, yVelocity, rotationalVelocity, getHeading());
        } else {
            speeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);
        }
        setModuleStates(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(speeds);
    }

    private void setModuleStates(ChassisSpeeds speeds) {
        // Discretize corrects for the fact that the robot is moving while turning.
        // Without this, translating while rotating causes drift perpendicular to
        // the direction of travel (a well-known second-order kinematics issue).
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_TRANSLATION_MPS);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void xLock() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    }

    public void beginValidation(SwerveCorner corner, SwerveValidationMode mode) {
        if (corner == null || mode == null) {
            stopValidation();
            return;
        }

        SwerveModule module = getModule(corner);
        stop();
        validationActive = true;
        validationCorner = corner;
        validationMode = mode;
        validationStartAngleDeg = module.getSteerAngle().getDegrees();
        validationStartCANcoderRot = module.getCANcoderPositionRot();
    }

    public void runValidationStep() {
        if (!validationActive || validationCorner == null || validationMode == null) {
            return;
        }

        for (SwerveModule module : modules) {
            module.stop();
        }
        getModule(validationCorner).setValidationPercentOutput(
                validationMode.drivePercent(),
                validationMode.steerPercent());
    }

    public void stopValidation() {
        stop();
        validationActive = false;
        validationCorner = null;
        validationMode = null;
        validationStartAngleDeg = Double.NaN;
        validationStartCANcoderRot = Double.NaN;
    }

    public ValidationStatus getValidationStatus() {
        if (!validationActive || validationCorner == null || validationMode == null) {
            return new ValidationStatus(
                    false,
                    "NONE",
                    "--",
                    "IDLE",
                    "Idle",
                    0.0,
                    0.0,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN);
        }

        SwerveModule module = getModule(validationCorner);
        double currentAngleDeg = module.getSteerAngle().getDegrees();
        double currentCANcoderRot = module.getCANcoderPositionRot();
        return new ValidationStatus(
                true,
                validationCorner.token(),
                validationCorner.displayName(),
                validationMode.token(),
                validationMode.displayName(),
                validationMode.drivePercent(),
                validationMode.steerPercent(),
                validationStartAngleDeg,
                SwerveCalibrationUtil.angleDeltaDeg(currentAngleDeg, validationStartAngleDeg),
                validationStartCANcoderRot,
                SwerveCalibrationUtil.wrapSignedRotations(currentCANcoderRot - validationStartCANcoderRot));
    }

    public void zeroHeading() {
        pigeon.reset();
        resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    // ==========================================================================
    // GETTERS (used by PathPlanner, commands, and telemetry)
    // ==========================================================================

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return cachedRobotRelativeSpeeds;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public double getPigeonYawDeg() {
        return pigeon.getYaw().getValueAsDouble();
    }

    public double getPigeonPitchDeg() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getPigeonRollDeg() {
        return pigeon.getRoll().getValueAsDouble();
    }

    private Rotation2d getGyroYaw() {
        return pigeon.getRotation2d();
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public double[] getModuleAnglesDeg() {
        return new double[] {
            frontLeft.getSteerAngle().getDegrees(),
            frontRight.getSteerAngle().getDegrees(),
            backLeft.getSteerAngle().getDegrees(),
            backRight.getSteerAngle().getDegrees()
        };
    }

    public double[] getDriveTemperaturesC() {
        return new double[] {
            frontLeft.getDriveTemperatureC(),
            frontRight.getDriveTemperatureC(),
            backLeft.getDriveTemperatureC(),
            backRight.getDriveTemperatureC()
        };
    }

    public double[] getCANcoderPositionsRot() {
        return new double[] {
            frontLeft.getCANcoderPositionRot(),
            frontRight.getCANcoderPositionRot(),
            backLeft.getCANcoderPositionRot(),
            backRight.getCANcoderPositionRot()
        };
    }

    public double[] getCANcoderAbsoluteRawRot() {
        return new double[] {
            frontLeft.getCANcoderAbsoluteRaw(),
            frontRight.getCANcoderAbsoluteRaw(),
            backLeft.getCANcoderAbsoluteRaw(),
            backRight.getCANcoderAbsoluteRaw()
        };
    }

    /** Returns calibration samples for all 4 modules (FL, FR, BL, BR order). */
    public SwerveCalibrationUtil.CalibrationSample[] getCalibrationSamples() {
        return new SwerveCalibrationUtil.CalibrationSample[] {
            frontLeft.getCalibrationSample(),
            frontRight.getCalibrationSample(),
            backLeft.getCalibrationSample(),
            backRight.getCalibrationSample()
        };
    }

    public boolean[] getCANcoderOkStates() {
        return new boolean[] {
            frontLeft.isCANcoderOk(),
            frontRight.isCANcoderOk(),
            backLeft.isCANcoderOk(),
            backRight.isCANcoderOk()
        };
    }

    /**
     * Adds a vision-derived pose measurement to the pose estimator.
     * Called by RobotContainer when PhotonPoseEstimator produces a result.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSec) {
        poseEstimator.addVisionMeasurement(visionPose, timestampSec);
        lastVisionUpdateSec = Timer.getFPGATimestamp();
    }

    /** Returns true if we've had a vision pose update recently. */
    public boolean isVisionActive() {
        if (!Double.isFinite(lastVisionUpdateSec)) return false;
        return (Timer.getFPGATimestamp() - lastVisionUpdateSec) < Constants.Vision.VISION_STALE_SEC;
    }

    /** Returns the horizontal distance from the robot to a field point. */
    public double getDistanceTo(Translation2d fieldPoint) {
        return getPose().getTranslation().getDistance(fieldPoint);
    }

    /**
     * Returns the heading error in degrees to face a field point.
     * Positive = need to turn CCW (left). Normalized to [-180, 180].
     */
    public double getHeadingErrorDegTo(Translation2d fieldPoint) {
        Pose2d pose = getPose();
        double dx = fieldPoint.getX() - pose.getX();
        double dy = fieldPoint.getY() - pose.getY();
        double desiredRad = Math.atan2(dy, dx);
        double currentRad = pose.getRotation().getRadians();
        double errorRad = desiredRad - currentRad;
        // Normalize to [-pi, pi]
        errorRad = Math.atan2(Math.sin(errorRad), Math.cos(errorRad));
        return Math.toDegrees(errorRad);
    }

    /** Dashboard compatibility — returns true if vision pose is recent. */
    public boolean isCameraConnected() {
        return isVisionActive();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    private SwerveModule getModule(SwerveCorner corner) {
        return switch (corner) {
            case FL -> frontLeft;
            case FR -> frontRight;
            case BL -> backLeft;
            case BR -> backRight;
        };
    }

    @FunctionalInterface
    private interface ConfigApplier {
        StatusCode apply();
    }

    private static void applyWithRetry(ConfigApplier applier, String action) {
        StatusCode lastCode = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CTRE_CONFIG_RETRIES; i++) {
            lastCode = applier.apply();
            if (lastCode.isOK()) {
                if (i > 0) {
                    System.out.println("[SwerveSubsystem] " + action + " succeeded on attempt " + (i + 1));
                }
                return;
            }
            System.out.println("[SwerveSubsystem] " + action + " attempt " + (i + 1)
                    + " failed: " + lastCode.getName());
        }
        System.err.println("[SwerveSubsystem] ERROR: " + action + " failed after "
                + CTRE_CONFIG_RETRIES + " attempts. Last status: " + lastCode.getName());
    }
}
