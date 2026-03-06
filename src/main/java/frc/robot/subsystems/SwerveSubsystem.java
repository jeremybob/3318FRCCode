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
// VISION FALLBACK:
//   With the USB camera fallback strategy, pose estimation relies on wheel
//   encoders + Pigeon 2 gyro only (no vision pose correction).  The background
//   RioVisionThread provides yaw/distance to AlignAndShootCommand directly.
// ============================================================================
package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.vision.VisionResult;

public class SwerveSubsystem extends SubsystemBase {

    // ---- Four swerve modules, one per corner ----
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.CAN.FRONT_LEFT_DRIVE,
            Constants.CAN.FRONT_LEFT_STEER,
            Constants.CAN.FRONT_LEFT_CANCODER,
            Constants.Swerve.FL_CANCODER_OFFSET_ROT,
            "FL");

    private final SwerveModule frontRight = new SwerveModule(
            Constants.CAN.FRONT_RIGHT_DRIVE,
            Constants.CAN.FRONT_RIGHT_STEER,
            Constants.CAN.FRONT_RIGHT_CANCODER,
            Constants.Swerve.FR_CANCODER_OFFSET_ROT,
            "FR");

    private final SwerveModule backLeft = new SwerveModule(
            Constants.CAN.BACK_LEFT_DRIVE,
            Constants.CAN.BACK_LEFT_STEER,
            Constants.CAN.BACK_LEFT_CANCODER,
            Constants.Swerve.BL_CANCODER_OFFSET_ROT,
            "BL");

    private final SwerveModule backRight = new SwerveModule(
            Constants.CAN.BACK_RIGHT_DRIVE,
            Constants.CAN.BACK_RIGHT_STEER,
            Constants.CAN.BACK_RIGHT_CANCODER,
            Constants.Swerve.BR_CANCODER_OFFSET_ROT,
            "BR");

    // ---- Gyro (Pigeon 2) ----
    // The Pigeon 2 measures the robot's yaw (rotation angle on the field).
    // We use it to make driving "field-oriented" so joystick up = away from driver.
    private final Pigeon2 pigeon = new Pigeon2(Constants.CAN.PIGEON, new CANBus(Constants.CAN.CTRE_CAN_BUS));

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

    // ---- Vision thread reference (for camera connectivity check) ----
    private final AtomicReference<VisionResult> visionResult;

    // ---- Field visualization (appears in Shuffleboard / SmartDashboard) ----
    private final Field2d field = new Field2d();

    // --------------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   visionResult - shared AtomicReference from RioVisionThread
    // --------------------------------------------------------------------------
    public SwerveSubsystem(AtomicReference<VisionResult> visionResult) {
        this.visionResult = visionResult;

        // Zero the gyro so "forward" is whatever direction the robot is facing
        // at power-on. If you want the robot to know field orientation from the
        // start of a match, call resetPose() with the actual starting pose instead.
        pigeon.reset();

        // Optimize Pigeon 2 CAN frame rates — yaw is critical for field-relative
        // driving, but pitch/roll are only used for telemetry.
        pigeon.getYaw().setUpdateFrequency(100);
        pigeon.getPitch().setUpdateFrequency(4);
        pigeon.getRoll().setUpdateFrequency(4);

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
        // Update the odometry estimate with the latest gyro and wheel data
        poseEstimator.update(getGyroYaw(), getModulePositions());

        // Update the field visualization (robot position on the dashboard)
        field.setRobotPose(getPose());

        // Publish useful debugging data to SmartDashboard
        SmartDashboard.putNumber("Swerve/HeadingDeg",        getHeading().getDegrees());
        SmartDashboard.putNumber("Swerve/PigeonYawDeg",      getPigeonYawDeg());
        SmartDashboard.putNumber("Swerve/PigeonPitchDeg",    getPigeonPitchDeg());
        SmartDashboard.putNumber("Swerve/PigeonRollDeg",     getPigeonRollDeg());
        SmartDashboard.putNumber("Swerve/PoseX_m",           getPose().getX());
        SmartDashboard.putNumber("Swerve/PoseY_m",           getPose().getY());
        SmartDashboard.putNumber("Swerve/FL_AngleDeg",       frontLeft.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/FR_AngleDeg",       frontRight.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/BL_AngleDeg",       backLeft.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/BR_AngleDeg",       backRight.getAbsoluteAngle().getDegrees());

        // ---- CANCoder diagnostics ----
        // Publish per-module CANCoder health so CAN issues are visible on the dashboard.
        for (SwerveModule mod : new SwerveModule[]{frontLeft, frontRight, backLeft, backRight}) {
            String prefix = "Swerve/" + mod.getName() + "_CC_";
            SmartDashboard.putNumber(prefix + "PosRot",    mod.getCANcoderPositionRot());
            SmartDashboard.putNumber(prefix + "AbsRaw",    mod.getCANcoderAbsoluteRaw());
            SmartDashboard.putBoolean(prefix + "OK",       mod.isCANcoderOk());
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
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_TRANSLATION_MPS);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void xLock() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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
        return kinematics.toChassisSpeeds(getModuleStates());
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
            frontLeft.getAbsoluteAngle().getDegrees(),
            frontRight.getAbsoluteAngle().getDegrees(),
            backLeft.getAbsoluteAngle().getDegrees(),
            backRight.getAbsoluteAngle().getDegrees()
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

    /**
     * Returns whether the vision thread is producing results.
     * A result less than 2 seconds old means the camera is connected and working.
     */
    public boolean isCameraConnected() {
        if (!Constants.Vision.ENABLE_VISION) return false;
        VisionResult result = visionResult.get();
        if (result == null) return false;
        return (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - result.timestampSec()) < 2.0;
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }
}
