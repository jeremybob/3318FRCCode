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
// NEW vs v1:
//   - Added Pigeon 2 gyro for heading
//   - Added SwerveDrivePoseEstimator (odometry)
//   - Added field-oriented drive
//   - Added methods PathPlanner needs (getPose, resetPose, getRobotRelativeSpeeds,
//     driveRobotRelative)
//   - Added SmartDashboard telemetry
//   - Moved joystick deadband to RobotContainer where it belongs (raw axis values)
// ============================================================================
package frc.robot.subsystems;

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

public class SwerveSubsystem extends SubsystemBase {

    // ---- Four swerve modules, one per corner ----
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.CAN.FRONT_LEFT_DRIVE,
            Constants.CAN.FRONT_LEFT_STEER,
            Constants.CAN.FRONT_LEFT_CANCODER,
            Constants.Swerve.FL_CANCODER_OFFSET_ROT);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.CAN.FRONT_RIGHT_DRIVE,
            Constants.CAN.FRONT_RIGHT_STEER,
            Constants.CAN.FRONT_RIGHT_CANCODER,
            Constants.Swerve.FR_CANCODER_OFFSET_ROT);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.CAN.BACK_LEFT_DRIVE,
            Constants.CAN.BACK_LEFT_STEER,
            Constants.CAN.BACK_LEFT_CANCODER,
            Constants.Swerve.BL_CANCODER_OFFSET_ROT);

    private final SwerveModule backRight = new SwerveModule(
            Constants.CAN.BACK_RIGHT_DRIVE,
            Constants.CAN.BACK_RIGHT_STEER,
            Constants.CAN.BACK_RIGHT_CANCODER,
            Constants.Swerve.BR_CANCODER_OFFSET_ROT);

    // ---- Gyro (Pigeon 2) ----
    // The Pigeon 2 measures the robot's yaw (rotation angle on the field).
    // We use it to make driving "field-oriented" so joystick up = away from driver.
    private final Pigeon2 pigeon = new Pigeon2(Constants.CAN.PIGEON);

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
    //   - Optionally: vision target measurements (call addVisionMeasurement())
    private final SwerveDrivePoseEstimator poseEstimator;

    // ---- Field visualization (appears in Shuffleboard / SmartDashboard) ----
    private final Field2d field = new Field2d();

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public SwerveSubsystem() {
        // Zero the gyro so "forward" is whatever direction the robot is facing
        // at power-on. If you want the robot to know field orientation from the
        // start of a match, call resetPose() with the actual starting pose instead.
        pigeon.reset();

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
        // These show up in the "Swerve" group when you open the dashboard.
        SmartDashboard.putNumber("Swerve/HeadingDeg",        getHeading().getDegrees());
        SmartDashboard.putNumber("Swerve/PoseX_m",           getPose().getX());
        SmartDashboard.putNumber("Swerve/PoseY_m",           getPose().getY());
        SmartDashboard.putNumber("Swerve/FL_AngleDeg",       frontLeft.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/FR_AngleDeg",       frontRight.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/BL_AngleDeg",       backLeft.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/BR_AngleDeg",       backRight.getAbsoluteAngle().getDegrees());
    }

    // ==========================================================================
    // DRIVING METHODS
    // ==========================================================================

    // --------------------------------------------------------------------------
    // drive()
    //
    // Main teleop drive method. Takes driver joystick values and applies them.
    //
    // Parameters:
    //   xVelocity         - forward/backward speed in m/s (+x = forward)
    //   yVelocity         - left/right speed in m/s (+y = left)
    //   rotationalVelocity - spin rate in rad/s (+omega = CCW / turning left)
    //   fieldRelative     - if true, "forward" is always away from the driver
    //                       regardless of which direction the robot faces.
    //                       if false, "forward" is relative to the robot's nose.
    // --------------------------------------------------------------------------
    public void drive(double xVelocity, double yVelocity,
                      double rotationalVelocity, boolean fieldRelative) {

        // Build the desired chassis speeds object
        ChassisSpeeds speeds;
        if (fieldRelative) {
            // Field-relative: rotate the joystick vector by the robot's current heading
            // so that joystick "up" always moves the robot away from the driver.
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity, yVelocity, rotationalVelocity, getHeading());
        } else {
            // Robot-relative: "forward" is the robot's nose direction.
            speeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);
        }

        // Convert chassis speeds to individual module states
        setModuleStates(speeds);
    }

    // --------------------------------------------------------------------------
    // driveRobotRelative()
    //
    // Used by PathPlanner during autonomous. PathPlanner sends robot-relative
    // chassis speeds (it handles field math itself).
    // --------------------------------------------------------------------------
    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(speeds);
    }

    // --------------------------------------------------------------------------
    // setModuleStates() - private helper
    //
    // Takes a ChassisSpeeds and distributes them to all four modules.
    // --------------------------------------------------------------------------
    private void setModuleStates(ChassisSpeeds speeds) {
        // Convert chassis speeds into target states for each wheel
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Normalize: if any wheel is asked to go faster than the max, slow ALL
        // wheels proportionally so the robot still moves in the right direction.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_TRANSLATION_MPS);

        // Send target states to each module
        // ORDER MUST MATCH the kinematics constructor: FL, FR, BL, BR
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Cuts power to all modules immediately.
    // --------------------------------------------------------------------------
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // --------------------------------------------------------------------------
    // xLock()
    //
    // Points all four wheels inward at 45-degree angles in an X pattern.
    // The drive motors are set to zero speed. This makes the robot very
    // difficult to push — useful for defense or holding position.
    // --------------------------------------------------------------------------
    public void xLock() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    // --------------------------------------------------------------------------
    // zeroHeading()
    //
    // Resets the gyro so the current robot heading becomes "0 degrees / forward."
    // Useful when the driver needs to re-align field orientation mid-match.
    // Bind this to a button (see RobotContainer).
    // --------------------------------------------------------------------------
    public void zeroHeading() {
        pigeon.reset();
        // Also reset the pose estimator's rotation so odometry stays consistent
        resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    // ==========================================================================
    // GETTERS (used by PathPlanner, commands, and telemetry)
    // ==========================================================================

    // Returns the robot's position and heading on the field
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // Resets odometry to a known pose (called at auto start)
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    // Returns the robot's current speeds in robot-relative frame
    // (PathPlanner needs this to calculate drive corrections)
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    // Returns the robot's current heading as a Rotation2d
    // (This comes from the odometry estimator, which fuses gyro + encoders
    //  and is more accurate than raw gyro alone)
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // Returns the raw Pigeon 2 yaw angle
    // Used only internally for updating the pose estimator
    private Rotation2d getGyroYaw() {
        return pigeon.getRotation2d();
    }

    // Returns an array of the current state (speed + angle) of each module
    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    // Returns an array of module positions (distance traveled + angle)
    // The pose estimator uses this every loop to track robot location.
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }
}
