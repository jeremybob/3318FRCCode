// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/swerve/SwerveModule.java
//
// PURPOSE: Controls ONE swerve module corner.
//   Each module has:
//     - A DRIVE motor (Falcon 500 / TalonFX) that spins the wheel
//     - A STEER motor (Falcon 500 / TalonFX) that rotates the wheel direction
//     - A CANcoder that reads the absolute wheel angle (so we never lose position)
//
// KEY FIX FROM v1: The steer motor's position loop now uses the CANcoder as its
//   feedback source via "FusedCANcoder." This means the controller sees angles
//   in WHEEL rotations (0–1 = 360°), not raw motor shaft rotations.
//   Without this, the steer would be off by the 12.8:1 gear ratio.
//
// KEY FIX FROM v1: Drive motor now uses VelocityVoltage (m/s → motor RPS)
//   instead of DutyCycleOut, giving consistent speed regardless of battery level.
// ============================================================================
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;

public class SwerveModule {

    // The three hardware devices on this corner
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder cancoder;

    // Reusable control request objects — avoids creating garbage every loop
    // VelocityVoltage: tells the drive motor "spin at exactly X rotations/sec"
    private final VelocityVoltage driveRequest = new VelocityVoltage(0)
            .withEnableFOC(true);   // FOC = Field-Oriented Control (better efficiency)

    // PositionVoltage: tells the steer motor "go to exactly this angle"
    private final PositionVoltage steerRequest = new PositionVoltage(0)
            .withEnableFOC(true);

    // --------------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   driveId        - CAN ID of the drive TalonFX
    //   steerId        - CAN ID of the steer TalonFX
    //   cancoderId     - CAN ID of the CANcoder on this corner
    //   cancoderOffsetRot - The offset (in rotations) to make "0" = forward
    //                       Find this in Phoenix Tuner X (see Constants.java notes)
    // --------------------------------------------------------------------------
    public SwerveModule(int driveId, int steerId, int cancoderId,
                        double cancoderOffsetRot) {

        // Create the hardware objects
        driveMotor = new TalonFX(driveId);
        steerMotor = new TalonFX(steerId);
        cancoder   = new CANcoder(cancoderId);

        // ---- Configure the CANcoder ----------------------------------------
        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        // Report angle around a centered discontinuity (equivalent to ±0.5 rotations).
        ccfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        // The offset you measured in Tuner X shifts the reading so forward = 0
        ccfg.MagnetSensor.MagnetOffset = cancoderOffsetRot;
        // CCW positive means turning left increases the angle value (WPILib convention)
        ccfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(ccfg);

        // ---- Configure the DRIVE motor -------------------------------------
        TalonFXConfiguration driveCfg = new TalonFXConfiguration();

        // Brake mode: wheel resists being pushed when power is removed
        driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limits to protect motors from damage during collisions or jams
        driveCfg.CurrentLimits.StatorCurrentLimit       = 60;
        driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        driveCfg.CurrentLimits.SupplyCurrentLimit       = 40;
        driveCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity PID — controls wheel speed
        // kS: min voltage to overcome static friction
        // kV: voltage per RPS (kV = 12V / free_speed for the configured drive motor)
        // kP: extra correction when actual speed doesn't match target
        driveCfg.Slot0.kS = Constants.Swerve.DRIVE_kS;
        driveCfg.Slot0.kV = Constants.Swerve.DRIVE_kV;
        driveCfg.Slot0.kP = Constants.Swerve.DRIVE_kP;

        driveMotor.getConfigurator().apply(driveCfg);

        // ---- Configure the STEER motor ------------------------------------
        TalonFXConfiguration steerCfg = new TalonFXConfiguration();

        // Brake mode: holds the wheel angle when not actively steering
        steerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limit for the steer motor (doesn't need as much torque as drive)
        steerCfg.CurrentLimits.StatorCurrentLimit       = 40;
        steerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // === CRITICAL: Tell the steer motor to use the CANcoder as feedback ===
        // FusedCANcoder means the motor's position loop works in WHEEL rotations,
        // automatically correcting for any sensor drift using the absolute encoder.
        //
        // SensorToMechanismRatio: How many CANcoder rotations per mechanism rotation.
        //   Our CANcoder IS the output shaft, so it's 1:1 → 1.0
        //
        // RotorToSensorRatio: How many motor shaft rotations per CANcoder rotation.
        //   MK4 steer gearbox = 12.8:1, so motor spins 12.8x per wheel turn → 12.8
        steerCfg.Feedback.FeedbackRemoteSensorID  = cancoderId;
        steerCfg.Feedback.FeedbackSensorSource    = FeedbackSensorSourceValue.FusedCANcoder;
        steerCfg.Feedback.SensorToMechanismRatio  = 1.0;
        steerCfg.Feedback.RotorToSensorRatio      = Constants.Swerve.STEER_GEAR_RATIO; // 12.8

        // ContinuousWrap: The position controller wraps around at ±0.5 rotations.
        // This means it always takes the shortest path to any angle — never spins
        // a full revolution when a small rotation is shorter.
        steerCfg.ClosedLoopGeneral.ContinuousWrap = true;

        // Position PID for steering angle control
        steerCfg.Slot0.kP = Constants.Swerve.STEER_kP;
        steerCfg.Slot0.kD = Constants.Swerve.STEER_kD;

        steerMotor.getConfigurator().apply(steerCfg);
    }

    // --------------------------------------------------------------------------
    // setDesiredState()
    //
    // Called every 20ms to tell this module where to point and how fast to go.
    //
    // Parameters:
    //   desiredState - the target angle and speed for this corner
    // --------------------------------------------------------------------------
    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimization: If the target angle is >90° away from where we are,
        // it's faster to reverse the drive direction and turn to the opposite angle.
        // WPILib's optimize() does this automatically.
        // FYI: With ContinuousWrap enabled, Phoenix 6 also handles wrap-around,
        // but we still need optimize() to flip the drive direction when needed.
        // NOTE: optimize() mutates desiredState in place (void return).
        desiredState.optimize(getAbsoluteAngle());

        // ---- Command the STEER motor to the target angle ----
        // The CANcoder (and therefore the position feedback) is in rotations.
        double targetRotations = desiredState.angle.getRotations();
        steerMotor.setControl(steerRequest.withPosition(targetRotations));

        // ---- Command the DRIVE motor to the target speed ----
        // VelocityVoltage takes motor RPS, so we convert:
        //   m/s → wheel RPS → motor RPS (accounting for gear ratio)
        double targetWheelRPS = desiredState.speedMetersPerSecond
                / Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        double targetMotorRPS = targetWheelRPS * Constants.Swerve.DRIVE_GEAR_RATIO;

        driveMotor.setControl(driveRequest.withVelocity(targetMotorRPS));
    }

    // --------------------------------------------------------------------------
    // getAbsoluteAngle()
    //
    // Returns the current wheel angle as a Rotation2d object.
    // Uses the CANcoder's absolute position, which never loses track — even
    // after the robot is turned off and back on.
    // --------------------------------------------------------------------------
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(
                cancoder.getAbsolutePosition().getValueAsDouble());
    }

    // --------------------------------------------------------------------------
    // getState()
    //
    // Returns the current speed AND angle of this module.
    // Used by SmartDashboard logging and odometry.
    // --------------------------------------------------------------------------
    public SwerveModuleState getState() {
        // Convert drive motor RPS back to wheel surface speed (m/s)
        double motorRPS    = driveMotor.getVelocity().getValueAsDouble();
        double wheelRPS    = motorRPS / Constants.Swerve.DRIVE_GEAR_RATIO;
        double speedMps    = wheelRPS * Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        return new SwerveModuleState(speedMps, getAbsoluteAngle());
    }

    // --------------------------------------------------------------------------
    // getPosition()
    //
    // Returns how far the wheel has traveled (in meters) AND its current angle.
    // This is what odometry uses to estimate the robot's position on the field.
    // --------------------------------------------------------------------------
    public SwerveModulePosition getPosition() {
        // Convert drive motor accumulated rotations to distance traveled
        double motorRotations  = driveMotor.getPosition().getValueAsDouble();
        double wheelRotations  = motorRotations / Constants.Swerve.DRIVE_GEAR_RATIO;
        double distanceMeters  = wheelRotations * Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        return new SwerveModulePosition(distanceMeters, getAbsoluteAngle());
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Immediately cuts power to both motors (neutral/coast).
    // Called when the robot is disabled or teleop ends.
    // --------------------------------------------------------------------------
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
}
