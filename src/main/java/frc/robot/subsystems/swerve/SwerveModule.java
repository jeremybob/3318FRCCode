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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;

public class SwerveModule {
    // Max retries when applying device configuration over CAN
    private static final int CONFIG_APPLY_RETRIES = 5;

    // The three hardware devices on this corner
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder cancoder;

    // Cached status signals for efficient batched reads (avoids per-call CAN traffic).
    // Raw StatusSignal type avoids needing measure-type generics (Angle, AngularVelocity, etc.)
    @SuppressWarnings("rawtypes")
    private final StatusSignal cancoderPosition;
    @SuppressWarnings("rawtypes")
    private final StatusSignal cancoderAbsolutePosition;
    @SuppressWarnings("rawtypes")
    private final StatusSignal driveVelocity;
    @SuppressWarnings("rawtypes")
    private final StatusSignal driveAppliedVoltage;
    @SuppressWarnings("rawtypes")
    private final StatusSignal drivePosition;
    @SuppressWarnings("rawtypes")
    private final StatusSignal driveTemp;

    // Module name for diagnostics
    private final String name;
    private final double cancoderOffsetRot;

    private Rotation2d lastAngle = new Rotation2d();

    // Reusable control request objects — avoids creating garbage every loop
    // VelocityVoltage: tells the drive motor "spin at exactly X rotations/sec"
    private final VelocityVoltage driveRequest = new VelocityVoltage(0)
            .withEnableFOC(Constants.Swerve.USE_PHOENIX_PRO_FEATURES);

    // PositionVoltage: tells the steer motor "go to exactly this angle"
    private final PositionVoltage steerRequest = new PositionVoltage(0)
            .withEnableFOC(Constants.Swerve.USE_PHOENIX_PRO_FEATURES);

    // DutyCycleOut: used only for validation/bring-up so one module can be
    // exercised at a time without the velocity/position loops masking sign issues.
    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0)
            .withEnableFOC(Constants.Swerve.USE_PHOENIX_PRO_FEATURES);
    private final DutyCycleOut steerDutyCycleRequest = new DutyCycleOut(0)
            .withEnableFOC(Constants.Swerve.USE_PHOENIX_PRO_FEATURES);

    // --------------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   driveId        - CAN ID of the drive TalonFX
    //   steerId        - CAN ID of the steer TalonFX
    //   cancoderId     - CAN ID of the CANcoder on this corner
    //   cancoderOffsetRot - The offset (in rotations) to make "0" = forward
    //                       Find this in Phoenix Tuner X (see Constants.java notes)
    //   cancoderClockwisePositive - true if clockwise hand motion increases the CANcoder reading
    //   driveInverted  - true if positive drive command spins the wheel backward
    //   steerInverted  - true if positive steer command moves opposite the CANcoder
    //   moduleName     - Human-readable name for diagnostics (e.g. "FL")
    // --------------------------------------------------------------------------
    public SwerveModule(int driveId, int steerId, int cancoderId,
                        double cancoderOffsetRot,
                        boolean cancoderClockwisePositive,
                        boolean driveInverted,
                        boolean steerInverted,
                        String moduleName) {

        this.name = moduleName;
        this.cancoderOffsetRot = cancoderOffsetRot;

        // Create the hardware objects
        driveMotor = new TalonFX(driveId, new CANBus(Constants.CAN.CTRE_CAN_BUS));
        steerMotor = new TalonFX(steerId, new CANBus(Constants.CAN.CTRE_CAN_BUS));
        cancoder   = new CANcoder(cancoderId, new CANBus(Constants.CAN.CTRE_CAN_BUS));

        // ---- Configure the CANcoder ----------------------------------------
        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        ccfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        ccfg.MagnetSensor.MagnetOffset = cancoderOffsetRot;
        ccfg.MagnetSensor.SensorDirection = sensorDirection(cancoderClockwisePositive);
        applyWithRetry(() -> cancoder.getConfigurator().apply(ccfg),
                "CANcoder config (id=" + cancoderId + ")");

        // ---- Configure the DRIVE motor -------------------------------------
        TalonFXConfiguration driveCfg = new TalonFXConfiguration();
        driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveCfg.MotorOutput.Inverted = invertedValue(driveInverted);
        driveCfg.CurrentLimits.StatorCurrentLimit       = 60;
        driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        driveCfg.CurrentLimits.SupplyCurrentLimit       = 40;
        driveCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveCfg.Slot0.kS = Constants.Swerve.DRIVE_kS;
        driveCfg.Slot0.kV = Constants.Swerve.DRIVE_kV;
        driveCfg.Slot0.kP = Constants.Swerve.DRIVE_kP;
        applyWithRetry(() -> driveMotor.getConfigurator().apply(driveCfg),
                "Drive TalonFX config (id=" + driveId + ")");

        // ---- Configure the STEER motor ------------------------------------
        TalonFXConfiguration steerCfg = new TalonFXConfiguration();
        steerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerCfg.MotorOutput.Inverted = invertedValue(steerInverted);
        steerCfg.CurrentLimits.StatorCurrentLimit       = 40;
        steerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        steerCfg.Feedback.FeedbackRemoteSensorID  = cancoderId;
        steerCfg.Feedback.FeedbackSensorSource = Constants.Swerve.USE_PHOENIX_PRO_FEATURES
                ? FeedbackSensorSourceValue.FusedCANcoder
                : FeedbackSensorSourceValue.RemoteCANcoder;
        steerCfg.Feedback.SensorToMechanismRatio  = 1.0;
        steerCfg.Feedback.RotorToSensorRatio      = Constants.Swerve.STEER_GEAR_RATIO;
        steerCfg.ClosedLoopGeneral.ContinuousWrap = true;
        steerCfg.Slot0.kP = Constants.Swerve.USE_PHOENIX_PRO_FEATURES
                ? Constants.Swerve.STEER_kP_PRO
                : Constants.Swerve.STEER_kP_NON_PRO;
        steerCfg.Slot0.kD = Constants.Swerve.USE_PHOENIX_PRO_FEATURES
                ? Constants.Swerve.STEER_kD_PRO
                : Constants.Swerve.STEER_kD_NON_PRO;
        steerCfg.Slot0.kS = Constants.Swerve.STEER_kS;
        steerCfg.Slot0.kV = Constants.Swerve.STEER_kV;
        applyWithRetry(() -> steerMotor.getConfigurator().apply(steerCfg),
                "Steer TalonFX config (id=" + steerId + ")");

        // ---- Cache status signals for efficient reads -----------------------
        // Grabbing signal references once avoids repeated hashmap lookups each loop.
        cancoderPosition = cancoder.getPosition();
        cancoderAbsolutePosition = cancoder.getAbsolutePosition();
        driveVelocity    = driveMotor.getVelocity();
        driveAppliedVoltage = driveMotor.getMotorVoltage();
        drivePosition    = driveMotor.getPosition();
        driveTemp        = driveMotor.getDeviceTemp();

        // ---- Optimize CAN frame rates for swerve-critical signals -----------
        // CANcoder position and drive velocity/position are needed every loop (50 Hz).
        // Drive temperature is low-priority — 4 Hz is plenty.
        // Steer motor signals we don't read directly (TalonFX reads CANcoder internally
        // for RemoteCANcoder feedback), so reduce steer's outbound status frames.
        BaseStatusSignal.setUpdateFrequencyForAll(100, cancoderPosition, cancoderAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(100, driveVelocity, drivePosition, driveAppliedVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(4, driveTemp);
        // Reduce steer motor status frames — we don't read these signals directly.
        // The TalonFX still reads the CANcoder internally at its own rate.
        steerMotor.getPosition().setUpdateFrequency(50);
        steerMotor.getVelocity().setUpdateFrequency(50);
        steerMotor.getDeviceTemp().setUpdateFrequency(4);

        lastAngle = getAbsoluteAngle();
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
        setDesiredState(desiredState, false);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean forceAngleAtLowSpeed) {

        // Optimization: If the target angle is >90° away from where we are,
        // it's faster to reverse the drive direction and turn to the opposite angle.
        // WPILib's optimize() does this automatically.
        // FYI: With ContinuousWrap enabled, Phoenix 6 also handles wrap-around,
        // but we still need optimize() to flip the drive direction when needed.
        // NOTE: optimize() mutates desiredState in place (void return).
        desiredState.optimize(getAbsoluteAngle());

        // ---- Command the STEER motor to the target angle ----
        // The CANcoder (and therefore the position feedback) is in rotations.
        Rotation2d targetAngle = desiredState.angle;
        if (!forceAngleAtLowSpeed
                && Math.abs(desiredState.speedMetersPerSecond) <= Constants.Swerve.ANGLE_HOLD_SPEED_MPS) {
            targetAngle = lastAngle;
        } else {
            lastAngle = desiredState.angle;
        }

        double targetRotations = targetAngle.getRotations();
        steerMotor.setControl(steerRequest.withPosition(targetRotations));

        // ---- Command the DRIVE motor to the target speed ----
        // VelocityVoltage takes motor RPS, so we convert:
        //   m/s → wheel RPS → motor RPS (accounting for gear ratio)
        double targetWheelRPS = desiredState.speedMetersPerSecond
                / Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        double targetMotorRPS = targetWheelRPS * Constants.Swerve.DRIVE_GEAR_RATIO;

        driveMotor.setControl(driveRequest.withVelocity(targetMotorRPS));
    }

    public void setValidationPercentOutput(double drivePercent, double steerPercent) {
        lastAngle = getAbsoluteAngle();
        driveMotor.setControl(driveDutyCycleRequest.withOutput(clampDutyCycle(drivePercent)));
        steerMotor.setControl(steerDutyCycleRequest.withOutput(clampDutyCycle(steerPercent)));
    }

    // --------------------------------------------------------------------------
    // getAbsoluteAngle()
    //
    // Returns the current wheel angle as a Rotation2d object.
    // Uses CANcoder Position for runtime control (higher default update rate than
    // AbsolutePosition). Magnet offset is still applied in device config.
    // --------------------------------------------------------------------------
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(cancoderPosition.refresh().getValueAsDouble());
    }

    public SwerveModuleState getState() {
        double motorRPS    = driveVelocity.refresh().getValueAsDouble();
        double wheelRPS    = motorRPS / Constants.Swerve.DRIVE_GEAR_RATIO;
        double speedMps    = wheelRPS * Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        return new SwerveModuleState(speedMps, getAbsoluteAngle());
    }

    public SwerveModulePosition getPosition() {
        double motorRotations  = drivePosition.refresh().getValueAsDouble();
        // Compensate for coupling: steering the module causes the drive motor
        // to spin slightly. Subtract that parasitic motion for accurate odometry.
        double steerRotations  = cancoderPosition.refresh().getValueAsDouble();
        double coupledMotorRot = steerRotations * Constants.Swerve.COUPLE_RATIO;
        double correctedMotorRot = motorRotations - coupledMotorRot;
        double wheelRotations  = correctedMotorRot / Constants.Swerve.DRIVE_GEAR_RATIO;
        double distanceMeters  = wheelRotations * Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        return new SwerveModulePosition(distanceMeters, getAbsoluteAngle());
    }

    public double getDriveTemperatureC() {
        return driveTemp.refresh().getValueAsDouble();
    }

    public double getDriveAppliedVoltage() {
        return driveAppliedVoltage.refresh().getValueAsDouble();
    }

    /** Module name for telemetry (e.g. "FL", "FR", "BL", "BR"). */
    public String getName() {
        return name;
    }

    /** Returns the raw CANCoder absolute position (no offset) for diagnostics. */
    public double getCANcoderAbsoluteRaw() {
        double configuredAbsoluteRot = cancoderAbsolutePosition.refresh().getValueAsDouble();
        return SwerveCalibrationUtil.sample(configuredAbsoluteRot, cancoderOffsetRot).noOffsetRot();
    }

    /** Returns the CANCoder position value used for control (with offset applied). */
    public double getCANcoderPositionRot() {
        return cancoderPosition.refresh().getValueAsDouble();
    }

    /** Returns true if the latest CANCoder read was OK (not stale/error). */
    public boolean isCANcoderOk() {
        return cancoderPosition.refresh().getStatus().isOK();
    }

    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    // --------------------------------------------------------------------------
    // applyWithRetry — retries CAN config application up to CONFIG_APPLY_RETRIES
    // times before logging an error. This prevents transient CAN bus glitches
    // from crashing robot init.
    // --------------------------------------------------------------------------
    @FunctionalInterface
    private interface ConfigApplier {
        StatusCode apply();
    }

    private static void applyWithRetry(ConfigApplier applier, String action) {
        StatusCode lastCode = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_APPLY_RETRIES; i++) {
            lastCode = applier.apply();
            if (lastCode.isOK()) {
                if (i > 0) {
                    System.out.println("[SwerveModule] " + action + " succeeded on attempt " + (i + 1));
                }
                return;
            }
            System.out.println("[SwerveModule] " + action + " attempt " + (i + 1)
                    + " failed: " + lastCode.getName());
        }
        // All retries exhausted — log error but do NOT crash robot init.
        // The module may work degraded or the issue may resolve at runtime.
        System.err.println("[SwerveModule] ERROR: " + action + " failed after "
                + CONFIG_APPLY_RETRIES + " attempts. Last status: " + lastCode.getName()
                + ". Module may be degraded.");
    }

    private static InvertedValue invertedValue(boolean inverted) {
        return inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
    }

    private static double clampDutyCycle(double output) {
        return Math.max(-1.0, Math.min(1.0, output));
    }

    private static SensorDirectionValue sensorDirection(boolean clockwisePositive) {
        return clockwisePositive
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
    }
}
