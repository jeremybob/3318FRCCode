// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/swerve/SwerveModule.java
//
// PURPOSE: Controls ONE swerve module corner.
//   Each module has:
//     - A DRIVE motor (Falcon 500 / TalonFX) that spins the wheel
//     - A STEER motor (Falcon 500 / TalonFX) that rotates the wheel direction
//     - A CANcoder that reads the absolute wheel angle (so we never lose position)
//
// STEER FEEDBACK STRATEGY:
//   With Phoenix Pro: FusedCANcoder provides continuous CANcoder-corrected feedback.
//   Without Pro (current): "Seed-once + periodic re-sync" — at boot, the CANcoder
//   absolute position seeds the TalonFX's internal encoder, then the position loop
//   runs off the internal encoder (RotorSensor) at full speed with zero CANcoder
//   CAN overhead. A periodic re-sync (~1 Hz) compares the internal encoder against
//   the CANcoder to detect and correct for belt skips.
//   This saves ~384 CAN frames/sec vs RemoteCANcoder at 100 Hz across 4 modules.
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
    private static final int CONTROL_SIGNAL_HZ = 50;
    private static final int CANCODER_TELEMETRY_HZ = 2;

    // Re-sync threshold: if internal encoder and CANcoder diverge by more than
    // this many rotations (~5.4°), re-seed the internal encoder.
    // Small enough to catch a belt skip (typically 1+ teeth ≈ 5-15°), large
    // enough to ignore normal sensor noise.
    private static final double RESYNC_THRESHOLD_ROT = 0.015;

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
    @SuppressWarnings("rawtypes")
    private final StatusSignal steerPosition;

    // Module name for diagnostics
    private final String name;
    private final double cancoderOffsetRot;

    private Rotation2d lastAngle = new Rotation2d();
    private Rotation2d cachedSteerAngle = new Rotation2d();

    // Track the coupling compensation offset so a belt-skip re-sync
    // (which jumps steer position) doesn't cause a sudden change in
    // reported drive distance for odometry.
    private double couplingOffsetMotorRot = 0.0;
    private double lastCouplingSteerRot = 0.0;

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
        driveCfg.CurrentLimits.StatorCurrentLimit       = Constants.Swerve.DRIVE_STATOR_CURRENT_LIMIT_A;
        driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        driveCfg.CurrentLimits.SupplyCurrentLimit       = Constants.Swerve.DRIVE_SUPPLY_CURRENT_LIMIT_A;
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
        steerCfg.CurrentLimits.StatorCurrentLimit       = Constants.Swerve.STEER_STATOR_CURRENT_LIMIT_A;
        steerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        if (Constants.Swerve.USE_PHOENIX_PRO_FEATURES) {
            // FusedCANcoder: TalonFX fuses internal encoder with CANcoder
            // for continuous correction. Requires Phoenix Pro license.
            steerCfg.Feedback.FeedbackRemoteSensorID  = cancoderId;
            steerCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            steerCfg.Feedback.SensorToMechanismRatio  = 1.0;
            steerCfg.Feedback.RotorToSensorRatio      = Constants.Swerve.STEER_GEAR_RATIO;
        } else {
            // Seed-once: use internal encoder (RotorSensor) for feedback.
            // Position is seeded from CANcoder at boot (below) and periodically
            // re-synced to detect belt skips. Saves ~96 CAN frames/sec per module.
            steerCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            steerCfg.Feedback.SensorToMechanismRatio  = Constants.Swerve.STEER_GEAR_RATIO;
        }
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
        steerPosition    = steerMotor.getPosition();

        // ---- Seed internal encoder from CANcoder (non-Pro only) -------------
        // Read the CANcoder's absolute position and write it into the steer
        // motor's internal encoder so the position PID starts at the correct
        // wheel angle. With FusedCANcoder (Pro) this happens automatically.
        if (!Constants.Swerve.USE_PHOENIX_PRO_FEATURES) {
            var cancoderStatus = cancoderPosition.waitForUpdate(0.5);
            if (cancoderStatus.getStatus().isOK()) {
                double cancoderRot = cancoderStatus.getValueAsDouble();
                steerMotor.setPosition(cancoderRot);
                System.out.println("[SwerveModule " + name + "] Seeded steer encoder from CANcoder: "
                        + String.format("%.4f", cancoderRot) + " rot");
            } else {
                System.err.println("[SwerveModule " + name + "] WARNING: CANcoder seed failed ("
                        + cancoderStatus.getStatus().getName()
                        + "). Steer position may be incorrect until periodic re-sync corrects it.");
            }
        }

        // ---- Optimize CAN frame rates for swerve-critical signals -----------
        BaseStatusSignal.setUpdateFrequencyForAll(
                CONTROL_SIGNAL_HZ,
                driveVelocity,
                drivePosition,
                driveAppliedVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(1, driveTemp);

        if (Constants.Swerve.USE_PHOENIX_PRO_FEATURES) {
            // FusedCANcoder: keep control-critical feedback at drivetrain rate.
            BaseStatusSignal.setUpdateFrequencyForAll(
                    CONTROL_SIGNAL_HZ,
                    cancoderPosition,
                    cancoderAbsolutePosition);
            steerPosition.setUpdateFrequency(CONTROL_SIGNAL_HZ);
        } else {
            // Seed-once: CANcoder only needed for diagnostics + periodic re-sync (~1 Hz)
            BaseStatusSignal.setUpdateFrequencyForAll(
                    CANCODER_TELEMETRY_HZ,
                    cancoderPosition,
                    cancoderAbsolutePosition);
            // Steer motor position IS the feedback now — needs full update rate
            steerPosition.setUpdateFrequency(CONTROL_SIGNAL_HZ);
        }
        steerMotor.getVelocity().setUpdateFrequency(8);
        steerMotor.getDeviceTemp().setUpdateFrequency(1);

        applyWithRetry(
                cancoder::optimizeBusUtilization,
                "CANcoder bus optimization (id=" + cancoderId + ")");
        applyWithRetry(
                driveMotor::optimizeBusUtilization,
                "Drive TalonFX bus optimization (id=" + driveId + ")");
        applyWithRetry(
                steerMotor::optimizeBusUtilization,
                "Steer TalonFX bus optimization (id=" + steerId + ")");

        refreshSteerAngle();
        lastAngle = getSteerAngle();

        // Initialize coupling compensation tracking from current positions.
        lastCouplingSteerRot = getSteerAngle().getRotations();
        couplingOffsetMotorRot = lastCouplingSteerRot * Constants.Swerve.COUPLE_RATIO;
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
        desiredState.optimize(getSteerAngle());

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
        lastAngle = getSteerAngle();
        driveMotor.setControl(driveDutyCycleRequest.withOutput(clampDutyCycle(drivePercent)));
        steerMotor.setControl(steerDutyCycleRequest.withOutput(clampDutyCycle(steerPercent)));
    }

    // --------------------------------------------------------------------------
    // getSteerAngle()
    //
    // Returns the current wheel angle from the steer motor's internal encoder.
    // With seed-once mode, this is the primary feedback source (seeded from
    // CANcoder at boot, periodically re-synced).
    // With FusedCANcoder (Pro), this returns the fused position.
    // --------------------------------------------------------------------------
    /** Refreshes the cached steer angle from CAN. Call once per loop cycle. */
    public void refreshSteerAngle() {
        cachedSteerAngle = Rotation2d.fromRotations(steerPosition.refresh().getValueAsDouble());
    }

    /** Returns the cached steer angle (updated by refreshSteerAngle). */
    public Rotation2d getSteerAngle() {
        return cachedSteerAngle;
    }

    /**
     * Returns the wheel angle from the CANcoder directly (bypass internal encoder).
     * Use this only for diagnostics and re-sync checks, NOT for control loops.
     */
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(cancoderPosition.refresh().getValueAsDouble());
    }

    public SwerveModuleState getState() {
        double motorRPS    = driveVelocity.refresh().getValueAsDouble();
        double wheelRPS    = motorRPS / Constants.Swerve.DRIVE_GEAR_RATIO;
        double speedMps    = wheelRPS * Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        return new SwerveModuleState(speedMps, getSteerAngle());
    }

    public SwerveModulePosition getPosition() {
        double motorRotations  = drivePosition.refresh().getValueAsDouble();
        // Compensate for coupling: steering the module causes the drive motor
        // to spin slightly. Subtract that parasitic motion for accurate odometry.
        // Use delta-based tracking so a belt-skip re-sync (which jumps steer
        // position) doesn't cause a sudden spike in the coupling compensation.
        double steerRotations  = cachedSteerAngle.getRotations();
        double steerDelta = steerRotations - lastCouplingSteerRot;
        lastCouplingSteerRot = steerRotations;
        couplingOffsetMotorRot += steerDelta * Constants.Swerve.COUPLE_RATIO;
        double correctedMotorRot = motorRotations - couplingOffsetMotorRot;
        double wheelRotations  = correctedMotorRot / Constants.Swerve.DRIVE_GEAR_RATIO;
        double distanceMeters  = wheelRotations * Constants.Swerve.WHEEL_CIRCUMFERENCE_M;
        return new SwerveModulePosition(distanceMeters, cachedSteerAngle);
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

    /** Returns a calibration sample from the existing CANcoder (no duplicate handles). */
    public SwerveCalibrationUtil.CalibrationSample getCalibrationSample() {
        double configuredAbsoluteRot = cancoderAbsolutePosition.refresh().getValueAsDouble();
        return SwerveCalibrationUtil.sample(configuredAbsoluteRot, cancoderOffsetRot);
    }

    // --------------------------------------------------------------------------
    // checkSteerSync()
    //
    // Compares the steer motor's internal encoder against the CANcoder.
    // If they diverge beyond RESYNC_THRESHOLD_ROT, re-seeds the internal
    // encoder from the CANcoder. This detects belt skips mid-match.
    //
    // Call this at ~1 Hz from SwerveSubsystem.periodic() — NOT every loop.
    // Returns true if a re-sync was performed.
    // --------------------------------------------------------------------------
    public boolean checkSteerSync() {
        if (Constants.Swerve.USE_PHOENIX_PRO_FEATURES) {
            return false; // FusedCANcoder handles this automatically
        }

        double internalRot = steerPosition.refresh().getValueAsDouble();
        double cancoderRot = cancoderPosition.refresh().getValueAsDouble();

        // Wrap the difference to [-0.5, +0.5] to handle wrap-around
        double error = cancoderRot - internalRot;
        error -= Math.round(error); // wrap to nearest half-rotation

        if (Math.abs(error) > RESYNC_THRESHOLD_ROT) {
            steerMotor.setPosition(cancoderRot);
            System.err.println("[SwerveModule " + name + "] BELT SKIP DETECTED! "
                    + "Internal=" + String.format("%.4f", internalRot)
                    + " CANcoder=" + String.format("%.4f", cancoderRot)
                    + " error=" + String.format("%.4f", error) + " rot. Re-synced.");
            return true;
        }
        return false;
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
