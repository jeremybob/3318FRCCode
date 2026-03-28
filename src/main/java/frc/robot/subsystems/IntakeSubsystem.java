// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/IntakeSubsystem.java
//
// PURPOSE: Controls the intake mechanism.
//   Hardware:
//     - SLIDE motor: REV SparkMax + NEO (rack and pinion linear slide)
//     - ROLLER motors: 2x TalonFX / Kraken X60 (leader + follower, opposite
//       directions) — spin rubber wheels to grab game pieces
//     - Hall effect sensor: detects when slide is fully retracted (home)
//
// NO BURN FLASH POLICY:
//   This code does NOT call burnFlash() or restoreFactoryDefaults().
//   The permanent configuration (conversion factors, default direction, etc.)
//   must be set once using the REV Hardware Client. This code only sets
//   runtime safety parameters (current limit, brake mode, PID gains).
//
// HOMING:
//   The linear slide needs to find its home position at startup because the
//   relative encoder loses track when the robot is powered off.
//   Run IntakeHomeCommand before using setSlidePosition().
// ============================================================================
package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static final int CTRE_CONFIG_RETRIES = 5;

    // ---- Slide motor (SparkMax + NEO, rack and pinion) ----
    private final SparkMax      slideMotor   = new SparkMax(Constants.CAN.INTAKE_SLIDE_NEO, MotorType.kBrushless);
    private final RelativeEncoder  slideEncoder = slideMotor.getEncoder();
    private final SparkClosedLoopController slidePID   = slideMotor.getClosedLoopController();

    // ---- Hall effect sensor (detects home / fully retracted position) ----
    // DigitalInput.get() returns true when sensor is OPEN (not triggered)
    //                             false when sensor is CLOSED (triggered)
    // We negate it in getLimitSwitchPressed() so "true" = triggered.
    private final DigitalInput homeSensor = new DigitalInput(Constants.DIO.INTAKE_HOME_SWITCH);
    private final Debouncer homeSensorDebouncer =
            new Debouncer(Constants.Intake.HOME_SWITCH_DEBOUNCE_SEC, Debouncer.DebounceType.kBoth);

    // ---- Roller motors (2x TalonFX / Kraken X60, leader + follower) ----
    private final TalonFX rollerLeader =
            new TalonFX(Constants.CAN.INTAKE_ROLLER_LEADER, new CANBus(Constants.CAN.RIO_CAN_BUS));
    private final TalonFX rollerFollower =
            new TalonFX(Constants.CAN.INTAKE_ROLLER_FOLLOWER, new CANBus(Constants.CAN.RIO_CAN_BUS));
    @SuppressWarnings("rawtypes")
    private final StatusSignal rollerStatorCurrent = rollerLeader.getStatorCurrent();

    // ---- State tracking ----
    // isHomed is false at startup until IntakeHomeCommand confirms the slide
    // has reached the Hall effect sensor. We refuse position commands until homed.
    private boolean isHomed = false;
    private boolean homeSensorRawPressed = false;
    private boolean homeSensorPressed = false;
    private boolean minSoftLimitLatched = false;
    private boolean maxSoftLimitLatched = false;
    private double cachedRollerCurrentAmps = 0.0;
    private boolean cachedRollerCurrentSignalOk = false;

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public IntakeSubsystem() {
        // ---- Slide motor (SparkMax) runtime configuration ----
        // NOTE: We do NOT call restoreFactoryDefaults() or burnFlash().
        // The permanent settings (position conversion factor, etc.) must be
        // configured once in the REV Hardware Client, then they persist.

        SparkMaxConfig slideConfig = new SparkMaxConfig();

        // Current limit protects the NEO and gearbox during homing stalls
        slideConfig.smartCurrentLimit(Constants.Intake.SLIDE_CURRENT_LIMIT_A);

        // Brake mode: slide holds its position when power is removed
        slideConfig.idleMode(IdleMode.kBrake);

        // Position conversion: sets what unit the encoder reports.
        // NEO → 2:1 reduction → 2.5" diameter pinion gear on rack.
        // One motor revolution = (π × 2.5) / 2.0 ≈ 3.927 inches of travel.
        // IMPORTANT: This MUST also be set in REV Hardware Client via burnFlash!
        //            We set it here as a safety net at runtime.
        slideConfig.encoder.positionConversionFactor(Constants.Intake.SLIDE_POS_CONV_IN);
        // Velocity unit in inches/sec so MAXMotion can be tuned in slide units.
        slideConfig.encoder.velocityConversionFactor(Constants.Intake.SLIDE_POS_CONV_IN / 60.0);

        // Slide position PID (built into SparkMax)
        slideConfig.closedLoop.p(Constants.Intake.SLIDE_kP);
        slideConfig.closedLoop.i(0.0);  // no integral — it causes windup in position control
        slideConfig.closedLoop.d(Constants.Intake.SLIDE_kD);
        slideConfig.closedLoop.outputRange(
                -Constants.Intake.SLIDE_PID_MAX_OUTPUT_RETRACT,
                Constants.Intake.SLIDE_PID_MAX_OUTPUT_EXTEND);
        slideConfig.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .cruiseVelocity(Constants.Intake.SLIDE_MAX_MOTION_CRUISE_VEL_IN_PER_SEC)
                .maxAcceleration(Constants.Intake.SLIDE_MAX_MOTION_ACCEL_IN_PER_SEC2)
                .allowedProfileError(Constants.Intake.SLIDE_MAX_MOTION_ALLOWED_ERROR_IN);
        slideMotor.configure(slideConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // ---- Roller leader motor (TalonFX) configuration ----
        TalonFXConfiguration rollerCfg = new TalonFXConfiguration();

        // Coast mode: roller can spin freely after power removed (doesn't jam)
        rollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerCfg.MotorOutput.Inverted = Constants.Intake.ROLLER_MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // === CRITICAL: Current limiting prevents motor damage during game piece jams ===
        // Without this, the Kraken can burn out or trip a breaker if the intake stalls.
        rollerCfg.CurrentLimits.StatorCurrentLimit       = Constants.Intake.ROLLER_STATOR_LIMIT_A;
        rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerCfg.CurrentLimits.SupplyCurrentLimit       = 40;
        rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        applyWithRetry(
                () -> rollerLeader.getConfigurator().apply(rollerCfg),
                "Intake roller leader config (id=" + Constants.CAN.INTAKE_ROLLER_LEADER + ")");

        // Reduce CAN status frame rates — intake roller is low-priority.
        // Stator current at 20 Hz for reliable 300ms stall detection window.
        rollerStatorCurrent.setUpdateFrequency(20);
        rollerLeader.getVelocity().setUpdateFrequency(4);
        rollerLeader.getPosition().setUpdateFrequency(4);
        rollerLeader.getDeviceTemp().setUpdateFrequency(1);
        applyWithRetry(
                rollerLeader::optimizeBusUtilization,
                "Intake roller leader bus optimization (id=" + Constants.CAN.INTAKE_ROLLER_LEADER + ")");

        // ---- Roller follower motor (TalonFX) configuration ----
        // Follower config: same current limits, but follower handles its own limits.
        TalonFXConfiguration followerCfg = new TalonFXConfiguration();
        followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerCfg.CurrentLimits.StatorCurrentLimit       = Constants.Intake.ROLLER_STATOR_LIMIT_A;
        followerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        followerCfg.CurrentLimits.SupplyCurrentLimit       = 40;
        followerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        applyWithRetry(
                () -> rollerFollower.getConfigurator().apply(followerCfg),
                "Intake roller follower config (id=" + Constants.CAN.INTAKE_ROLLER_FOLLOWER + ")");

        // Motors point in opposite directions, so follower opposes the leader.
        // Follower(leaderID, opposeLeader=true) — Phoenix 6 handles the rest.
        rollerFollower.setControl(new Follower(Constants.CAN.INTAKE_ROLLER_LEADER, true));

        // Minimize CAN traffic on the follower — we only need telemetry from the leader.
        rollerFollower.getVelocity().setUpdateFrequency(4);
        rollerFollower.getPosition().setUpdateFrequency(4);
        rollerFollower.getDeviceTemp().setUpdateFrequency(1);
        applyWithRetry(
                rollerFollower::optimizeBusUtilization,
                "Intake roller follower bus optimization (id=" + Constants.CAN.INTAKE_ROLLER_FOLLOWER + ")");

        updateRollerTelemetry();
        updateHomeSensorState();
    }

    // --------------------------------------------------------------------------
    // periodic() — publish telemetry to SmartDashboard every 20ms
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        updateHomeSensorState();
        updateRollerTelemetry();

        // If we boot while already at the home sensor, trust that as a valid zero.
        if (!isHomed && getLimitSwitchPressed()) {
            resetEncoderToHome();
        }

        SmartDashboard.putNumber("Intake/SlidePositionIn",  slideEncoder.getPosition());
        SmartDashboard.putBoolean("Intake/IsHomed",         isHomed);
        SmartDashboard.putBoolean("Intake/HomeSensor",      getLimitSwitchPressed());
        SmartDashboard.putBoolean("Intake/HomeSensorRaw",   homeSensorRawPressed);
    }

    // --------------------------------------------------------------------------
    // getLimitSwitchPressed()
    //
    // Returns true when the slide has reached the home (retracted) position.
    // --------------------------------------------------------------------------
    public boolean getLimitSwitchPressed() {
        return homeSensorPressed;  // true = sensor triggered = slide is home
    }

    // --------------------------------------------------------------------------
    // isHomed()
    //
    // Returns true once IntakeHomeCommand has successfully found the home position.
    // --------------------------------------------------------------------------
    public boolean isHomed() {
        return isHomed;
    }

    public double getSlidePositionIn() {
        return slideEncoder.getPosition();
    }

    public record RollerCurrentSample(double amps, boolean signalOk) {}

    public RollerCurrentSample sampleRollerCurrent() {
        return new RollerCurrentSample(cachedRollerCurrentAmps, cachedRollerCurrentSignalOk);
    }

    public double getRollerCurrentAmps() {
        return sampleRollerCurrent().amps();
    }

    private void updateRollerTelemetry() {
        cachedRollerCurrentAmps = rollerStatorCurrent.getValueAsDouble();
        cachedRollerCurrentSignalOk = rollerStatorCurrent.getStatus().isOK();
    }

    // --------------------------------------------------------------------------
    // setSlidePower()
    //
    // Directly sets slide motor power (-1.0 to +1.0) with encoder soft limits.
    // Used by automated safety-controlled flows. Does NOT use PID.
    // --------------------------------------------------------------------------
    public void setSlidePower(double power) {
        setSlidePowerInternal(power, true);
    }

    /** @deprecated Use {@link #setSlidePower(double)} */
    @Deprecated
    public void setTiltPower(double power) {
        setSlidePower(power);
    }

    // --------------------------------------------------------------------------
    // setSlidePowerHoming()
    //
    // Homing must always be able to drive to the home sensor even if encoder
    // state is stale and soft limits are latched. Home-sensor stop logic still
    // applies, so motion toward home stops when the sensor is triggered.
    // --------------------------------------------------------------------------
    public void setSlidePowerHoming(double power) {
        setSlidePowerInternal(power, false);
    }

    /** @deprecated Use {@link #setSlidePowerHoming(double)} */
    @Deprecated
    public void setTiltPowerHoming(double power) {
        setSlidePowerHoming(power);
    }

    // --------------------------------------------------------------------------
    // setSlidePowerManual()
    //
    // Manual control bypasses software position limits for operator authority,
    // but still honors the home sensor when commanding toward home.
    // --------------------------------------------------------------------------
    public void setSlidePowerManual(double power) {
        setSlidePowerInternal(power, false);
    }

    /** @deprecated Use {@link #setSlidePowerManual(double)} */
    @Deprecated
    public void setTiltPowerManual(double power) {
        setSlidePowerManual(power);
    }

    private void setSlidePowerInternal(double power, boolean enforceSoftLimits) {
        double clampedPower = Math.max(-1.0, Math.min(1.0, power));
        double homeDirection = Math.signum(Constants.Intake.HOME_POWER);
        boolean commandingTowardHome = homeDirection != 0.0 && (clampedPower * homeDirection) > 0.0;
        boolean commandingAwayFromHome = homeDirection != 0.0 && (clampedPower * homeDirection) < 0.0;

        if (commandingTowardHome && getLimitSwitchPressed()) {
            clampedPower = 0.0;
        }
        if (enforceSoftLimits && isHomed) {
            double positionIn = getSlidePositionIn();
            double hysteresisIn = Constants.Intake.SLIDE_SOFT_LIMIT_HYSTERESIS_IN;

            if (positionIn <= Constants.Intake.SLIDE_MIN_IN) {
                minSoftLimitLatched = true;
            } else if (minSoftLimitLatched
                    && positionIn >= Constants.Intake.SLIDE_MIN_IN + hysteresisIn) {
                minSoftLimitLatched = false;
            }

            if (positionIn >= Constants.Intake.SLIDE_MAX_IN) {
                maxSoftLimitLatched = true;
            } else if (maxSoftLimitLatched
                    && positionIn <= Constants.Intake.SLIDE_MAX_IN - hysteresisIn) {
                maxSoftLimitLatched = false;
            }

            if (commandingAwayFromHome && maxSoftLimitLatched) {
                clampedPower = 0.0;
            }
            if (commandingTowardHome && minSoftLimitLatched) {
                clampedPower = 0.0;
            }
        } else {
            minSoftLimitLatched = false;
            maxSoftLimitLatched = false;
        }

        slideMotor.set(clampedPower);
    }

    // --------------------------------------------------------------------------
    // setRollerPower()
    //
    // Spins the intake rollers at the given power (-1.0 to +1.0).
    // Positive = intake direction, negative = eject.
    // The follower motor automatically mirrors the leader (opposite direction).
    // --------------------------------------------------------------------------
    public void setRollerPower(double power) {
        rollerLeader.set(power);
    }

    // --------------------------------------------------------------------------
    // resetEncoderToHome()
    //
    // Marks the current position as 0 inches (home/retracted).
    // Called by IntakeHomeCommand when the Hall effect sensor is triggered.
    // --------------------------------------------------------------------------
    public void resetEncoderToHome() {
        slideEncoder.setPosition(0.0);
        isHomed = true;
    }

    // --------------------------------------------------------------------------
    // setSlidePosition()
    //
    // Commands the linear slide to move to a specific position (in inches).
    // Only works after homing — if the encoder isn't zeroed, we don't know
    // where "6 inches" actually is relative to the slide's true position.
    //
    // The target is clamped to [SLIDE_MIN_IN, SLIDE_MAX_IN] to prevent
    // commanding the slide past its mechanical travel.
    // --------------------------------------------------------------------------
    public void setSlidePosition(double targetInches) {
        if (isHomed) {
            double clamped = Math.max(Constants.Intake.SLIDE_MIN_IN,
                    Math.min(Constants.Intake.SLIDE_MAX_IN, targetInches));
            slidePID.setSetpoint(clamped, ControlType.kPosition);
        } else {
            System.out.println("[IntakeSubsystem] WARNING: setSlidePosition called before homing!");
        }
    }

    /** @deprecated Use {@link #setSlidePosition(double)} */
    @Deprecated
    public void setTiltPosition(double target) {
        setSlidePosition(target);
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Immediately stops both slide and roller motors.
    // --------------------------------------------------------------------------
    public void stop() {
        slideMotor.stopMotor();
        rollerLeader.stopMotor();
        // Follower automatically stops when leader stops.
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
                    System.out.println("[IntakeSubsystem] " + action + " succeeded on attempt " + (i + 1));
                }
                return;
            }
            System.out.println("[IntakeSubsystem] " + action + " attempt " + (i + 1)
                    + " failed: " + lastCode.getName());
        }
        System.err.println("[IntakeSubsystem] ERROR: " + action + " failed after "
                + CTRE_CONFIG_RETRIES + " attempts. Last status: " + lastCode.getName());
    }

    private void updateHomeSensorState() {
        homeSensorRawPressed = !homeSensor.get();
        homeSensorPressed = homeSensorDebouncer.calculate(homeSensorRawPressed);
    }
}
