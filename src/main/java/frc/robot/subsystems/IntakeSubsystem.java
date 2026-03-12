// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/IntakeSubsystem.java
//
// PURPOSE: Controls the intake mechanism.
//   Hardware:
//     - TILT motor: REV SparkMax + NEO (rotates intake arm up/down)
//     - ROLLER motor: TalonFX / Kraken X60 (spins rubber wheels to grab game pieces)
//     - Limit switch: digital sensor that detects when arm is fully "home" (up)
//
// NO BURN FLASH POLICY:
//   This code does NOT call burnFlash() or restoreFactoryDefaults().
//   The permanent configuration (conversion factors, default direction, etc.)
//   must be set once using the REV Hardware Client. This code only sets
//   runtime safety parameters (current limit, brake mode, PID gains).
//
// HOMING:
//   The intake arm needs to find its home position at startup because the
//   relative encoder loses track when the robot is powered off.
//   Run IntakeHomeCommand before using setTiltPosition().
// ============================================================================
package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    // ---- Tilt motor (SparkMax + NEO) ----
    private final SparkMax      tiltMotor   = new SparkMax(Constants.CAN.INTAKE_TILT_NEO, MotorType.kBrushless);
    private final RelativeEncoder  tiltEncoder = tiltMotor.getEncoder();
    private final SparkClosedLoopController tiltPID   = tiltMotor.getClosedLoopController();

    // ---- Limit switch (detects home position) ----
    // DigitalInput.get() returns true when switch is OPEN (not pressed)
    //                             false when switch is CLOSED (pressed)
    // We negate it in getLimitSwitchPressed() so "true" = pressed.
    private final DigitalInput homeLimitSwitch = new DigitalInput(Constants.DIO.INTAKE_HOME_SWITCH);
    private final Debouncer homeLimitSwitchDebouncer =
            new Debouncer(Constants.Intake.HOME_SWITCH_DEBOUNCE_SEC, Debouncer.DebounceType.kBoth);

    // ---- Roller motor (TalonFX / Kraken X60) ----
    private final TalonFX rollerMotor =
            new TalonFX(Constants.CAN.INTAKE_ROLLER, new CANBus(Constants.CAN.CTRE_CAN_BUS));

    // ---- State tracking ----
    // isHomed is false at startup until IntakeHomeCommand confirms the arm
    // has touched the limit switch. We refuse position commands until homed.
    private boolean isHomed = false;
    private boolean homeLimitSwitchRawPressed = false;
    private boolean homeLimitSwitchPressed = false;
    private boolean minSoftLimitLatched = false;
    private boolean maxSoftLimitLatched = false;

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public IntakeSubsystem() {
        // ---- Tilt motor (SparkMax) runtime configuration ----
        // NOTE: We do NOT call restoreFactoryDefaults() or burnFlash().
        // The permanent settings (position conversion factor, etc.) must be
        // configured once in the REV Hardware Client, then they persist.

        SparkMaxConfig tiltConfig = new SparkMaxConfig();

        // Current limit protects the NEO and gearbox during homing stalls
        tiltConfig.smartCurrentLimit(Constants.Intake.TILT_CURRENT_LIMIT_A);

        // Brake mode: arm holds its position when power is removed
        tiltConfig.idleMode(IdleMode.kBrake);

        // Position conversion: sets what unit the encoder reports.
        // If gearbox = 96:1, one motor revolution = 1/96 arm revolution = 3.75°
        // So position in "degrees" = motor_rotations × (360 / gear_ratio)
        // IMPORTANT: This MUST also be set in REV Hardware Client via burnFlash!
        //            We set it here as a safety net at runtime.
        tiltConfig.encoder.positionConversionFactor(Constants.Intake.TILT_POS_CONV_DEG);
        // Velocity unit in deg/sec so MAXMotion can be tuned in arm units.
        tiltConfig.encoder.velocityConversionFactor(Constants.Intake.TILT_POS_CONV_DEG / 60.0);

        // Tilt position PID (built into SparkMax)
        tiltConfig.closedLoop.p(Constants.Intake.TILT_kP);
        tiltConfig.closedLoop.i(0.0);  // no integral — it causes windup in position control
        tiltConfig.closedLoop.d(Constants.Intake.TILT_kD);
        tiltConfig.closedLoop.outputRange(
                -Constants.Intake.TILT_PID_MAX_OUTPUT_DOWN,
                Constants.Intake.TILT_PID_MAX_OUTPUT_UP);
        tiltConfig.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .cruiseVelocity(Constants.Intake.TILT_MAX_MOTION_CRUISE_VEL_DEG_PER_SEC)
                .maxAcceleration(Constants.Intake.TILT_MAX_MOTION_ACCEL_DEG_PER_SEC2)
                .allowedProfileError(Constants.Intake.TILT_MAX_MOTION_ALLOWED_ERROR_DEG);
        tiltMotor.configure(tiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // ---- Roller motor (TalonFX) configuration ----
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

        rollerMotor.getConfigurator().apply(rollerCfg);

        // Reduce CAN status frame rates — intake roller is low-priority.
        // Stator current at 20 Hz for reliable 300ms stall detection window.
        rollerMotor.getStatorCurrent().setUpdateFrequency(20);
        rollerMotor.getVelocity().setUpdateFrequency(4);
        rollerMotor.getPosition().setUpdateFrequency(4);
        rollerMotor.getDeviceTemp().setUpdateFrequency(1);

        updateHomeLimitSwitchState();
    }

    // --------------------------------------------------------------------------
    // periodic() — publish telemetry to SmartDashboard every 20ms
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        updateHomeLimitSwitchState();

        // If we boot while already at the home switch, trust that as a valid zero.
        if (!isHomed && getLimitSwitchPressed()) {
            resetEncoderToHome();
        }

        SmartDashboard.putNumber("Intake/TiltPositionDeg",  tiltEncoder.getPosition());
        SmartDashboard.putBoolean("Intake/IsHomed",         isHomed);
        SmartDashboard.putBoolean("Intake/LimitSwitch",     getLimitSwitchPressed());
        SmartDashboard.putBoolean("Intake/LimitSwitchRaw",  homeLimitSwitchRawPressed);
    }

    // --------------------------------------------------------------------------
    // getLimitSwitchPressed()
    //
    // Returns true when the tilt arm has reached the home (raised) position.
    // Most limit switches are "normally open" — DigitalInput.get() = true when
    // the switch is NOT pressed. We invert this for readability.
    // --------------------------------------------------------------------------
    public boolean getLimitSwitchPressed() {
        return homeLimitSwitchPressed;  // true = switch pressed = arm is home
    }

    // --------------------------------------------------------------------------
    // isHomed()
    //
    // Returns true once IntakeHomeCommand has successfully found the home position.
    // --------------------------------------------------------------------------
    public boolean isHomed() {
        return isHomed;
    }

    public double getTiltPositionDeg() {
        return tiltEncoder.getPosition();
    }

    public double getRollerCurrentAmps() {
        return rollerMotor.getStatorCurrent().getValueAsDouble();
    }

    // --------------------------------------------------------------------------
    // setTiltPower()
    //
    // Directly sets tilt motor power (-1.0 to +1.0) with encoder soft limits.
    // Used by homing/automated safety-controlled flows. Does NOT use PID.
    // --------------------------------------------------------------------------
    public void setTiltPower(double power) {
        setTiltPowerInternal(power, true);
    }

    // --------------------------------------------------------------------------
    // setTiltPowerManual()
    //
    // Manual control bypasses software angle limits for operator authority,
    // but still honors the home switch when commanding toward home.
    // --------------------------------------------------------------------------
    public void setTiltPowerManual(double power) {
        setTiltPowerInternal(power, false);
    }

    private void setTiltPowerInternal(double power, boolean enforceSoftLimits) {
        double clampedPower = Math.max(-1.0, Math.min(1.0, power));
        double homeDirection = Math.signum(Constants.Intake.HOME_POWER);
        boolean commandingTowardHome = homeDirection != 0.0 && (clampedPower * homeDirection) > 0.0;
        boolean commandingAwayFromHome = homeDirection != 0.0 && (clampedPower * homeDirection) < 0.0;

        if (commandingTowardHome && getLimitSwitchPressed()) {
            clampedPower = 0.0;
        }
        if (enforceSoftLimits && isHomed) {
            double positionDeg = getTiltPositionDeg();
            double hysteresisDeg = Constants.Intake.TILT_SOFT_LIMIT_HYSTERESIS_DEG;

            if (positionDeg <= Constants.Intake.TILT_MIN_DEG) {
                minSoftLimitLatched = true;
            } else if (minSoftLimitLatched
                    && positionDeg >= Constants.Intake.TILT_MIN_DEG + hysteresisDeg) {
                minSoftLimitLatched = false;
            }

            if (positionDeg >= Constants.Intake.TILT_MAX_DEG) {
                maxSoftLimitLatched = true;
            } else if (maxSoftLimitLatched
                    && positionDeg <= Constants.Intake.TILT_MAX_DEG - hysteresisDeg) {
                maxSoftLimitLatched = false;
            }

            if (commandingAwayFromHome && minSoftLimitLatched) {
                clampedPower = 0.0;
            }
            if (commandingTowardHome && maxSoftLimitLatched) {
                clampedPower = 0.0;
            }
        } else {
            minSoftLimitLatched = false;
            maxSoftLimitLatched = false;
        }

        tiltMotor.set(clampedPower);
    }

    // --------------------------------------------------------------------------
    // setRollerPower()
    //
    // Spins the intake roller at the given power (-1.0 to +1.0).
    // Positive = intake direction, negative = eject.
    // --------------------------------------------------------------------------
    public void setRollerPower(double power) {
        rollerMotor.set(power);
    }

    // --------------------------------------------------------------------------
    // resetEncoderToHome()
    //
    // Marks the current position as 0 degrees (home/raised).
    // Called by IntakeHomeCommand when the limit switch is triggered.
    // --------------------------------------------------------------------------
    public void resetEncoderToHome() {
        tiltEncoder.setPosition(0.0);
        isHomed = true;
    }

    // --------------------------------------------------------------------------
    // setTiltPosition()
    //
    // Commands the tilt arm to move to a specific angle (in degrees).
    // Only works after homing — if the encoder isn't zeroed, we don't know
    // where "45 degrees" actually is relative to the arm's true position.
    //
    // The target is clamped to [TILT_MIN_DEG, TILT_MAX_DEG] to prevent
    // commanding the arm into the chassis or past its mechanical travel.
    // --------------------------------------------------------------------------
    public void setTiltPosition(double targetDegrees) {
        if (isHomed) {
            double clamped = Math.max(Constants.Intake.TILT_MIN_DEG,
                    Math.min(Constants.Intake.TILT_MAX_DEG, targetDegrees));
            // MAXMotionPositionControl applies trapezoidal profiling on setpoint moves.
            tiltPID.setSetpoint(clamped, ControlType.kMAXMotionPositionControl);
        } else {
            // Safety: refuse to move if we haven't homed yet.
            // Log a warning so the student knows why it's not moving.
            System.out.println("[IntakeSubsystem] WARNING: setTiltPosition called before homing!");
        }
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Immediately stops both tilt and roller motors.
    // Every subsystem should have a stop() for safety and cleanup consistency.
    // --------------------------------------------------------------------------
    public void stop() {
        tiltMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    private void updateHomeLimitSwitchState() {
        homeLimitSwitchRawPressed = !homeLimitSwitch.get();
        homeLimitSwitchPressed = homeLimitSwitchDebouncer.calculate(homeLimitSwitchRawPressed);
    }
}
