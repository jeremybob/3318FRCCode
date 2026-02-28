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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // ---- Roller motor (TalonFX / Kraken X60) ----
    private final TalonFX rollerMotor = new TalonFX(Constants.CAN.INTAKE_ROLLER, Constants.CAN.CTRE_CAN_BUS);

    // ---- State tracking ----
    // isHomed is false at startup until IntakeHomeCommand confirms the arm
    // has touched the limit switch. We refuse position commands until homed.
    private boolean isHomed = false;

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
        // If gearbox = 10:1, one motor revolution = 1/10 arm revolution = 36°
        // So position in "degrees" = motor_rotations × (360 / gear_ratio)
        // IMPORTANT: This MUST also be set in REV Hardware Client via burnFlash!
        //            We set it here as a safety net at runtime.
        tiltConfig.encoder.positionConversionFactor(Constants.Intake.TILT_POS_CONV_DEG);

        // Tilt position PID (built into SparkMax)
        tiltConfig.closedLoop.p(Constants.Intake.TILT_kP);
        tiltConfig.closedLoop.i(0.0);  // no integral — it causes windup in position control
        tiltConfig.closedLoop.d(0.0);  // add D if you see oscillation
        tiltMotor.configure(tiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // ---- Roller motor (TalonFX) configuration ----
        TalonFXConfiguration rollerCfg = new TalonFXConfiguration();

        // Coast mode: roller can spin freely after power removed (doesn't jam)
        rollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // === CRITICAL: Current limiting prevents motor damage during game piece jams ===
        // Without this, the Kraken can burn out or trip a breaker if the intake stalls.
        rollerCfg.CurrentLimits.StatorCurrentLimit       = Constants.Intake.ROLLER_STATOR_LIMIT_A;
        rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerCfg.CurrentLimits.SupplyCurrentLimit       = 40;
        rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerMotor.getConfigurator().apply(rollerCfg);
    }

    // --------------------------------------------------------------------------
    // periodic() — publish telemetry to SmartDashboard every 20ms
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        // If we boot while already at the home switch, trust that as a valid zero.
        if (!isHomed && getLimitSwitchPressed()) {
            resetEncoderToHome();
        }

        SmartDashboard.putNumber("Intake/TiltPositionDeg",  tiltEncoder.getPosition());
        SmartDashboard.putBoolean("Intake/IsHomed",         isHomed);
        SmartDashboard.putBoolean("Intake/LimitSwitch",     getLimitSwitchPressed());
    }

    // --------------------------------------------------------------------------
    // getLimitSwitchPressed()
    //
    // Returns true when the tilt arm has reached the home (raised) position.
    // Most limit switches are "normally open" — DigitalInput.get() = true when
    // the switch is NOT pressed. We invert this for readability.
    // --------------------------------------------------------------------------
    public boolean getLimitSwitchPressed() {
        return !homeLimitSwitch.get();  // true = switch pressed = arm is home
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
    // Directly sets tilt motor power (-1.0 to +1.0).
    // Used during homing and manual control. Does NOT use PID.
    // --------------------------------------------------------------------------
    public void setTiltPower(double power) {
        tiltMotor.set(power);
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
            // kPosition tells the SparkMax to run its built-in position PID loop
            tiltPID.setSetpoint(clamped, ControlType.kPosition);
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
}
