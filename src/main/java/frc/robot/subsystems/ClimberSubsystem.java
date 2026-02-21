// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/ClimberSubsystem.java
//
// PURPOSE: Controls the climber winch that lifts the robot at the end of a match.
//
// Hardware: Two TalonFX (Kraken X60) with one leader + one follower.
//   The follower copies the leader's output automatically.
//
// KEY FIXES FROM v1:
//   1. StrictFollower is stored and reapplied in stop(). This prevents the
//      follower from getting "stuck" in a neutral state if the leader changes
//      control modes.
//   2. stop() now explicitly puts BOTH motors back into follower mode or stops
//      the leader (follower stops automatically when leader does).
//   3. Added manual power scaling so the operator can climb carefully.
//   4. Added telemetry so the team can monitor winch position during endgame.
// ============================================================================
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX leaderWinch   = new TalonFX(Constants.CAN.CLIMBER_LEADER);
    private final TalonFX followerWinch = new TalonFX(Constants.CAN.CLIMBER_FOLLOWER);

    // Position control request — used for the automatic Level 1 climb
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    // Store the follower control request so we can re-apply it if needed.
    // StrictFollower means the follower exactly copies the leader's output.
    private final StrictFollower followRequest =
            new StrictFollower(Constants.CAN.CLIMBER_LEADER);

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public ClimberSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Brake mode: robot holds its position when winch is not commanded.
        // This is CRITICAL for climbing — without it, the robot would fall.
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Software limits prevent the winch from over-extending or over-retracting.
        // If the winch hits a hardware limit without these, it will stall and potentially
        // damage the mechanism or trip a breaker.
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FWD_SOFT_LIMIT;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REV_SOFT_LIMIT;

        // Current limiting: the winch pulls hard, but we need to cap it to
        // avoid tripping breakers or destroying the gearbox.
        cfg.CurrentLimits.StatorCurrentLimit       = Constants.Climber.STATOR_LIMIT_A;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Position PID (for the auto-climb feature)
        cfg.Slot0.kP = Constants.Climber.CLIMBER_kP;
        cfg.Slot0.kD = Constants.Climber.CLIMBER_kD;

        // Apply the same config to both motors
        leaderWinch.getConfigurator().apply(cfg);
        followerWinch.getConfigurator().apply(cfg);

        // Tell the follower to mirror the leader.
        // The follower will automatically match whatever the leader outputs.
        followerWinch.setControl(followRequest);
    }

    // --------------------------------------------------------------------------
    // periodic() — publish winch position to dashboard for endgame monitoring
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/WinchPositionRot",
                leaderWinch.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber/CurrentAmps",
                leaderWinch.getStatorCurrent().getValueAsDouble());
    }

    // --------------------------------------------------------------------------
    // setWinchPower()
    //
    // Manually controls the winch with direct power from the operator joystick.
    // The follower motor automatically mirrors whatever we command the leader.
    //
    // Parameters:
    //   power - value from -1.0 to +1.0
    //           positive = extend (robot goes up)
    //           negative = retract
    // --------------------------------------------------------------------------
    public void setWinchPower(double power) {
        // Scale by MANUAL_POWER_SCALE so the operator can control carefully.
        // Full stick doesn't mean full power — prevents dangerous sudden movements.
        leaderWinch.set(power * Constants.Climber.MANUAL_POWER_SCALE);
        // Note: followerWinch copies leaderWinch automatically via StrictFollower
    }

    // --------------------------------------------------------------------------
    // autoClimbLevel1()
    //
    // Commands the winch to a specific position for a Level 1 automatic climb.
    // Uses the position PID loop — call this in auto or as a button trigger.
    // --------------------------------------------------------------------------
    public void autoClimbLevel1() {
        leaderWinch.setControl(
                positionRequest.withPosition(Constants.Climber.LEVEL1_TARGET_ROT));
        // Follower automatically copies the leader's output
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Stops the leader (follower will stop too since it copies the leader).
    // Re-apply StrictFollower so it's ready for the next movement command.
    // --------------------------------------------------------------------------
    public void stop() {
        leaderWinch.stopMotor();
        // Re-apply follow control so follower is always in the correct state.
        // This prevents the follower from getting "stuck" in a neutral control
        // mode after the leader stops.
        followerWinch.setControl(followRequest);
    }
}
