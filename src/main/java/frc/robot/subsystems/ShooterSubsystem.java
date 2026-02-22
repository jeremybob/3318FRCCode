// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/ShooterSubsystem.java
//
// PURPOSE: Controls the two shooter wheels that launch game pieces.
//   Hardware: Two Kraken X60 (TalonFX), 4-inch wheels, 1:1 gearing
//
// HOW IT WORKS:
//   - Left and right wheels spin at the same target speed (right is inverted).
//   - We use velocity closed-loop control so the wheels reach and hold a
//     precise RPS even as the battery voltage drops during a match.
//   - isAtSpeed() checks whether the wheels are within tolerance before feeding.
// ============================================================================
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftShooter  = new TalonFX(Constants.CAN.SHOOTER_LEFT);
    private final TalonFX rightShooter = new TalonFX(Constants.CAN.SHOOTER_RIGHT);

    // VelocityVoltage: tells the motor "spin at exactly X rotations per second"
    // enableFOC = true improves efficiency and torque with Kraken's TorqueCurrentFOC
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0)
            .withEnableFOC(true);

    // --------------------------------------------------------------------------
    // Constructor: configure both shooter motors identically
    // --------------------------------------------------------------------------
    public ShooterSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Coast mode: wheels can keep spinning freely when we stop commanding them.
        // This is intentional — we don't want the shooter to brake and slow down
        // mid-shot.
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limiting protects the motors from stall damage
        cfg.CurrentLimits.StatorCurrentLimit       = 80;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = 60;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity PID (Slot 0)
        // kS: static friction offset — minimum voltage to get the wheel moving
        // kV: feedforward gain — 12V ÷ free-speed-RPS = 12 ÷ 100 = 0.12
        //     This is the main driver of the wheel speed. Get this right first.
        // kP: small correction term — adds voltage when actual speed ≠ target
        //     Start low; too high causes oscillation / speed hunting.
        cfg.Slot0.kS = Constants.Shooter.SHOOTER_kS;   // TUNE ME
        cfg.Slot0.kV = Constants.Shooter.SHOOTER_kV;   // 0.12 for Kraken at 12V
        cfg.Slot0.kP = Constants.Shooter.SHOOTER_kP;   // TUNE ME

        leftShooter.getConfigurator().apply(cfg);
        rightShooter.getConfigurator().apply(cfg);

        // Right wheel command sign is flipped in setShooterVelocity().
    }

    // --------------------------------------------------------------------------
    // periodic() — called every 20ms
    // Publishes current speed to SmartDashboard for easy tuning.
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/LeftRPS",
                leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/RightRPS",
                rightShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/AtSpeed",
                isAtSpeed(Constants.Shooter.TARGET_RPS));
    }

    // --------------------------------------------------------------------------
    // setShooterVelocity()
    //
    // Spins both shooter wheels to the target speed (in RPS).
    // Call this before feeding a game piece so wheels are already up to speed.
    // --------------------------------------------------------------------------
    public void setShooterVelocity(double targetRPS) {
        leftShooter.setControl(velocityRequest.withVelocity(targetRPS));
        rightShooter.setControl(velocityRequest.withVelocity(-targetRPS));
    }

    // --------------------------------------------------------------------------
    // isAtSpeed()
    //
    // Returns true when BOTH wheels are within tolerance of the target RPS.
    // Used by shoot routines to wait until wheels are ready before feeding.
    // --------------------------------------------------------------------------
    public boolean isAtSpeed(double targetRPS) {
        double leftRPS  = Math.abs(getLeftRPS());
        double rightRPS = Math.abs(getRightRPS());
        return Math.abs(leftRPS  - targetRPS) <= Constants.Shooter.TOLERANCE_RPS
            && Math.abs(rightRPS - targetRPS) <= Constants.Shooter.TOLERANCE_RPS;
    }

    public double getLeftRPS() {
        return leftShooter.getVelocity().getValueAsDouble();
    }

    public double getRightRPS() {
        return rightShooter.getVelocity().getValueAsDouble();
    }

    // --------------------------------------------------------------------------
    // calculateTargetRPS()
    //
    // Solves projectile motion for the required wheel RPS given the horizontal
    // distance to the HUB.  Assumes a fixed launch angle (configurable in
    // Constants.Shooter.SHOT_ANGLE_DEG) and known shooter/target heights.
    //
    // Physics:
    //   v² = (g × d²) / (2 × cos²θ × (d × tanθ − Δh))
    //   where Δh = target_height − shooter_exit_height
    //
    // Then convert surface speed (m/s) to wheel RPS:
    //   RPS = surface_speed / wheel_circumference / gear_ratio
    //
    // Returns TARGET_RPS as fallback if distance is invalid or the shot is
    // physically impossible at the configured angle.
    // --------------------------------------------------------------------------
    public static double calculateTargetRPS(double distanceM) {
        if (!Double.isFinite(distanceM) || distanceM <= 0) {
            return Constants.Shooter.TARGET_RPS;
        }

        double angleRad = Math.toRadians(Constants.Shooter.SHOT_ANGLE_DEG);
        double cosAngle = Math.cos(angleRad);
        double tanAngle = Math.tan(angleRad);
        double deltaH   = Constants.Shooter.HUB_SCORING_HEIGHT_M
                         - Constants.Shooter.SHOOTER_EXIT_HEIGHT_M;

        // denominator = d × tanθ − Δh
        // If ≤ 0, the shot arc can't reach the target at this angle.
        double denom = tanAngle * distanceM - deltaH;
        if (denom <= 0) {
            return Constants.Shooter.TARGET_RPS;
        }

        double g = 9.81; // m/s²
        double vSquared = (g * distanceM * distanceM)
                        / (2.0 * cosAngle * cosAngle * denom);
        double surfaceSpeed = Math.sqrt(vSquared); // m/s

        // Convert surface speed to wheel RPS (accounting for gear ratio)
        double wheelRPS = surfaceSpeed / Constants.Shooter.WHEEL_CIRCUMFERENCE_M;
        double motorRPS = wheelRPS * Constants.Shooter.GEAR_RATIO;

        return MathUtil.clamp(motorRPS,
                Constants.Shooter.MIN_SHOT_RPS,
                Constants.Shooter.MAX_SHOT_RPS);
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Stops both shooter motors (they will coast to a stop).
    // --------------------------------------------------------------------------
    public void stop() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }

    // --------------------------------------------------------------------------
    // buildShootRoutine()
    //
    // Creates a sequential command that performs a full shoot cycle:
    //   1. Start spinning shooter wheels to target speed
    //   2. Briefly reverse the feeder to clear any jammed game piece
    //   3. Wait for shooter wheels to reach speed (or timeout)
    //   4. Feed game piece through hopper + feeder + intake roller
    //   5. Stop everything when done
    //
    // NOTE: This command owns the shooter, feeder, hopper, AND intake during
    //       execution. No other command can use those subsystems simultaneously.
    // --------------------------------------------------------------------------
    public Command buildShootRoutine(FeederSubsystem feeder,
                                     HopperSubsystem hopper,
                                     IntakeSubsystem intake,
                                     double targetRPS) {
        return Commands.sequence(
                // Step 1: Start spinning wheels immediately
                Commands.runOnce(() -> setShooterVelocity(targetRPS), this),

                // Step 2: Clear any double-fed game piece with a brief reverse pulse
                Commands.run(() -> feeder.setPower(Constants.Shooter.CLEAR_POWER), feeder)
                        .withTimeout(Constants.Shooter.CLEAR_TIME_SEC),
                Commands.runOnce(feeder::stop, feeder),

                // Step 3: Wait for wheels to reach speed (bail after timeout)
                Commands.waitUntil(() -> isAtSpeed(targetRPS))
                        .withTimeout(Constants.Shooter.AT_SPEED_TIMEOUT_SEC),

                // Step 4: Feed the game piece — run all three feed mechanisms together
                Commands.parallel(
                        Commands.run(() -> feeder.setPower(Constants.Shooter.FEED_POWER), feeder),
                        Commands.run(() -> hopper.setPower(Constants.Shooter.FEED_POWER), hopper),
                        Commands.run(() -> intake.setRollerPower(Constants.Shooter.FEED_POWER), intake)
                ).withTimeout(Constants.Shooter.FEED_TIME_SEC)

        // finallyDo runs whether the command finished normally OR was interrupted
        // (e.g., driver pressed a cancel button). Always leaves things in a safe state.
        ).finallyDo(() -> {
            stop();
            feeder.stop();
            hopper.stop();
            intake.setRollerPower(0);
        });
    }
}
