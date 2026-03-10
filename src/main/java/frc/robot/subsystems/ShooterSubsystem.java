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

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
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
    private static final double GRAVITY_MPS2 = 9.81;
    private static final int MOVING_SHOT_SOLVE_ITERATIONS = 24;

    private final TalonFX leftShooter =
            new TalonFX(Constants.CAN.SHOOTER_LEFT, new CANBus(Constants.CAN.CTRE_CAN_BUS));
    private final TalonFX rightShooter =
            new TalonFX(Constants.CAN.SHOOTER_RIGHT, new CANBus(Constants.CAN.CTRE_CAN_BUS));

    // VelocityVoltage: tells the motor "spin at exactly X rotations per second"
    // FOC improves efficiency and torque but requires a Phoenix Pro license.
    // Use the same flag as swerve to stay consistent across the robot.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0)
            .withEnableFOC(Constants.Swerve.USE_PHOENIX_PRO_FEATURES);

    // Cached velocity values — updated once per periodic() to avoid redundant CAN reads
    private double cachedLeftRPS = 0;
    private double cachedRightRPS = 0;

    // The currently commanded target RPS (updated by setShooterVelocity).
    // Used by periodic() so the dashboard AtSpeed indicator reflects the actual target,
    // not just the default TARGET_RPS.
    private double currentTargetRPS = Constants.Shooter.TARGET_RPS;

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
        cfg.CurrentLimits.StatorCurrentLimit       = Constants.Shooter.STATOR_CURRENT_LIMIT_A;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = Constants.Shooter.SUPPLY_CURRENT_LIMIT_A;
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

        applyWithRetry(() -> leftShooter.getConfigurator().apply(cfg), "Left shooter config");
        applyWithRetry(() -> rightShooter.getConfigurator().apply(cfg), "Right shooter config");

        // Reduce CAN status frame rates — shooter doesn't need high-frequency updates.
        // Velocity at 10 Hz is enough for isAtSpeed() checks.
        // Position and temperature at 4 Hz — we rarely read these.
        leftShooter.getVelocity().setUpdateFrequency(10);
        leftShooter.getPosition().setUpdateFrequency(4);
        leftShooter.getDeviceTemp().setUpdateFrequency(1);
        rightShooter.getVelocity().setUpdateFrequency(10);
        rightShooter.getPosition().setUpdateFrequency(4);
        rightShooter.getDeviceTemp().setUpdateFrequency(1);
    }

    // --------------------------------------------------------------------------
    // periodic() — called every 20ms
    // Publishes current speed to SmartDashboard for easy tuning.
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        // Cache velocity once per loop — avoids redundant CAN reads in
        // isAtSpeed(), getLeftRPS(), getRightRPS() later this cycle.
        cachedLeftRPS = leftShooter.getVelocity().getValueAsDouble();
        cachedRightRPS = rightShooter.getVelocity().getValueAsDouble();

        SmartDashboard.putNumber("Shooter/LeftRPS", cachedLeftRPS);
        SmartDashboard.putNumber("Shooter/RightRPS", cachedRightRPS);
        SmartDashboard.putBoolean("Shooter/AtSpeed",
                isAtSpeed(currentTargetRPS));
    }

    // --------------------------------------------------------------------------
    // setShooterVelocity()
    //
    // Spins both shooter wheels to the target speed (in RPS).
    // Call this before feeding a game piece so wheels are already up to speed.
    // --------------------------------------------------------------------------
    public void setShooterVelocity(double targetRPS) {
        currentTargetRPS = targetRPS;
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
        return cachedLeftRPS;
    }

    public double getRightRPS() {
        return cachedRightRPS;
    }

    public double getLeftTemperatureC() {
        return leftShooter.getDeviceTemp().getValueAsDouble();
    }

    public double getRightTemperatureC() {
        return rightShooter.getDeviceTemp().getValueAsDouble();
    }

    public record ShotSolution(
            double targetRps,
            double launchSpeedMps,
            double horizontalSpeedMps,
            double timeOfFlightSec,
            boolean feasible) {}

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
        double launchSpeedMps = calculateRequiredLaunchSpeedMps(distanceM);
        if (!Double.isFinite(launchSpeedMps) || launchSpeedMps <= 0) {
            return Constants.Shooter.TARGET_RPS;
        }

        return launchSpeedToMotorRps(launchSpeedMps);
    }

    public static ShotSolution calculateMovingShotSolution(
            double distanceM,
            double radialVelocityMps,
            double lateralVelocityMps) {
        double launchSpeedMps = calculateRequiredLaunchSpeedMps(distanceM);
        if (!Double.isFinite(launchSpeedMps) || launchSpeedMps <= 0) {
            return fallbackShotSolution();
        }

        double minLaunchSpeedMps = motorRpsToLaunchSpeed(Constants.Shooter.MIN_SHOT_RPS);
        double maxLaunchSpeedMps = motorRpsToLaunchSpeed(Constants.Shooter.MAX_SHOT_RPS);
        double highHeightM = calculateVerticalDisplacement(
                maxLaunchSpeedMps, distanceM, radialVelocityMps, lateralVelocityMps);
        double deltaHeightM = Constants.Shooter.HUB_SCORING_HEIGHT_M
                - Constants.Shooter.SHOOTER_EXIT_HEIGHT_M;
        if (!Double.isFinite(highHeightM) || highHeightM < deltaHeightM) {
            return fallbackShotSolution();
        }

        double low = minLaunchSpeedMps;
        double high = maxLaunchSpeedMps;
        for (int i = 0; i < MOVING_SHOT_SOLVE_ITERATIONS; i++) {
            double mid = (low + high) * 0.5;
            double midHeightM = calculateVerticalDisplacement(
                    mid, distanceM, radialVelocityMps, lateralVelocityMps);
            if (!Double.isFinite(midHeightM) || midHeightM < deltaHeightM) {
                low = mid;
            } else {
                high = mid;
            }
        }

        double solvedLaunchSpeedMps = high;
        double horizontalSpeedMps = solvedLaunchSpeedMps
                * Math.cos(Math.toRadians(Constants.Shooter.SHOT_ANGLE_DEG));
        if (Math.abs(lateralVelocityMps) >= horizontalSpeedMps) {
            return fallbackShotSolution();
        }

        double shotLineSpeedMps = Math.sqrt(
                horizontalSpeedMps * horizontalSpeedMps
                        - lateralVelocityMps * lateralVelocityMps)
                + radialVelocityMps;
        if (shotLineSpeedMps <= 1e-6) {
            return fallbackShotSolution();
        }

        return new ShotSolution(
                launchSpeedToMotorRps(solvedLaunchSpeedMps),
                solvedLaunchSpeedMps,
                horizontalSpeedMps,
                distanceM / shotLineSpeedMps,
                true);
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Stops both shooter motors (they will coast to a stop).
    // --------------------------------------------------------------------------
    public void stop() {
        currentTargetRPS = Constants.Shooter.TARGET_RPS;
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }

    // --------------------------------------------------------------------------
    // CAN config retry logic — matches swerve module pattern
    // --------------------------------------------------------------------------
    private static final int CONFIG_APPLY_RETRIES = 5;

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
                    System.out.println("[ShooterSubsystem] " + action + " succeeded on attempt " + (i + 1));
                }
                return;
            }
            System.out.println("[ShooterSubsystem] " + action + " attempt " + (i + 1)
                    + " failed: " + lastCode.getName());
        }
        System.err.println("[ShooterSubsystem] ERROR: " + action + " failed after "
                + CONFIG_APPLY_RETRIES + " attempts. Last status: " + lastCode.getName());
    }

    private static double calculateRequiredLaunchSpeedMps(double distanceM) {
        if (!Double.isFinite(distanceM) || distanceM <= 0) {
            return Double.NaN;
        }

        double angleRad = Math.toRadians(Constants.Shooter.SHOT_ANGLE_DEG);
        double cosAngle = Math.cos(angleRad);
        double tanAngle = Math.tan(angleRad);
        double deltaH = Constants.Shooter.HUB_SCORING_HEIGHT_M
                - Constants.Shooter.SHOOTER_EXIT_HEIGHT_M;

        double denom = tanAngle * distanceM - deltaH;
        if (denom <= 0) {
            return Double.NaN;
        }

        double vSquared = (GRAVITY_MPS2 * distanceM * distanceM)
                / (2.0 * cosAngle * cosAngle * denom);
        if (!Double.isFinite(vSquared) || vSquared <= 0) {
            return Double.NaN;
        }
        return Math.sqrt(vSquared);
    }

    private static double calculateVerticalDisplacement(
            double launchSpeedMps,
            double distanceM,
            double radialVelocityMps,
            double lateralVelocityMps) {
        if (!Double.isFinite(launchSpeedMps) || launchSpeedMps <= 0
                || !Double.isFinite(distanceM) || distanceM <= 0) {
            return Double.NaN;
        }

        double angleRad = Math.toRadians(Constants.Shooter.SHOT_ANGLE_DEG);
        double horizontalSpeedMps = launchSpeedMps * Math.cos(angleRad);
        if (Math.abs(lateralVelocityMps) >= horizontalSpeedMps) {
            return Double.NaN;
        }

        double shotLineSpeedMps = Math.sqrt(
                horizontalSpeedMps * horizontalSpeedMps
                        - lateralVelocityMps * lateralVelocityMps)
                + radialVelocityMps;
        if (shotLineSpeedMps <= 1e-6) {
            return Double.NaN;
        }

        double timeOfFlightSec = distanceM / shotLineSpeedMps;
        double verticalSpeedMps = launchSpeedMps * Math.sin(angleRad);
        return verticalSpeedMps * timeOfFlightSec
                - 0.5 * GRAVITY_MPS2 * timeOfFlightSec * timeOfFlightSec;
    }

    private static double launchSpeedToMotorRps(double launchSpeedMps) {
        double wheelRps = launchSpeedMps / Constants.Shooter.WHEEL_CIRCUMFERENCE_M;
        double motorRps = wheelRps * Constants.Shooter.GEAR_RATIO;
        return MathUtil.clamp(
                motorRps,
                Constants.Shooter.MIN_SHOT_RPS,
                Constants.Shooter.MAX_SHOT_RPS);
    }

    private static double motorRpsToLaunchSpeed(double motorRps) {
        return motorRps / Constants.Shooter.GEAR_RATIO
                * Constants.Shooter.WHEEL_CIRCUMFERENCE_M;
    }

    private static ShotSolution fallbackShotSolution() {
        return new ShotSolution(
                Constants.Shooter.TARGET_RPS,
                motorRpsToLaunchSpeed(Constants.Shooter.TARGET_RPS),
                motorRpsToLaunchSpeed(Constants.Shooter.TARGET_RPS)
                        * Math.cos(Math.toRadians(Constants.Shooter.SHOT_ANGLE_DEG)),
                Double.NaN,
                false);
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

                // Log if we timed out without reaching target speed
                Commands.runOnce(() -> {
                    if (!isAtSpeed(targetRPS)) {
                        System.out.println("[ShootRoutine] WARNING: Feeding at "
                                + String.format("%.1f", Math.abs(getLeftRPS()))
                                + "/" + String.format("%.1f", Math.abs(getRightRPS()))
                                + " RPS, target was " + String.format("%.1f", targetRPS));
                        SmartDashboard.putBoolean("Shooter/FedBelowSpeed", true);
                    } else {
                        SmartDashboard.putBoolean("Shooter/FedBelowSpeed", false);
                    }
                }),

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
