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

        // Reduce CAN status frame rates without starving the ready-to-shoot gate.
        // Velocity needs low enough latency that isAtSpeed() can react promptly.
        // Position and temperature at 4 Hz — we rarely read these.
        leftShooter.getVelocity().setUpdateFrequency(Constants.Shooter.VELOCITY_SIGNAL_HZ);
        leftShooter.getPosition().setUpdateFrequency(4);
        leftShooter.getDeviceTemp().setUpdateFrequency(1);
        rightShooter.getVelocity().setUpdateFrequency(Constants.Shooter.VELOCITY_SIGNAL_HZ);
        rightShooter.getPosition().setUpdateFrequency(4);
        rightShooter.getDeviceTemp().setUpdateFrequency(1);
        applyWithRetry(
                leftShooter::optimizeBusUtilization,
                "Left shooter bus optimization (id=" + Constants.CAN.SHOOTER_LEFT + ")");
        applyWithRetry(
                rightShooter::optimizeBusUtilization,
                "Right shooter bus optimization (id=" + Constants.CAN.SHOOTER_RIGHT + ")");
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

    public static double manualStickToTargetRps(double rawStickInput) {
        double filteredInput = MathUtil.applyDeadband(
                rawStickInput,
                Constants.Shooter.MANUAL_SPEED_DEADBAND);
        return Math.max(0.0, filteredInput) * Constants.Shooter.MANUAL_MAX_RPS;
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
    // Uses the robot's measured close shot and the existing 60 RPS warmup value
    // as anchors for an empirical distance-to-speed curve.
    // --------------------------------------------------------------------------
    public static double calculateTargetRPS(double distanceM) {
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return Constants.Shooter.TARGET_RPS;
        }
        return calculateEmpiricalTargetRps(distanceM);
    }

    public static ShotSolution calculateMovingShotSolution(
            double distanceM,
            double radialVelocityMps,
            double lateralVelocityMps) {
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return fallbackShotSolution();
        }

        double stationaryTargetRps = calculateTargetRPS(distanceM);
        if (!Double.isFinite(stationaryTargetRps) || stationaryTargetRps <= 0.0) {
            return fallbackShotSolution();
        }

        double angleRad = Math.toRadians(Constants.Shooter.SHOT_ANGLE_DEG);
        double stationaryLaunchSpeedMps = motorRpsToLaunchSpeed(stationaryTargetRps);
        double stationaryHorizontalSpeedMps = stationaryLaunchSpeedMps * Math.cos(angleRad);
        double requiredShotLineSpeedMps = stationaryHorizontalSpeedMps + radialVelocityMps;
        if (!Double.isFinite(requiredShotLineSpeedMps) || requiredShotLineSpeedMps <= 1e-6) {
            return fallbackShotSolution();
        }
        double requiredHorizontalSpeedMps = Math.hypot(requiredShotLineSpeedMps, lateralVelocityMps);
        double solvedLaunchSpeedMps = requiredHorizontalSpeedMps / Math.cos(angleRad);
        if (!Double.isFinite(solvedLaunchSpeedMps) || solvedLaunchSpeedMps <= 0.0) {
            return fallbackShotSolution();
        }
        double targetRps = launchSpeedToMotorRps(solvedLaunchSpeedMps);

        return new ShotSolution(
                targetRps,
                solvedLaunchSpeedMps,
                requiredHorizontalSpeedMps,
                distanceM / requiredShotLineSpeedMps,
                true);
    }

    // --------------------------------------------------------------------------
    // stop()
    //
    // Stops both shooter motors (they will coast to a stop).
    // --------------------------------------------------------------------------
    public void stop() {
        currentTargetRPS = 0.0;
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

    private static double calculateEmpiricalTargetRps(double distanceM) {
        double extraDistanceM = Math.max(
                0.0,
                distanceM - Constants.Shooter.MEASURED_CLOSE_SHOT_DISTANCE_M);
        double targetRps = Constants.Shooter.FALLBACK_RPS
                + extraDistanceM * Constants.Shooter.EMPIRICAL_SHOT_SLOPE_RPS_PER_M;
        return MathUtil.clamp(
                targetRps,
                Constants.Shooter.MIN_SHOT_RPS,
                Constants.Shooter.MAX_SHOT_RPS);
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

    public Command buildContinuousShootRoutine(FeederSubsystem feeder,
                                               HopperSubsystem hopper,
                                               IntakeSubsystem intake,
                                               double targetRPS) {
        return Commands.sequence(
                Commands.runOnce(() -> setShooterVelocity(targetRPS), this),

                Commands.run(() -> feeder.setPower(Constants.Shooter.CLEAR_POWER), feeder)
                        .withTimeout(Constants.Shooter.CLEAR_TIME_SEC),
                Commands.runOnce(feeder::stop, feeder),

                Commands.waitUntil(() -> isAtSpeed(targetRPS))
                        .withTimeout(Constants.Shooter.AT_SPEED_TIMEOUT_SEC),

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

                Commands.parallel(
                        Commands.run(() -> feeder.setPower(Constants.Shooter.FEED_POWER), feeder),
                        Commands.run(() -> hopper.setPower(Constants.Shooter.FEED_POWER), hopper),
                        Commands.run(() -> intake.setRollerPower(Constants.Shooter.FEED_POWER), intake)
                )
        ).finallyDo(() -> {
            stop();
            feeder.stop();
            hopper.stop();
            intake.setRollerPower(0);
        });
    }
}
