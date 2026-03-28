// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/ShooterSubsystem.java
//
// PURPOSE: Controls the two shooter motors that launch game pieces.
//   Hardware: Two Kraken X60 (TalonFX) on a single shared shaft, 4-inch
//   wheels, 1:1 gearing. Motors are mounted facing opposite directions.
//
// HOW IT WORKS:
//   - Left motor is the LEADER running velocity closed-loop control.
//   - Right motor is a FOLLOWER that mirrors the leader's output with
//     opposite direction (since the motors face each other on the shaft).
//   - A single PID loop controls both motors, eliminating the dual-loop
//     fighting problem that occurs when two independent controllers drive
//     the same mechanism.
//   - isAtSpeed() checks the leader's velocity before feeding.
// ============================================================================
package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.ShotSolver;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftShooter =
            new TalonFX(Constants.CAN.SHOOTER_LEFT, new CANBus(Constants.CAN.RIO_CAN_BUS));
    private final TalonFX rightShooter =
            new TalonFX(Constants.CAN.SHOOTER_RIGHT, new CANBus(Constants.CAN.RIO_CAN_BUS));

    // VelocityVoltage: tells the motor "spin at exactly X rotations per second"
    // FOC improves efficiency and torque but requires a Phoenix Pro license.
    // Use the same flag as swerve to stay consistent across the robot.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0)
            .withEnableFOC(Constants.Swerve.USE_PHOENIX_PRO_FEATURES);

    @SuppressWarnings("rawtypes")
    private final StatusSignal leftVelocitySignal = leftShooter.getVelocity();
    @SuppressWarnings("rawtypes")
    private final StatusSignal rightVelocitySignal = rightShooter.getVelocity();
    @SuppressWarnings("rawtypes")
    private final StatusSignal leftTempSignal = leftShooter.getDeviceTemp();
    @SuppressWarnings("rawtypes")
    private final StatusSignal rightTempSignal = rightShooter.getDeviceTemp();

    // Cached velocity values — updated once per periodic() to avoid redundant CAN reads
    private double cachedLeftRPS = 0;
    private double cachedRightRPS = 0;
    private double cachedLeftTempC = 0;
    private double cachedRightTempC = 0;

    // The currently commanded target RPS (updated by setShooterVelocity).
    // Used by periodic() so the dashboard AtSpeed indicator reflects the actual target,
    // not just the default TARGET_RPS.
    private double currentTargetRPS = Constants.Shooter.TARGET_RPS;

    // --------------------------------------------------------------------------
    // Constructor: configure leader + follower shooter motors
    // --------------------------------------------------------------------------
    public ShooterSubsystem() {
        // ---- Leader (left) configuration ----
        TalonFXConfiguration leaderCfg = new TalonFXConfiguration();

        // Coast mode: shaft keeps spinning freely when we stop commanding.
        // Intentional — we don't want to brake and slow down mid-shot.
        leaderCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limiting protects the motors from stall damage
        leaderCfg.CurrentLimits.StatorCurrentLimit       = Constants.Shooter.STATOR_CURRENT_LIMIT_A;
        leaderCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        leaderCfg.CurrentLimits.SupplyCurrentLimit       = Constants.Shooter.SUPPLY_CURRENT_LIMIT_A;
        leaderCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity PID (Slot 0) — only runs on the leader
        // kS: static friction offset — minimum voltage to get the shaft moving
        // kV: feedforward gain — 12V ÷ free-speed-RPS = 12 ÷ 100 = 0.12
        //     This is the main driver of the shaft speed. Get this right first.
        // kP: small correction term — adds voltage when actual speed ≠ target
        //     Start low; too high causes oscillation / speed hunting.
        leaderCfg.Slot0.kS = Constants.Shooter.SHOOTER_kS;   // TUNE ME
        leaderCfg.Slot0.kV = Constants.Shooter.SHOOTER_kV;   // 0.12 for Kraken at 12V
        leaderCfg.Slot0.kP = Constants.Shooter.SHOOTER_kP;   // TUNE ME

        applyWithRetry(() -> leftShooter.getConfigurator().apply(leaderCfg), "Left shooter (leader) config");

        // ---- Follower (right) configuration ----
        // The follower only needs current limits and neutral mode — PID gains are
        // irrelevant since it mirrors the leader's output directly.
        TalonFXConfiguration followerCfg = new TalonFXConfiguration();
        followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerCfg.CurrentLimits.StatorCurrentLimit       = Constants.Shooter.STATOR_CURRENT_LIMIT_A;
        followerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        followerCfg.CurrentLimits.SupplyCurrentLimit       = Constants.Shooter.SUPPLY_CURRENT_LIMIT_A;
        followerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        applyWithRetry(() -> rightShooter.getConfigurator().apply(followerCfg), "Right shooter (follower) config");

        // Set right motor as follower with opposed direction.
        // Motors face opposite directions on the shared shaft, so opposing the
        // master makes them apply torque in the same physical direction.
        rightShooter.setControl(new Follower(Constants.CAN.SHOOTER_LEFT, true));

        // Reduce CAN status frame rates without starving the ready-to-shoot gate.
        // Only the leader needs high-rate velocity for isAtSpeed() checks.
        // The follower's velocity is monitored at a lower rate for diagnostics only.
        leftVelocitySignal.setUpdateFrequency(Constants.Shooter.VELOCITY_SIGNAL_HZ);
        leftShooter.getPosition().setUpdateFrequency(4);
        leftTempSignal.setUpdateFrequency(1);
        rightVelocitySignal.setUpdateFrequency(4);
        rightShooter.getPosition().setUpdateFrequency(4);
        rightTempSignal.setUpdateFrequency(1);
        applyWithRetry(
                leftShooter::optimizeBusUtilization,
                "Left shooter bus optimization (id=" + Constants.CAN.SHOOTER_LEFT + ")");
        applyWithRetry(
                rightShooter::optimizeBusUtilization,
                "Right shooter bus optimization (id=" + Constants.CAN.SHOOTER_RIGHT + ")");

        cachedLeftTempC = leftTempSignal.getValueAsDouble();
        cachedRightTempC = rightTempSignal.getValueAsDouble();
    }

    // --------------------------------------------------------------------------
    // periodic() — called every 20ms
    // Publishes current speed to SmartDashboard for easy tuning.
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        // Cache velocity once per loop — avoids redundant CAN reads in
        // isAtSpeed(), getLeftRPS(), getRightRPS() later this cycle.
        cachedLeftRPS = leftVelocitySignal.getValueAsDouble();
        cachedRightRPS = rightVelocitySignal.getValueAsDouble();
        cachedLeftTempC = leftTempSignal.getValueAsDouble();
        cachedRightTempC = rightTempSignal.getValueAsDouble();

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
        // Only command the leader — the follower mirrors it automatically.
        leftShooter.setControl(velocityRequest.withVelocity(targetRPS));
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
    // Returns true when the leader wheel is within tolerance of the target RPS.
    // The follower is mechanically locked to the same shaft, so checking one
    // motor is sufficient. Used by shoot routines to gate feeding.
    // --------------------------------------------------------------------------
    public boolean isAtSpeed(double targetRPS) {
        return Math.abs(Math.abs(getLeftRPS()) - targetRPS) <= Constants.Shooter.TOLERANCE_RPS;
    }

    public double getLeftRPS() {
        return cachedLeftRPS;
    }

    public double getRightRPS() {
        return cachedRightRPS;
    }

    public double getLeftTemperatureC() {
        return cachedLeftTempC;
    }

    public double getRightTemperatureC() {
        return cachedRightTempC;
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
    // Returns the motor RPS required to reach the target at the given distance.
    // Delegates to ShotSolver which uses projectile physics to compute a
    // coupled (angle, speed) pair — the speed component is returned here.
    // --------------------------------------------------------------------------
    public static double calculateTargetRPS(double distanceM) {
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return Constants.Shooter.TARGET_RPS;
        }
        ShotSolver.Solution solution = ShotSolver.solve(distanceM);
        if (!solution.feasible()) {
            return Constants.Shooter.TARGET_RPS;
        }
        return MathUtil.clamp(
                solution.motorRps(),
                Constants.Shooter.MIN_SHOT_RPS,
                Constants.Shooter.MAX_SHOT_RPS);
    }

    public static ShotSolution calculateMovingShotSolution(
            double distanceM,
            double radialVelocityMps,
            double lateralVelocityMps) {
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return fallbackShotSolution();
        }

        // Get the physics-based stationary solution (coupled angle + speed)
        ShotSolver.Solution base = ShotSolver.solve(distanceM);
        if (!base.feasible()) {
            return fallbackShotSolution();
        }

        double angleRad = Math.toRadians(base.angleDeg());
        double stationaryHorizontalSpeedMps = base.launchSpeedMps() * Math.cos(angleRad);
        double requiredShotLineSpeedMps = stationaryHorizontalSpeedMps + radialVelocityMps;
        if (!Double.isFinite(requiredShotLineSpeedMps) || requiredShotLineSpeedMps <= 1e-6) {
            return fallbackShotSolution();
        }
        double requiredHorizontalSpeedMps = Math.hypot(requiredShotLineSpeedMps, lateralVelocityMps);
        double solvedLaunchSpeedMps = requiredHorizontalSpeedMps / Math.cos(angleRad);
        if (!Double.isFinite(solvedLaunchSpeedMps) || solvedLaunchSpeedMps <= 0.0) {
            return fallbackShotSolution();
        }
        double targetRps = ShotSolver.launchSpeedToMotorRps(solvedLaunchSpeedMps);

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
        // stopMotor() sends a NeutralOut control request which overrides the
        // Follower state. Re-establish follower so the right motor resumes
        // mirroring the leader on the next setShooterVelocity() call.
        rightShooter.setControl(new Follower(Constants.CAN.SHOOTER_LEFT, true));
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

    private static ShotSolution fallbackShotSolution() {
        double launchSpeed = ShotSolver.motorRpsToLaunchSpeed(Constants.Shooter.TARGET_RPS);
        return new ShotSolution(
                Constants.Shooter.TARGET_RPS,
                launchSpeed,
                launchSpeed * Math.cos(Math.toRadians(Constants.Hood.DEFAULT_ANGLE_DEG)),
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
                                     HoodSubsystem hood,
                                     double targetRPS,
                                     double hoodAngleDeg) {
        return Commands.sequence(
                // Step 1: Start spinning wheels and set hood angle immediately
                Commands.runOnce(() -> {
                    setShooterVelocity(targetRPS);
                    hood.setAngle(hoodAngleDeg);
                }, this, hood),

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
            hood.setDefault();
        });
    }

    public Command buildContinuousShootRoutine(FeederSubsystem feeder,
                                               HopperSubsystem hopper,
                                               IntakeSubsystem intake,
                                               HoodSubsystem hood,
                                               double targetRPS,
                                               double hoodAngleDeg) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    setShooterVelocity(targetRPS);
                    hood.setAngle(hoodAngleDeg);
                }, this, hood),

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
            hood.setDefault();
        });
    }
}
