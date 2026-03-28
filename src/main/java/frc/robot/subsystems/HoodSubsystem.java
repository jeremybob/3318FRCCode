// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/HoodSubsystem.java
//
// PURPOSE: Controls the variable hood angle via a linear servo.
//   The hood adjusts the shooter's launch angle to optimize trajectory
//   based on distance to target.
//
// HOW IT WORKS:
//   - A linear servo (PWM) moves the hood between min and max angles.
//   - Servo position 0.0 corresponds to MIN_ANGLE_DEG (flat/close).
//   - Servo position 1.0 corresponds to MAX_ANGLE_DEG (steep/far).
//   - calculateTargetAngle() delegates to ShotSolver which uses projectile
//     physics to compute a coupled (angle, speed) pair.
// ============================================================================
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.ShotSolver;

public class HoodSubsystem extends SubsystemBase {
    private final Servo hoodServo = new Servo(Constants.PWM.HOOD_SERVO);

    // The currently commanded angle (degrees). Used for telemetry and isAtAngle().
    private double commandedAngleDeg = Constants.Hood.DEFAULT_ANGLE_DEG;

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public HoodSubsystem() {
        // Set the servo to the default position on startup.
        setAngle(Constants.Hood.DEFAULT_ANGLE_DEG);
    }

    // --------------------------------------------------------------------------
    // periodic() — called every 20ms
    // --------------------------------------------------------------------------
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/CommandedAngleDeg", commandedAngleDeg);
        SmartDashboard.putNumber("Hood/ServoPosition", hoodServo.get());
    }

    // --------------------------------------------------------------------------
    // setAngle()
    //
    // Commands the hood to a specific angle (in degrees).
    // Clamps to the physical limits defined in Constants.Hood.
    // --------------------------------------------------------------------------
    public void setAngle(double angleDeg) {
        commandedAngleDeg = MathUtil.clamp(
                angleDeg,
                Constants.Hood.MIN_ANGLE_DEG,
                Constants.Hood.MAX_ANGLE_DEG);
        hoodServo.set(angleToServoPosition(commandedAngleDeg));
    }

    // --------------------------------------------------------------------------
    // setDefault()
    //
    // Moves the hood to the default/neutral position.
    // --------------------------------------------------------------------------
    public void setDefault() {
        setAngle(Constants.Hood.DEFAULT_ANGLE_DEG);
    }

    // --------------------------------------------------------------------------
    // getCommandedAngleDeg()
    // --------------------------------------------------------------------------
    public double getCommandedAngleDeg() {
        return commandedAngleDeg;
    }

    // --------------------------------------------------------------------------
    // calculateTargetAngle()
    //
    // Returns the optimal hood angle for the given distance, computed by
    // ShotSolver using projectile physics.  The angle and shooter RPS are
    // derived from the same trajectory equation, so they are always coupled.
    // --------------------------------------------------------------------------
    public static double calculateTargetAngle(double distanceM) {
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return Constants.Hood.DEFAULT_ANGLE_DEG;
        }
        ShotSolver.Solution solution = ShotSolver.solve(distanceM);
        return solution.angleDeg();
    }

    // --------------------------------------------------------------------------
    // Servo position conversion
    //
    // Maps angle (degrees) to servo range [0.0, 1.0].
    // 0.0 = MIN_ANGLE_DEG, 1.0 = MAX_ANGLE_DEG.
    // --------------------------------------------------------------------------
    private static double angleToServoPosition(double angleDeg) {
        double range = Constants.Hood.MAX_ANGLE_DEG - Constants.Hood.MIN_ANGLE_DEG;
        if (range <= 0.0) {
            return 0.5;
        }
        return MathUtil.clamp(
                (angleDeg - Constants.Hood.MIN_ANGLE_DEG) / range,
                0.0,
                1.0);
    }
}
