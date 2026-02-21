// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/HopperSubsystem.java
//
// PURPOSE: Controls the hopper floor motor that pushes game pieces toward the
//   feeder and into the shooter. Open-loop power control by design.
//
// Hardware: REV SparkMax + NEO brushless motor
// NO burnFlash() — see IntakeSubsystem for the policy explanation.
// ============================================================================
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {

    private final SparkMax floorMotor =
            new SparkMax(Constants.CAN.HOPPER_FLOOR_NEO, MotorType.kBrushless);

    public HopperSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(Constants.NeoMotors.DEFAULT_CURRENT_LIMIT_A);
        config.idleMode(IdleMode.kBrake);
        floorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        // Publish current draw to dashboard — useful for detecting jams
        SmartDashboard.putNumber("Hopper/CurrentAmps",
                getCurrentAmps());
    }

    public double getCurrentAmps() {
        return floorMotor.getOutputCurrent();
    }

    // Sets motor power. Positive = toward feeder. Negative = reverse/clear.
    public void setPower(double power) {
        floorMotor.set(power);
    }

    public void stop() {
        floorMotor.stopMotor();
    }
}
