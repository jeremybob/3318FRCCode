// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/HopperSubsystem.java
//
// PURPOSE: Controls the hopper floor motor that pushes game pieces toward the
//   feeder and into the shooter. Simple on/off/power control.
//
// Hardware: REV SparkMax + NEO brushless motor
// NO burnFlash() — see IntakeSubsystem for the policy explanation.
// ============================================================================
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {

    private final CANSparkMax floorMotor =
            new CANSparkMax(Constants.CAN.HOPPER_FLOOR_NEO, MotorType.kBrushless);

    public HopperSubsystem() {
        floorMotor.setSmartCurrentLimit(Constants.NeoMotors.DEFAULT_CURRENT_LIMIT_A);
        floorMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // Publish current draw to dashboard — useful for detecting jams
        SmartDashboard.putNumber("Hopper/CurrentAmps",
                floorMotor.getOutputCurrent());
    }

    // Sets motor power. Positive = toward feeder. Negative = reverse/clear.
    public void setPower(double power) {
        floorMotor.set(power);
    }

    public void stop() {
        floorMotor.stopMotor();
    }
}
