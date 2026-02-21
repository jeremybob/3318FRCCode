// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/FeederSubsystem.java
//
// PURPOSE: Controls the feeder motor that sits between the hopper and the
//   shooter wheels. It is the last mechanism the game piece passes through
//   before being launched.
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

public class FeederSubsystem extends SubsystemBase {

    private final CANSparkMax feederMotor =
            new CANSparkMax(Constants.CAN.FEEDER_NEO, MotorType.kBrushless);

    public FeederSubsystem() {
        feederMotor.setSmartCurrentLimit(Constants.NeoMotors.DEFAULT_CURRENT_LIMIT_A);
        feederMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/CurrentAmps",
                feederMotor.getOutputCurrent());
    }

    // Sets motor power. Positive = toward shooter. Negative = clear/reverse.
    public void setPower(double power) {
        feederMotor.set(power);
    }

    public void stop() {
        feederMotor.stopMotor();
    }
}
