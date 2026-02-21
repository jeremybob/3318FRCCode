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

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final SparkMax feederMotor =
            new SparkMax(Constants.CAN.FEEDER_NEO, MotorType.kBrushless);

    public FeederSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(Constants.NeoMotors.DEFAULT_CURRENT_LIMIT_A);
        config.idleMode(IdleMode.kBrake);
        feederMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/CurrentAmps", getCurrentAmps());
    }

    public double getCurrentAmps() {
        return feederMotor.getOutputCurrent();
    }

    // Sets motor power. Positive = toward shooter. Negative = clear/reverse.
    public void setPower(double power) {
        feederMotor.set(power);
    }

    public void stop() {
        feederMotor.stopMotor();
    }
}
