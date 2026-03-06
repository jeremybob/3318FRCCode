// ============================================================================
// FILE: src/main/java/frc/robot/commands/CalibrateCANcodersCommand.java
//
// PURPOSE: Reads raw CANcoder positions and prints the values needed for
//   Constants.java CANcoder offset calibration.
//
// HOW TO USE:
//   1. Physically align all 4 swerve wheels pointing STRAIGHT FORWARD
//      (use a straightedge across the robot frame to ensure precision)
//   2. Add this auto option to the SmartDashboard auto chooser (already done)
//   3. Enable the robot briefly in Test mode or run this auto
//   4. Check the console output or SmartDashboard for the offset values
//   5. Copy the NEGATED values into Constants.Swerve.*_CANCODER_OFFSET_ROT
//   6. Re-deploy and verify all wheels face forward at startup
//
// IMPORTANT: This command runs once and finishes immediately.
//   It does NOT move any motors — safe to run at any time.
// ============================================================================
package frc.robot.commands;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveCalibrationUtil;

public class CalibrateCANcodersCommand extends Command {

    public CalibrateCANcodersCommand() {
        // No subsystem requirements — just reads encoders
    }

    @Override
    public void initialize() {
        SwerveCalibrationUtil.CalibrationSample flSample;
        SwerveCalibrationUtil.CalibrationSample frSample;
        SwerveCalibrationUtil.CalibrationSample blSample;
        SwerveCalibrationUtil.CalibrationSample brSample;

        // Read all four CANcoders and close handles immediately after sampling.
        try (CANcoder flEncoder = new CANcoder(Constants.CAN.FRONT_LEFT_CANCODER,
                new CANBus(Constants.CAN.CTRE_CAN_BUS));
                CANcoder frEncoder = new CANcoder(Constants.CAN.FRONT_RIGHT_CANCODER,
                        new CANBus(Constants.CAN.CTRE_CAN_BUS));
                CANcoder blEncoder = new CANcoder(Constants.CAN.BACK_LEFT_CANCODER,
                        new CANBus(Constants.CAN.CTRE_CAN_BUS));
                CANcoder brEncoder = new CANcoder(Constants.CAN.BACK_RIGHT_CANCODER,
                        new CANBus(Constants.CAN.CTRE_CAN_BUS))) {
            flSample = sampleCalibration(flEncoder);
            frSample = sampleCalibration(frEncoder);
            blSample = sampleCalibration(blEncoder);
            brSample = sampleCalibration(brEncoder);
        }

        // Print calibration values to console using plain ASCII for terminal compatibility.
        System.out.println("+------------------------------------------------------+");
        System.out.println("| CANCODER CALIBRATION - ALIGN WHEELS FIRST!          |");
        System.out.println("+------------------------------------------------------+");
        System.out.printf("| FL NoOffset: %+.6f -> Offset: %+.6f%n",
                flSample.noOffsetRot(), flSample.recommendedOffsetRot());
        System.out.printf("| FR NoOffset: %+.6f -> Offset: %+.6f%n",
                frSample.noOffsetRot(), frSample.recommendedOffsetRot());
        System.out.printf("| BL NoOffset: %+.6f -> Offset: %+.6f%n",
                blSample.noOffsetRot(), blSample.recommendedOffsetRot());
        System.out.printf("| BR NoOffset: %+.6f -> Offset: %+.6f%n",
                brSample.noOffsetRot(), brSample.recommendedOffsetRot());
        System.out.println("+------------------------------------------------------+");
        System.out.println("| Copy these values into Constants.Swerve:            |");
        System.out.printf("|   FL_CANCODER_OFFSET_ROT = %.6f;%n", flSample.recommendedOffsetRot());
        System.out.printf("|   FR_CANCODER_OFFSET_ROT = %.6f;%n", frSample.recommendedOffsetRot());
        System.out.printf("|   BL_CANCODER_OFFSET_ROT = %.6f;%n", blSample.recommendedOffsetRot());
        System.out.printf("|   BR_CANCODER_OFFSET_ROT = %.6f;%n", brSample.recommendedOffsetRot());
        System.out.println("+------------------------------------------------------+");

        // Also publish to SmartDashboard for easy access
        SmartDashboard.putNumber("CANcoder/FL_Raw", flSample.noOffsetRot());
        SmartDashboard.putNumber("CANcoder/FR_Raw", frSample.noOffsetRot());
        SmartDashboard.putNumber("CANcoder/BL_Raw", blSample.noOffsetRot());
        SmartDashboard.putNumber("CANcoder/BR_Raw", brSample.noOffsetRot());
        SmartDashboard.putNumber("CANcoder/FL_Offset", flSample.recommendedOffsetRot());
        SmartDashboard.putNumber("CANcoder/FR_Offset", frSample.recommendedOffsetRot());
        SmartDashboard.putNumber("CANcoder/BL_Offset", blSample.recommendedOffsetRot());
        SmartDashboard.putNumber("CANcoder/BR_Offset", brSample.recommendedOffsetRot());

        // Mirror to custom dashboard contract table so pit dashboard can read them too.
        NetworkTable dashboardTable = NetworkTableInstance.getDefault().getTable("Dashboard");
        dashboardTable.getEntry("cancoder/fl_raw_rot").setDouble(flSample.noOffsetRot());
        dashboardTable.getEntry("cancoder/fr_raw_rot").setDouble(frSample.noOffsetRot());
        dashboardTable.getEntry("cancoder/bl_raw_rot").setDouble(blSample.noOffsetRot());
        dashboardTable.getEntry("cancoder/br_raw_rot").setDouble(brSample.noOffsetRot());
        dashboardTable.getEntry("cancoder/fl_offset_rot").setDouble(flSample.recommendedOffsetRot());
        dashboardTable.getEntry("cancoder/fr_offset_rot").setDouble(frSample.recommendedOffsetRot());
        dashboardTable.getEntry("cancoder/bl_offset_rot").setDouble(blSample.recommendedOffsetRot());
        dashboardTable.getEntry("cancoder/br_offset_rot").setDouble(brSample.recommendedOffsetRot());
    }

    @Override
    public boolean isFinished() {
        return true; // runs once and finishes
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private static SwerveCalibrationUtil.CalibrationSample sampleCalibration(CANcoder encoder) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        encoder.getConfigurator().refresh(config);
        double configuredAbsoluteRot = encoder.getAbsolutePosition().getValueAsDouble();
        return SwerveCalibrationUtil.sample(
                configuredAbsoluteRot,
                config.MagnetSensor.MagnetOffset);
    }
}
