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

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class CalibrateCANcodersCommand extends Command {

    public CalibrateCANcodersCommand() {
        // No subsystem requirements — just reads encoders
    }

    @Override
    public void initialize() {
        double flRaw;
        double frRaw;
        double blRaw;
        double brRaw;

        // Read all four CANcoders and close handles immediately after sampling.
        try (CANcoder flEncoder = new CANcoder(Constants.CAN.FRONT_LEFT_CANCODER, Constants.CAN.CTRE_CAN_BUS);
                CANcoder frEncoder = new CANcoder(Constants.CAN.FRONT_RIGHT_CANCODER, Constants.CAN.CTRE_CAN_BUS);
                CANcoder blEncoder = new CANcoder(Constants.CAN.BACK_LEFT_CANCODER, Constants.CAN.CTRE_CAN_BUS);
                CANcoder brEncoder = new CANcoder(Constants.CAN.BACK_RIGHT_CANCODER, Constants.CAN.CTRE_CAN_BUS)) {
            flRaw = flEncoder.getAbsolutePosition().getValueAsDouble();
            frRaw = frEncoder.getAbsolutePosition().getValueAsDouble();
            blRaw = blEncoder.getAbsolutePosition().getValueAsDouble();
            brRaw = brEncoder.getAbsolutePosition().getValueAsDouble();
        }

        // Print calibration values to console using plain ASCII for terminal compatibility.
        System.out.println("+------------------------------------------------------+");
        System.out.println("| CANCODER CALIBRATION - ALIGN WHEELS FIRST!          |");
        System.out.println("+------------------------------------------------------+");
        System.out.printf("| FL Raw: %+.4f -> Offset: %+.4f%n", flRaw, -flRaw);
        System.out.printf("| FR Raw: %+.4f -> Offset: %+.4f%n", frRaw, -frRaw);
        System.out.printf("| BL Raw: %+.4f -> Offset: %+.4f%n", blRaw, -blRaw);
        System.out.printf("| BR Raw: %+.4f -> Offset: %+.4f%n", brRaw, -brRaw);
        System.out.println("+------------------------------------------------------+");
        System.out.println("| Copy these values into Constants.Swerve:            |");
        System.out.printf("|   FL_CANCODER_OFFSET_ROT = %.4f;%n", -flRaw);
        System.out.printf("|   FR_CANCODER_OFFSET_ROT = %.4f;%n", -frRaw);
        System.out.printf("|   BL_CANCODER_OFFSET_ROT = %.4f;%n", -blRaw);
        System.out.printf("|   BR_CANCODER_OFFSET_ROT = %.4f;%n", -brRaw);
        System.out.println("+------------------------------------------------------+");

        // Also publish to SmartDashboard for easy access
        SmartDashboard.putNumber("CANcoder/FL_Raw", flRaw);
        SmartDashboard.putNumber("CANcoder/FR_Raw", frRaw);
        SmartDashboard.putNumber("CANcoder/BL_Raw", blRaw);
        SmartDashboard.putNumber("CANcoder/BR_Raw", brRaw);
        SmartDashboard.putNumber("CANcoder/FL_Offset", -flRaw);
        SmartDashboard.putNumber("CANcoder/FR_Offset", -frRaw);
        SmartDashboard.putNumber("CANcoder/BL_Offset", -blRaw);
        SmartDashboard.putNumber("CANcoder/BR_Offset", -brRaw);

        // Mirror to custom dashboard contract table so pit dashboard can read them too.
        NetworkTable dashboardTable = NetworkTableInstance.getDefault().getTable("Dashboard");
        dashboardTable.getEntry("cancoder/fl_raw_rot").setDouble(flRaw);
        dashboardTable.getEntry("cancoder/fr_raw_rot").setDouble(frRaw);
        dashboardTable.getEntry("cancoder/bl_raw_rot").setDouble(blRaw);
        dashboardTable.getEntry("cancoder/br_raw_rot").setDouble(brRaw);
        dashboardTable.getEntry("cancoder/fl_offset_rot").setDouble(-flRaw);
        dashboardTable.getEntry("cancoder/fr_offset_rot").setDouble(-frRaw);
        dashboardTable.getEntry("cancoder/bl_offset_rot").setDouble(-blRaw);
        dashboardTable.getEntry("cancoder/br_offset_rot").setDouble(-brRaw);
    }

    @Override
    public boolean isFinished() {
        return true; // runs once and finishes
    }
}
