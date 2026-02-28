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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class CalibrateCANcodersCommand extends Command {

    public CalibrateCANcodersCommand() {
        // No subsystem requirements — just reads encoders
    }

    @Override
    public void initialize() {
        // Read all four CANcoders
        CANcoder flEncoder = new CANcoder(Constants.CAN.FRONT_LEFT_CANCODER);
        CANcoder frEncoder = new CANcoder(Constants.CAN.FRONT_RIGHT_CANCODER);
        CANcoder blEncoder = new CANcoder(Constants.CAN.BACK_LEFT_CANCODER);
        CANcoder brEncoder = new CANcoder(Constants.CAN.BACK_RIGHT_CANCODER);

        double flRaw = flEncoder.getAbsolutePosition().getValueAsDouble();
        double frRaw = frEncoder.getAbsolutePosition().getValueAsDouble();
        double blRaw = blEncoder.getAbsolutePosition().getValueAsDouble();
        double brRaw = brEncoder.getAbsolutePosition().getValueAsDouble();

        // Print calibration values to console
        System.out.println("╔══════════════════════════════════════════════════════╗");
        System.out.println("║        CANCODER CALIBRATION — ALIGN WHEELS FIRST!   ║");
        System.out.println("╠══════════════════════════════════════════════════════╣");
        System.out.printf("║  FL Raw: %+.4f  → Offset: %+.4f%n", flRaw, -flRaw);
        System.out.printf("║  FR Raw: %+.4f  → Offset: %+.4f%n", frRaw, -frRaw);
        System.out.printf("║  BL Raw: %+.4f  → Offset: %+.4f%n", blRaw, -blRaw);
        System.out.printf("║  BR Raw: %+.4f  → Offset: %+.4f%n", brRaw, -brRaw);
        System.out.println("╠══════════════════════════════════════════════════════╣");
        System.out.println("║  Copy these values into Constants.Swerve:           ║");
        System.out.printf("║    FL_CANCODER_OFFSET_ROT = %.4f;%n", -flRaw);
        System.out.printf("║    FR_CANCODER_OFFSET_ROT = %.4f;%n", -frRaw);
        System.out.printf("║    BL_CANCODER_OFFSET_ROT = %.4f;%n", -blRaw);
        System.out.printf("║    BR_CANCODER_OFFSET_ROT = %.4f;%n", -brRaw);
        System.out.println("╚══════════════════════════════════════════════════════╝");

        // Also publish to SmartDashboard for easy access
        SmartDashboard.putNumber("CANcoder/FL_Raw", flRaw);
        SmartDashboard.putNumber("CANcoder/FR_Raw", frRaw);
        SmartDashboard.putNumber("CANcoder/BL_Raw", blRaw);
        SmartDashboard.putNumber("CANcoder/BR_Raw", brRaw);
        SmartDashboard.putNumber("CANcoder/FL_Offset", -flRaw);
        SmartDashboard.putNumber("CANcoder/FR_Offset", -frRaw);
        SmartDashboard.putNumber("CANcoder/BL_Offset", -blRaw);
        SmartDashboard.putNumber("CANcoder/BR_Offset", -brRaw);
    }

    @Override
    public boolean isFinished() {
        return true; // runs once and finishes
    }
}
