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

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveCalibrationUtil;

public class CalibrateCANcodersCommand extends Command {

    private final SwerveSubsystem swerve;

    public CalibrateCANcodersCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        // No subsystem requirements — only reads existing encoder data, does not drive
    }

    @Override
    public void initialize() {
        // Read calibration samples from the existing SwerveModule CANcoder handles
        // (no duplicate device objects created on the CAN bus).
        SwerveCalibrationUtil.CalibrationSample[] samples = swerve.getCalibrationSamples();
        SwerveCalibrationUtil.CalibrationSample flSample = samples[0];
        SwerveCalibrationUtil.CalibrationSample frSample = samples[1];
        SwerveCalibrationUtil.CalibrationSample blSample = samples[2];
        SwerveCalibrationUtil.CalibrationSample brSample = samples[3];

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

        // Mirror to custom dashboard contract table using typed publishers.
        NetworkTable dashboardTable = NetworkTableInstance.getDefault().getTable("Dashboard");
        publishCalibrationValue(dashboardTable, "cancoder/fl_raw_rot", flSample.noOffsetRot());
        publishCalibrationValue(dashboardTable, "cancoder/fr_raw_rot", frSample.noOffsetRot());
        publishCalibrationValue(dashboardTable, "cancoder/bl_raw_rot", blSample.noOffsetRot());
        publishCalibrationValue(dashboardTable, "cancoder/br_raw_rot", brSample.noOffsetRot());
        publishCalibrationValue(dashboardTable, "cancoder/fl_offset_rot", flSample.recommendedOffsetRot());
        publishCalibrationValue(dashboardTable, "cancoder/fr_offset_rot", frSample.recommendedOffsetRot());
        publishCalibrationValue(dashboardTable, "cancoder/bl_offset_rot", blSample.recommendedOffsetRot());
        publishCalibrationValue(dashboardTable, "cancoder/br_offset_rot", brSample.recommendedOffsetRot());
    }

    @Override
    public boolean isFinished() {
        return true; // runs once and finishes
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private static void publishCalibrationValue(NetworkTable table, String key, double value) {
        try (DoublePublisher pub = table.getDoubleTopic(key).publish()) {
            pub.set(value);
        }
    }
}
