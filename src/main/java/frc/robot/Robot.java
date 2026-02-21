// ============================================================================
// FILE: src/main/java/frc/robot/Robot.java
//
// PURPOSE: The main robot class. WPILib calls specific methods at specific times:
//   robotInit()         — once when the robot powers on
//   robotPeriodic()     — every 20ms, always
//   autonomousInit()    — once when autonomous starts
//   autonomousPeriodic()— every 20ms during autonomous
//   teleopInit()        — once when teleop starts
//   teleopPeriodic()    — every 20ms during teleop
//   disabledInit()      — once when robot is disabled
//
// In a command-based robot, most logic lives in Subsystems and Commands,
// not in Robot.java. This file mainly starts and stops the command scheduler.
// ============================================================================
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    // The RobotContainer holds all subsystems and command bindings.
    private RobotContainer robotContainer;

    // The autonomous command to run (selected from SmartDashboard)
    private Command autonomousCommand;

    // --------------------------------------------------------------------------
    // robotInit()
    //
    // Called ONCE when the robot first powers on.
    // We create the RobotContainer and trigger the intake homing sequence.
    // --------------------------------------------------------------------------
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        // ---- Auto-home the intake at startup ----
        // Because the tilt uses a relative encoder that resets on power-off,
        // we need to find the home position before we can use position control.
        // This runs immediately on power-on so it's done before the match starts.
        CommandScheduler.getInstance().schedule(robotContainer.getIntakeHomeCommand());
    }

    // --------------------------------------------------------------------------
    // robotPeriodic()
    //
    // Called every 20ms in ALL modes (disabled, auto, teleop, test).
    // CommandScheduler.run() updates all active commands and subsystem periodic()
    // methods — this is the heartbeat of the entire command-based framework.
    // --------------------------------------------------------------------------
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // --------------------------------------------------------------------------
    // disabledInit() — called when robot is disabled
    // --------------------------------------------------------------------------
    @Override
    public void disabledInit() {
        // Nothing needed here — CommandScheduler stops commanding motors when
        // the robot is disabled by the Driver Station.
    }

    // --------------------------------------------------------------------------
    // autonomousInit() — called once when auto starts
    // --------------------------------------------------------------------------
    @Override
    public void autonomousInit() {
        // Get the auto selected by the drive team on SmartDashboard
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        } else {
            System.out.println("[Robot] No autonomous command selected.");
        }
    }

    // --------------------------------------------------------------------------
    // autonomousPeriodic() — runs every 20ms during auto
    // CommandScheduler.run() in robotPeriodic() handles the actual command logic.
    // --------------------------------------------------------------------------
    @Override
    public void autonomousPeriodic() {}

    // --------------------------------------------------------------------------
    // teleopInit() — called once when teleop starts
    // --------------------------------------------------------------------------
    @Override
    public void teleopInit() {
        // Stop any autonomous command when teleop begins.
        // Without this, the auto path would keep running into teleop!
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    // --------------------------------------------------------------------------
    // teleopPeriodic() — runs every 20ms during teleop
    // CommandScheduler.run() in robotPeriodic() handles all command logic.
    // --------------------------------------------------------------------------
    @Override
    public void teleopPeriodic() {}

    // --------------------------------------------------------------------------
    // testInit() / testPeriodic() — for running test routines
    // --------------------------------------------------------------------------
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}
