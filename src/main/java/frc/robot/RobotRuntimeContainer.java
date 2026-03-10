package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

interface RobotRuntimeContainer {
    void periodicDashboard();

    Command getAutonomousCommand();

    void setCurrentAutoCommand(Command command);

    Command getIntakeHomeCommand();
}
