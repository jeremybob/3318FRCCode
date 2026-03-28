// ============================================================================
// FILE: src/main/java/frc/robot/RobotRuntimeContainer.java
//
// PURPOSE: Interface that decouples Robot.java from RobotContainer.java.
//   Robot.java only knows about this interface, not the concrete class.
//   This lets unit tests substitute a fake/mock container without needing
//   real hardware or subsystems.
// ============================================================================
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

interface RobotRuntimeContainer {
    void periodicDashboard();

    Command getAutonomousCommand();

    void setCurrentAutoCommand(Command command);

    Command getIntakeHomeCommand();
}
