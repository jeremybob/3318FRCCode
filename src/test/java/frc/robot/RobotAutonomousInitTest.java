package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

class RobotAutonomousInitTest {

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @AfterEach
    void cleanScheduler() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Test
    void autonomousInitWrapsAndSchedulesSelectedAuto() {
        Command selectedAuto = new WaitCommand(30.0).withName("SelectedAuto");
        TestRobotContainer container = new TestRobotContainer(selectedAuto, Commands.none());

        try (Robot robot = new Robot()) {
            robot.setRobotContainerForTest(container);
            robot.autonomousInit();

            assertEquals(2, container.setCurrentAutoCommandCalls.size());
            assertSame(selectedAuto, container.setCurrentAutoCommandCalls.get(0));
            assertNotNull(container.setCurrentAutoCommandCalls.get(1));
            assertNotSame(selectedAuto, container.setCurrentAutoCommandCalls.get(1));
            assertTrue(CommandScheduler.getInstance().isScheduled(container.setCurrentAutoCommandCalls.get(1)));
        }
    }

    @Test
    void autonomousInitHandlesNoSelectedAuto() {
        TestRobotContainer container = new TestRobotContainer(null, Commands.none());

        try (Robot robot = new Robot()) {
            robot.setRobotContainerForTest(container);
            robot.autonomousInit();

            assertEquals(1, container.setCurrentAutoCommandCalls.size());
            assertNull(container.setCurrentAutoCommandCalls.get(0));
            assertFalse(container.periodicDashboardCalled);
        }
    }

    private static final class TestRobotContainer implements RobotRuntimeContainer {
        private final Command autonomousCommand;
        private final Command intakeHomeCommand;
        private final List<Command> setCurrentAutoCommandCalls = new ArrayList<>();
        private boolean periodicDashboardCalled = false;

        private TestRobotContainer(Command autonomousCommand, Command intakeHomeCommand) {
            this.autonomousCommand = autonomousCommand;
            this.intakeHomeCommand = intakeHomeCommand;
        }

        @Override
        public void periodicDashboard() {
            periodicDashboardCalled = true;
        }

        @Override
        public Command getAutonomousCommand() {
            return autonomousCommand;
        }

        @Override
        public void setCurrentAutoCommand(Command command) {
            setCurrentAutoCommandCalls.add(command);
        }

        @Override
        public Command getIntakeHomeCommand() {
            return intakeHomeCommand;
        }
    }
}
