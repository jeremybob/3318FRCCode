package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

class RobotContainerAutoSelectionStateTest {

    @Test
    void defaultSelectionStartsWithDoNothingAndNoPose() {
        RobotContainer.AutoSelectionState state = new RobotContainer.AutoSelectionState();
        Command doNothing = Commands.none();

        state.registerOption("Do Nothing", doNothing, true, null);

        assertSame(doNothing, state.getSelectedAutoCommand());
        assertEquals("Do Nothing", state.getSelectedAutoName());
        assertEquals("DEFAULT", state.getSelectedAutoSource());
        assertTrue(Double.isNaN(state.getExpectedAutoStartX()));
        assertTrue(Double.isNaN(state.getExpectedAutoStartY()));
        assertTrue(Double.isNaN(state.getExpectedAutoStartHeadingDeg()));
    }

    @Test
    void selectingPathPlannerAutoUpdatesExpectedPoseMetadata() {
        RobotContainer.AutoSelectionState state = new RobotContainer.AutoSelectionState();
        Command doNothing = Commands.none();
        Command blueDepot = Commands.runOnce(() -> { });
        Pose2d startPose = new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(180.0));

        state.registerOption("Do Nothing", doNothing, true, null);
        state.registerOption("Blue Depot", blueDepot, false, startPose);
        state.selectCommand(blueDepot, "SMARTDASHBOARD");

        assertSame(blueDepot, state.getSelectedAutoCommand());
        assertEquals("Blue Depot", state.getSelectedAutoName());
        assertEquals("SMARTDASHBOARD", state.getSelectedAutoSource());
        assertEquals(2.5, state.getExpectedAutoStartX(), 1e-9);
        assertEquals(4.0, state.getExpectedAutoStartY(), 1e-9);
        assertEquals(180.0, state.getExpectedAutoStartHeadingDeg(), 1e-9);
    }

    @Test
    void competitionAutoCatalogMatchesConfiguredBlueAutos() {
        var autos = RobotAutoCatalog.competitionPathPlannerAutos();

        assertEquals(2, autos.size());
        assertEquals("Blue Depot", autos.get(0).chooserName());
        assertEquals("BlueDepot", autos.get(0).autoFileName());
        assertEquals("Blue Outpost", autos.get(1).chooserName());
        assertEquals("BlueOutpost", autos.get(1).autoFileName());
        assertArrayEquals(
                new String[] {
                    RobotAutoCatalog.NAMED_HOME_INTAKE,
                    RobotAutoCatalog.NAMED_INTAKE_FUEL,
                    RobotAutoCatalog.NAMED_INTAKE_GAME_PIECE,
                    RobotAutoCatalog.NAMED_INTAKE_DEPLOY_ONLY,
                    RobotAutoCatalog.NAMED_INTAKE_BALLS,
                    RobotAutoCatalog.NAMED_AUTO_SHOOT
                },
                new String[] {
                    "HomeIntake",
                    "IntakeFuel",
                    "IntakeGamePiece",
                    "IntakeDeployOnly",
                    "IntakeBalls",
                    "AutoShoot"
                });
    }
}
