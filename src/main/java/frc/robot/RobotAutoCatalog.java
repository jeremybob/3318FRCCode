package frc.robot;

import java.util.List;

final class RobotAutoCatalog {
    static final String NAMED_HOME_INTAKE = "HomeIntake";
    static final String NAMED_INTAKE_FUEL = "IntakeFuel";
    static final String NAMED_INTAKE_GAME_PIECE = "IntakeGamePiece";
    static final String NAMED_AUTO_SHOOT = "AutoShoot";

    private RobotAutoCatalog() {}

    static List<PathPlannerAutoSpec> competitionPathPlannerAutos() {
        return List.of(
                new PathPlannerAutoSpec("Blue Depot", "BlueDepot"),
                new PathPlannerAutoSpec("Blue Outpost", "BlueOutpost"));
    }

    record PathPlannerAutoSpec(String chooserName, String autoFileName) {}
}
