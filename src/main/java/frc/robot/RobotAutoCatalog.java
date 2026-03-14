package frc.robot;

import java.util.List;

final class RobotAutoCatalog {
    static final String NAMED_HOME_INTAKE = "HomeIntake";
    static final String NAMED_INTAKE_FUEL = "IntakeFuel";
    static final String NAMED_INTAKE_DEPLOY_ONLY = "IntakeDeployOnly";
    static final String NAMED_INTAKE_BALLS = "IntakeBalls";
    static final String NAMED_AUTO_SHOOT = "AutoShoot";
    static final String NAMED_AUTO_MANUAL_DISTANCE_SHOOT = "AutoManualDistanceShoot";

    private RobotAutoCatalog() {}

    static List<PathPlannerAutoSpec> competitionPathPlannerAutos() {
        return List.of(
                new PathPlannerAutoSpec("Depot", "Depot"),
                new PathPlannerAutoSpec("Only Shoot", "OnlyShoot"),
                new PathPlannerAutoSpec("Outpost", "Outpost"));
    }

    record PathPlannerAutoSpec(String chooserName, String autoFileName) {}
}
