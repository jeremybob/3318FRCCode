// ============================================================================
// FILE: src/main/java/frc/robot/RobotAutoCatalog.java
//
// PURPOSE: Central registry of autonomous routines and PathPlanner named
//   commands. All auto-related constants live here so students can see
//   every available auto option in one place.
//
//   - Named command strings (e.g., "IntakeFuel") must match what is used
//     in PathPlanner path event markers.
//   - competitionPathPlannerAutos() returns the list of autos shown in the
//     SmartDashboard chooser dropdown.
// ============================================================================
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
                new PathPlannerAutoSpec("Only Shoot Left", "OnlyShootLeft"),
                new PathPlannerAutoSpec("Only Shoot Middle", "OnlyShootMiddle"),
                new PathPlannerAutoSpec("Only Shoot Right", "OnlyShootRight"),
                new PathPlannerAutoSpec("Outpost", "Outpost"));
    }

    /** chooserName = human-readable label in SmartDashboard; autoFileName = .auto file in deploy/pathplanner/autos/ */
    record PathPlannerAutoSpec(String chooserName, String autoFileName) {}
}
