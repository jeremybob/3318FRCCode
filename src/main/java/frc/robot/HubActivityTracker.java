// ============================================================================
// FILE: src/main/java/frc/robot/HubActivityTracker.java
//
// PURPOSE: Tracks whether your alliance's HUB is currently active in the
//   2026 REBUILT game. In REBUILT, HUBs alternate between active/inactive
//   during teleop "shifts" based on autonomous performance.
//
// GAME MECHANIC:
//   - Both HUBs are active during AUTO (20 sec) and END GAME (last 30 sec)
//   - The alliance that scores MORE in auto has their HUB INACTIVE first
//   - Teleop is divided into: Transition (10s), Shift1-4 (25s each)
//   - Game Data from FMS ('R' or 'B') tells which alliance goes inactive first
//   - If auto is tied, FMS randomly selects
//
// SHIFT TIMING (from match time remaining):
//   2:40 - 2:20  = Auto         → BOTH active
//   2:20 - 2:10  = Transition   → BOTH active
//   2:10 - 1:45  = Shift 1      → alternating
//   1:45 - 1:20  = Shift 2      → alternating (opposite)
//   1:20 - 0:55  = Shift 3      → alternating (same as Shift 1)
//   0:55 - 0:30  = Shift 4      → alternating (same as Shift 2)
//   0:30 - 0:00  = End Game     → BOTH active
//
// USAGE:
//   Call HubActivityTracker.isOurHubActive() in teleop to decide whether to
//   shoot or instead collect/stage FUEL for the next active window.
// ============================================================================
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class HubActivityTracker {

    private HubActivityTracker() {}

    /**
     * Returns true if your alliance's HUB is currently active (accepting FUEL for points).
     * During auto and end game, always returns true.
     * During teleop shifts, uses Game Data from FMS to determine activity.
     */
    public static boolean isOurHubActive() {
        // During autonomous, both HUBs are always active
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        // During disabled/test, assume active (doesn't matter for scoring)
        if (!DriverStation.isTeleopEnabled()) {
            return true;
        }

        double matchTime = DriverStation.getMatchTime();

        // End Game (last 30 seconds): both HUBs active
        if (matchTime <= 30.0) {
            return true;
        }

        // Transition shift (2:20 to 2:10 remaining = 130-140 remaining): both active
        if (matchTime >= 130.0) {
            return true;
        }

        // Determine shift number based on match time
        // Shift 1: 2:10 to 1:45 (130 to 105 remaining)
        // Shift 2: 1:45 to 1:20 (105 to 80 remaining)
        // Shift 3: 1:20 to 0:55 (80 to 55 remaining)
        // Shift 4: 0:55 to 0:30 (55 to 30 remaining)
        int shiftNumber;
        if (matchTime >= 105.0) {
            shiftNumber = 1;
        } else if (matchTime >= 80.0) {
            shiftNumber = 2;
        } else if (matchTime >= 55.0) {
            shiftNumber = 3;
        } else {
            shiftNumber = 4;
        }

        // Game Data tells us which alliance goes inactive first
        // 'R' = Red goes inactive in Shifts 1 & 3 (meaning Red won auto)
        // 'B' = Blue goes inactive in Shifts 1 & 3 (meaning Blue won auto)
        // If Game Data is empty, auto was tied — FMS randomly chose.
        String gameData = DriverStation.getGameSpecificMessage();
        boolean weGoInactiveFirst = false;

        if (gameData != null && !gameData.isEmpty()) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                char inactiveFirst = gameData.charAt(0);
                if (alliance.get() == DriverStation.Alliance.Red) {
                    weGoInactiveFirst = (inactiveFirst == 'R');
                } else {
                    weGoInactiveFirst = (inactiveFirst == 'B');
                }
            }
        }

        // Odd shifts (1, 3): the "inactive first" alliance is inactive
        // Even shifts (2, 4): the "inactive first" alliance is active
        boolean oddShift = (shiftNumber % 2 == 1);
        boolean active;
        if (weGoInactiveFirst) {
            active = !oddShift; // inactive in 1 & 3, active in 2 & 4
        } else {
            active = oddShift;  // active in 1 & 3, inactive in 2 & 4
        }

        // Publish to dashboard for operator awareness
        SmartDashboard.putBoolean("HUB/Active", active);
        SmartDashboard.putNumber("HUB/ShiftNumber", shiftNumber);
        SmartDashboard.putNumber("HUB/MatchTimeRemaining", matchTime);

        return active;
    }

    /**
     * Returns the number of seconds until the next shift change.
     * Useful for deciding whether to shoot now or wait.
     */
    public static double secondsUntilNextShiftChange() {
        double matchTime = DriverStation.getMatchTime();

        // Shift boundaries (match time remaining)
        double[] boundaries = {130.0, 105.0, 80.0, 55.0, 30.0};
        for (double boundary : boundaries) {
            if (matchTime > boundary) {
                return matchTime - boundary;
            }
        }
        return matchTime; // end game, just return remaining time
    }
}
