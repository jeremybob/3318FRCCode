// ============================================================================
// FILE: src/main/java/frc/robot/HubActivityTracker.java
//
// PURPOSE: Tracks whether your alliance's HUB is currently active in the
//   2026 REBUILT game. In REBUILT, HUBs alternate between active/inactive
//   during teleop "shifts" based on autonomous performance.
//
// GAME MECHANIC:
//   - Both HUBs are active during AUTO (20 sec) and END GAME (last 30 sec)
//   - The alliance that scores MORE FUEL in AUTO has their HUB go INACTIVE first
//     in SHIFT 1 (ties are broken by FMS)
//   - Teleop is divided into: Transition (10s), Shift1-4 (25s each)
//   - Game Data from FMS: first char tells which alliance's HUB goes inactive first
//   - If AUTO is tied, FMS randomly selects which HUB goes inactive first
//
// SHIFT TIMING (DriverStation.getMatchTime() counts DOWN from 140 in teleop):
//   140 - 130  = Transition   → BOTH active
//   130 - 105  = Shift 1      → alternating
//   105 -  80  = Shift 2      → alternating (opposite)
//    80 -  55  = Shift 3      → alternating (same as Shift 1)
//    55 -  30  = Shift 4      → alternating (same as Shift 2)
//    30 -   0  = End Game     → BOTH active
//
// IMPORTANT: Verify these boundaries and Game Data interpretation against the
// official 2026 FMS Game Data protocol at your first event. Practice mode
// may not provide Game Data or accurate match time.
//
// USAGE:
//   Call HubActivityTracker.isOurHubActive() in teleop to decide whether to
//   shoot or instead collect/stage FUEL for the next active window.
// ============================================================================
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public final class HubActivityTracker {

    private HubActivityTracker() {}

    /**
     * Returns true if your alliance's HUB is currently active (accepting FUEL for points).
     * During auto and end game, always returns true.
     * During teleop shifts, uses Game Data from FMS to determine activity.
     *
     * This method is a pure function with no side effects — SmartDashboard
     * publishing is handled by the caller (RobotDashboardService).
     */
    public static boolean isOurHubActive() {
        var alliance = DriverStation.getAlliance();
        return isOurHubActive(
                DriverStation.isAutonomousEnabled(),
                DriverStation.isTeleopEnabled(),
                DriverStation.getMatchTime(),
                alliance.orElse(null),
                DriverStation.getGameSpecificMessage());
    }

    static boolean isOurHubActive(
            boolean autonomousEnabled,
            boolean teleopEnabled,
            double matchTime,
            DriverStation.Alliance alliance,
            String gameData) {
        // During autonomous, both HUBs are always active
        if (autonomousEnabled) {
            return true;
        }

        // During disabled/test, assume active (doesn't matter for scoring)
        if (!teleopEnabled) {
            return true;
        }

        // End Game (last 30 seconds): both HUBs active
        if (matchTime <= 30.0) {
            return true;
        }

        // Transition shift (2:20 to 2:10 remaining = 130-140 remaining): both active
        if (matchTime >= 130.0) {
            return true;
        }

        // Determine shift number based on match time
        int shiftNumber = getShiftNumber(matchTime);

        // Game Data tells us which alliance's HUB goes inactive first in SHIFT 1.
        boolean weGoInactiveFirst = false;
        boolean gameDataAvailable = false;

        if (gameData != null && !gameData.isEmpty()) {
            if (alliance != null) {
                gameDataAvailable = true;
                char inactiveFirst = Character.toUpperCase(gameData.charAt(0));
                if (alliance == DriverStation.Alliance.Red) {
                    weGoInactiveFirst = inactiveFirst == 'R';
                } else {
                    weGoInactiveFirst = inactiveFirst == 'B';
                }
            }
        }

        // If Game Data is unavailable (practice mode), assume our HUB is active
        // to avoid suppressing shots. Better to shoot into an inactive HUB (0 pts)
        // than to hold fire when the HUB is actually active.
        if (!gameDataAvailable) {
            return true;
        }

        // Odd shifts (1, 3): the "inactive first" alliance is inactive
        // Even shifts (2, 4): the "inactive first" alliance is active
        boolean oddShift = (shiftNumber % 2 == 1);
        if (weGoInactiveFirst) {
            return !oddShift; // inactive in 1 & 3, active in 2 & 4
        } else {
            return oddShift;  // active in 1 & 3, inactive in 2 & 4
        }
    }

    /**
     * Returns the number of seconds until the next shift change.
     * Useful for deciding whether to shoot now or wait.
     */
    public static double secondsUntilNextShiftChange() {
        return secondsUntilNextShiftChange(DriverStation.getMatchTime());
    }

    static double secondsUntilNextShiftChange(double matchTime) {
        // Shift boundaries (match time remaining)
        double[] boundaries = {130.0, 105.0, 80.0, 55.0, 30.0};
        for (double boundary : boundaries) {
            if (matchTime > boundary) {
                return matchTime - boundary;
            }
        }
        return matchTime; // end game, just return remaining time
    }

    private static int getShiftNumber(double matchTime) {
        if (matchTime >= 105.0) return 1;
        if (matchTime >= 80.0) return 2;
        if (matchTime >= 55.0) return 3;
        return 4;
    }
}
