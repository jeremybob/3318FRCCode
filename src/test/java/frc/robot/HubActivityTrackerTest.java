package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation;

class HubActivityTrackerTest {

    @Test
    void autonomousAndTransitionWindowsAlwaysReportActive() {
        assertTrue(HubActivityTracker.isOurHubActive(
                true,
                false,
                18.0,
                DriverStation.Alliance.Red,
                "R"));
        assertTrue(HubActivityTracker.isOurHubActive(
                false,
                true,
                135.0,
                DriverStation.Alliance.Blue,
                "B"));
        assertTrue(HubActivityTracker.isOurHubActive(
                false,
                true,
                25.0,
                DriverStation.Alliance.Blue,
                "R"));
    }

    @Test
    void inactiveFirstAllianceAlternatesByShift() {
        assertFalse(HubActivityTracker.isOurHubActive(
                false,
                true,
                120.0,
                DriverStation.Alliance.Red,
                "R"));
        assertTrue(HubActivityTracker.isOurHubActive(
                false,
                true,
                120.0,
                DriverStation.Alliance.Blue,
                "R"));

        assertTrue(HubActivityTracker.isOurHubActive(
                false,
                true,
                95.0,
                DriverStation.Alliance.Red,
                "R"));
        assertFalse(HubActivityTracker.isOurHubActive(
                false,
                true,
                95.0,
                DriverStation.Alliance.Blue,
                "R"));
    }

    @Test
    void missingGameDataFailsOpenToActive() {
        assertTrue(HubActivityTracker.isOurHubActive(
                false,
                true,
                120.0,
                DriverStation.Alliance.Red,
                ""));
        assertTrue(HubActivityTracker.isOurHubActive(
                false,
                true,
                120.0,
                null,
                "B"));
    }

    @Test
    void reportsTimeUntilNextShiftBoundary() {
        assertEquals(5.0, HubActivityTracker.secondsUntilNextShiftChange(135.0), 1e-9);
        assertEquals(15.0, HubActivityTracker.secondsUntilNextShiftChange(120.0), 1e-9);
        assertEquals(25.0, HubActivityTracker.secondsUntilNextShiftChange(80.0), 1e-9);
        assertEquals(20.0, HubActivityTracker.secondsUntilNextShiftChange(20.0), 1e-9);
    }
}
