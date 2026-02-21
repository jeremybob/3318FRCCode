package frc.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DashboardMainArgsTest {

    @Test
    void parseArgsUsesDefaultsWhenNoOverridesProvided() {
        DashboardMain.ConnectionArgs args = DashboardMain.parseArgs(new String[] {});
        assertEquals(3318, args.teamNumber());
        assertEquals("10.33.18.2", args.hostOverride());
    }

    @Test
    void parseArgsUsesTeamWhenProvided() {
        DashboardMain.ConnectionArgs args = DashboardMain.parseArgs(new String[] {"--team", "2468"});
        assertEquals(2468, args.teamNumber());
        assertEquals("", args.hostOverride());
    }

    @Test
    void parseArgsUsesHostOverrideWhenProvided() {
        DashboardMain.ConnectionArgs args = DashboardMain.parseArgs(new String[] {"--host", "127.0.0.1"});
        assertEquals(0, args.teamNumber());
        assertEquals("127.0.0.1", args.hostOverride());
    }

    @Test
    void parseArgsFallsBackToDefaultsOnInvalidTeam() {
        DashboardMain.ConnectionArgs args = DashboardMain.parseArgs(new String[] {"--team", "abc"});
        assertEquals(3318, args.teamNumber());
        assertEquals("10.33.18.2", args.hostOverride());
    }
}
