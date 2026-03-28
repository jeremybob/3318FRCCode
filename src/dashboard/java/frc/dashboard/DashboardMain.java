// ============================================================================
// FILE: src/dashboard/java/frc/dashboard/DashboardMain.java
//
// PURPOSE: Entry point for the custom dashboard application.
//   This is a standalone Java Swing app (not deployed to the robot) that
//   connects to the robot via NetworkTables to display real-time telemetry.
//
// USAGE:
//   ./run-dashboard.sh              (default: connect to team 3318)
//   ./run-dashboard.sh --host IP    (connect to a specific robot IP)
//   ./run-dashboard.sh --team 3318  (connect by team number)
//
// The dashboard shows shooter speed, intake status, alignment state,
// vision info, and provides buttons for operator commands.
// ============================================================================
package frc.dashboard;

import javax.swing.SwingUtilities;

public final class DashboardMain {

    private DashboardMain() {}

    public static void main(String[] args) {
        ConnectionArgs connectionArgs = parseArgs(args);

        // Append a timestamp to make the client name unique if multiple dashboards connect
        DashboardNtClient client = new DashboardNtClient(
                "3318Dashboard-" + System.currentTimeMillis(),
                connectionArgs.teamNumber(),
                connectionArgs.hostOverride());

        // Clean up the NetworkTables connection when the program closes
        Runtime.getRuntime().addShutdownHook(new Thread(client::close));

        // Swing UI must be created on the Event Dispatch Thread (required by Java's UI system)
        SwingUtilities.invokeLater(() -> {
            DashboardFrame frame = new DashboardFrame(client);
            frame.setVisible(true);
        });
    }

    /** Parses command-line arguments for robot connection settings. */
    static ConnectionArgs parseArgs(String[] args) {
        int teamNumber = 3318;                  // Default team number
        String hostOverride = "10.33.18.2";     // Default roboRIO IP (10.TE.AM.2 for team 3318)

        for (int i = 0; i < args.length; i++) {
            if ("--team".equals(args[i])) {
                if (i + 1 >= args.length) {
                    System.err.println("Missing value for --team; using default team 3318.");
                    continue;
                }
                String value = args[++i];
                try {
                    teamNumber = Integer.parseInt(value);
                    hostOverride = "";
                } catch (NumberFormatException ex) {
                    System.err.println("Invalid team number '" + value + "'; using default team 3318.");
                    teamNumber = 3318;
                    hostOverride = "10.33.18.2";
                }
            } else if ("--host".equals(args[i])) {
                if (i + 1 >= args.length) {
                    System.err.println("Missing value for --host; using default team 3318.");
                    continue;
                }
                hostOverride = args[++i];
                teamNumber = 0;
            }
        }

        return new ConnectionArgs(teamNumber, hostOverride);
    }

    /** Parsed command-line arguments for robot connection. */
    record ConnectionArgs(int teamNumber, String hostOverride) {}
}
