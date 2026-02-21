package frc.dashboard;

import javax.swing.SwingUtilities;

public final class DashboardMain {

    private DashboardMain() {}

    public static void main(String[] args) {
        ConnectionArgs connectionArgs = parseArgs(args);

        DashboardNtClient client = new DashboardNtClient(
                "3318Dashboard-" + System.currentTimeMillis(),
                connectionArgs.teamNumber(),
                connectionArgs.hostOverride());

        Runtime.getRuntime().addShutdownHook(new Thread(client::close));

        SwingUtilities.invokeLater(() -> {
            DashboardFrame frame = new DashboardFrame(client);
            frame.setVisible(true);
        });
    }

    static ConnectionArgs parseArgs(String[] args) {
        int teamNumber = 3318;
        String hostOverride = "10.33.18.2";

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

    record ConnectionArgs(int teamNumber, String hostOverride) {}
}
