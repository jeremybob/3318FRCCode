package frc.dashboard;

import javax.swing.SwingUtilities;

public final class DashboardMain {

    private DashboardMain() {}

    public static void main(String[] args) {
        int teamNumber = 3318;
        String hostOverride = "10.33.18.2";

        for (int i = 0; i < args.length; i++) {
            if ("--team".equals(args[i]) && i + 1 < args.length) {
                teamNumber = Integer.parseInt(args[++i]);
                hostOverride = "";
            } else if ("--host".equals(args[i]) && i + 1 < args.length) {
                hostOverride = args[++i];
                teamNumber = 0;
            }
        }

        DashboardNtClient client = new DashboardNtClient(
                "3318Dashboard-" + System.currentTimeMillis(),
                teamNumber,
                hostOverride);

        Runtime.getRuntime().addShutdownHook(new Thread(client::close));

        SwingUtilities.invokeLater(() -> {
            DashboardFrame frame = new DashboardFrame(client);
            frame.setVisible(true);
        });
    }
}
