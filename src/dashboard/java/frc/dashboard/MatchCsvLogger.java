package frc.dashboard;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;

/**
 * Records {@link DashboardData} snapshots to CSV during a match and loads
 * them back for the Event Replay feature.  The CSV uses a fixed header that
 * matches the subset of DashboardData fields most useful for post-match
 * analysis.
 *
 * <p>Design goals:
 * <ul>
 *   <li>Zero external dependencies (Java standard library only).</li>
 *   <li>Writes are append-only and flushed on every snapshot so data
 *       survives a dashboard crash.</li>
 *   <li>The CSV is human-readable and can be opened in Excel.</li>
 * </ul>
 */
public final class MatchCsvLogger implements AutoCloseable {

    static final String[] HEADER = {
            "timestamp_sec",
            "battery_voltage",
            "mode",
            "enabled",
            "match_time_sec",
            "shooter_left_rps",
            "shooter_right_rps",
            "shooter_at_speed",
            "intake_homed",
            "intake_tilt_deg",
            "ready_to_score",
            "ready_reason",
            "align_state",
            "align_yaw_deg",
            "align_aim_error_deg",
            "align_lead_yaw_deg",
            "align_pitch_deg",
            "align_target_rps",
            "align_feed_gate_ready",
            "heading_deg",
            "pose_x_m",
            "pose_y_m",
            "can_bus_utilization",
            "brownout_alert",
    };

    private BufferedWriter writer;
    private final Path path;

    /** Opens (or creates) a CSV file for writing. Writes header if file is new. */
    public MatchCsvLogger(Path path) throws IOException {
        this.path = path;
        boolean isNew = !Files.exists(path) || Files.size(path) == 0;
        writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8,
                StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        if (isNew) {
            writer.write(String.join(",", HEADER));
            writer.newLine();
            writer.flush();
        }
    }

    /** Append one data snapshot as a CSV row. */
    public void log(DashboardData data) throws IOException {
        if (writer == null) return;
        StringBuilder sb = new StringBuilder();
        sb.append(data.robotTimestampSec());
        sb.append(',').append(data.batteryVoltage());
        sb.append(',').append(csvSafe(data.mode()));
        sb.append(',').append(data.enabled());
        sb.append(',').append(data.matchTimeSec());
        sb.append(',').append(data.shooterLeftRps());
        sb.append(',').append(data.shooterRightRps());
        sb.append(',').append(data.shooterAtSpeed());
        sb.append(',').append(data.intakeHomed());
        sb.append(',').append(data.intakeTiltDeg());
        sb.append(',').append(data.readyToScore());
        sb.append(',').append(csvSafe(data.readyReason()));
        sb.append(',').append(csvSafe(data.alignState()));
        sb.append(',').append(data.alignYawDeg());
        sb.append(',').append(data.alignAimErrorDeg());
        sb.append(',').append(data.alignLeadYawDeg());
        sb.append(',').append(data.alignPitchDeg());
        sb.append(',').append(data.alignTargetRps());
        sb.append(',').append(data.alignFeedGateReady());
        sb.append(',').append(data.headingDeg());
        sb.append(',').append(data.poseX_m());
        sb.append(',').append(data.poseY_m());
        sb.append(',').append(data.canBusUtilization());
        sb.append(',').append(data.brownoutAlert());
        writer.write(sb.toString());
        writer.newLine();
        writer.flush();
    }

    /** Path being written to. */
    public Path getPath() {
        return path;
    }

    @Override
    public void close() throws IOException {
        if (writer != null) {
            writer.close();
            writer = null;
        }
    }

    // ------------------------------------------------------------------
    // CSV reader for replay
    // ------------------------------------------------------------------

    /** A single row loaded from a match CSV. */
    public record CsvRow(
            double timestampSec,
            double batteryVoltage,
            String mode,
            boolean enabled,
            double matchTimeSec,
            double shooterLeftRps,
            double shooterRightRps,
            boolean shooterAtSpeed,
            boolean intakeHomed,
            double intakeTiltDeg,
            boolean readyToScore,
            String readyReason,
            String alignState,
            double alignYawDeg,
            double alignAimErrorDeg,
            double alignLeadYawDeg,
            double alignPitchDeg,
            double alignTargetRps,
            boolean alignFeedGateReady,
            double headingDeg,
            double poseXm,
            double poseYm,
            double canBusUtilization,
            boolean brownoutAlert) {}

    /** Load all rows from a CSV file produced by this logger. */
    public static List<CsvRow> loadCsv(Path path) throws IOException {
        List<CsvRow> rows = new ArrayList<>();
        try (BufferedReader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
            String headerLine = reader.readLine(); // skip header
            if (headerLine == null) return rows;
            String line;
            while ((line = reader.readLine()) != null) {
                String[] cols = line.split(",", -1);
                if (cols.length < HEADER.length) continue;
                try {
                    rows.add(new CsvRow(
                            parseDouble(cols[0]),
                            parseDouble(cols[1]),
                            cols[2],
                            Boolean.parseBoolean(cols[3]),
                            parseDouble(cols[4]),
                            parseDouble(cols[5]),
                            parseDouble(cols[6]),
                            Boolean.parseBoolean(cols[7]),
                            Boolean.parseBoolean(cols[8]),
                            parseDouble(cols[9]),
                            Boolean.parseBoolean(cols[10]),
                            cols[11],
                            cols[12],
                            parseDouble(cols[13]),
                            parseDouble(cols[14]),
                            parseDouble(cols[15]),
                            parseDouble(cols[16]),
                            parseDouble(cols[17]),
                            Boolean.parseBoolean(cols[18]),
                            parseDouble(cols[19]),
                            parseDouble(cols[20]),
                            parseDouble(cols[21]),
                            parseDouble(cols[22]),
                            Boolean.parseBoolean(cols[23])));
                } catch (NumberFormatException ignored) {
                    // skip malformed rows
                }
            }
        }
        return rows;
    }

    private static double parseDouble(String s) {
        if (s == null || s.isEmpty() || "NaN".equals(s)) return Double.NaN;
        return Double.parseDouble(s);
    }

    private static String csvSafe(String value) {
        if (value == null) return "";
        // strip commas and newlines to keep CSV simple
        return value.replace(',', ';').replace('\n', ' ').replace('\r', ' ');
    }
}
