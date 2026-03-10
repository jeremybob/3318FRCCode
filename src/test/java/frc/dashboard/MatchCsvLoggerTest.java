package frc.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class MatchCsvLoggerTest {

    @TempDir
    Path tempDir;

    private static DashboardData sampleData(double timestamp, double battery, String mode) {
        return new DashboardData(
                true, mode, true, "RED", 120.0, timestamp,
                1.5, 2.5, 45.0,
                Double.NaN, Double.NaN, Double.NaN,
                80.0, 79.0, true,
                true, false, 30.0, 5.0, 3.0, 2.0,
                false, 0.0, 0.0,
                "TRACKING", true, true, true, true, 1.2, 0.3, 0.9, -0.5,
                74.0, 0.2, -0.3, 0.4, -0.1, 0.8, 0.45, true, "",
                true, "All conditions met",
                true, 10.0,
                battery, false, false,
                "TwoNoteCenter", "CUSTOM DASHBOARD", new String[]{"TwoNoteCenter"}, false,
                Double.NaN, Double.NaN, Double.NaN,
                1, "TestEvent",
                true, "OK", 0, "cam", "/dev/video0", "", "", 100, 10.0,
                1, true, 1.2, -0.5, 2.5, 48.0, 10.0,
                0.5, 0, 0,
                10.0, 20.0, 30.0, 40.0,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                true, true, true, true,
                40.0, 41.0, 42.0, 43.0, 50.0, 51.0,
                "", "", 0, 0.0, "",
                false, "NONE", "--", "IDLE", "Idle", 0.0, 0.0,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                "ZERO_HEADING", "OK", 1, "Done", 10.0);
    }

    @Test
    void writeAndReadBackRoundTrip() throws IOException {
        Path csvPath = tempDir.resolve("test_match.csv");

        try (MatchCsvLogger logger = new MatchCsvLogger(csvPath)) {
            logger.log(sampleData(1.0, 12.5, "TELEOP"));
            logger.log(sampleData(2.0, 12.3, "TELEOP"));
            logger.log(sampleData(3.0, 11.8, "AUTONOMOUS"));
        }

        List<MatchCsvLogger.CsvRow> rows = MatchCsvLogger.loadCsv(csvPath);
        assertEquals(3, rows.size());

        MatchCsvLogger.CsvRow first = rows.get(0);
        assertEquals(1.0, first.timestampSec(), 0.001);
        assertEquals(12.5, first.batteryVoltage(), 0.001);
        assertEquals("TELEOP", first.mode());
        assertTrue(first.enabled());
        assertEquals(80.0, first.shooterLeftRps(), 0.1);

        MatchCsvLogger.CsvRow last = rows.get(2);
        assertEquals(3.0, last.timestampSec(), 0.001);
        assertEquals("AUTONOMOUS", last.mode());
    }

    @Test
    void csvHeaderIsWrittenOnce() throws IOException {
        Path csvPath = tempDir.resolve("header_test.csv");

        try (MatchCsvLogger logger = new MatchCsvLogger(csvPath)) {
            logger.log(sampleData(1.0, 12.0, "TELEOP"));
        }

        List<String> lines = Files.readAllLines(csvPath);
        assertTrue(lines.size() >= 2);
        assertTrue(lines.get(0).startsWith("timestamp_sec,"));
        // Second line should be data, not another header
        assertFalse(lines.get(1).startsWith("timestamp_sec"));
    }

    @Test
    void loadEmptyFileReturnsEmptyList() throws IOException {
        Path csvPath = tempDir.resolve("empty.csv");
        Files.createFile(csvPath);

        List<MatchCsvLogger.CsvRow> rows = MatchCsvLogger.loadCsv(csvPath);
        assertTrue(rows.isEmpty());
    }
}
