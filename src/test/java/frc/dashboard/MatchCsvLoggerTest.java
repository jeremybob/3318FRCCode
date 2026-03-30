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
                true, mode, true, "RED", 120.0, timestamp, // connected, mode, enabled, alliance, matchTimeSec, robotTimestampSec
                1.5, 2.5, 45.0,                            // poseX_m, poseY_m, headingDeg
                Double.NaN, Double.NaN, Double.NaN,        // pigeonYawDeg, pigeonPitchDeg, pigeonRollDeg
                0.05, 1.2, 0.3, 0.4, true,                // driver turn + fieldRelativeEnabled
                false, Double.NaN, Double.NaN, Double.NaN, // heading hold
                80.0, 79.0, true,                           // shooterLeftRps, shooterRightRps, shooterAtSpeed
                true, false, 30.0, 5.0, 3.0, 2.0,         // intake + feeder + hopper
                false, 0.0, 0.0,                            // climber
                // Alignment pipeline
                "TRACKING", true, true, true, true,        // alignState, alignCommandActive, alignHasTarget, alignGeometryFeasible, alignHasShootableTarget
                1.2, 0.3, 0.9, -0.5,                      // alignHeadingErrorDeg, alignAimErrorDeg, alignDistanceM, alignTargetRps
                true, "",                                   // alignFeedGateReady, alignAbortReason
                true, "All conditions met",                // readyToScore, readyReason
                true, 10.0,                                 // hubActive, hubSecondsToNextShift
                battery, false, false,                      // batteryVoltage, brownoutAlert, isBrownout
                "TwoNoteCenter", "CUSTOM DASHBOARD", new String[]{"TwoNoteCenter"}, false, // auto
                Double.NaN, Double.NaN, Double.NaN,        // autoStart pose
                1, "TestEvent",                             // matchNumber, eventName
                // Camera / vision
                true, "PHOTON_CONNECTED", "OV9281", "PHOTONVISION", // cameraConnected, cameraStatus, cameraName, activeCameraType
                1, true, 1.2, 2.5, 3, 120.0,              // visionTagId, visionHasTarget, visionHeadingErrorDeg, visionDistanceM, visionHubTagCount, visionTargetTimestampSec
                0.5, 0, 0,                                  // canBusUtilization, canReceiveErrorCount, canTransmitErrorCount
                10.0, 20.0, 30.0, 40.0,                    // swerve angles
                // CANCoder raw + offset (8 fields)
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                // CANCoder pos + absRaw + ok (12 fields)
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                true, true, true, true,
                // Motor temps
                40.0, 41.0, 42.0, 43.0, 50.0, 51.0,
                // Controller diagnostics
                "", "", 0, 0.0, "",
                // Swerve validation
                false, "NONE", "--", "IDLE", "Idle", 0.0, 0.0,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                // Ack
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
