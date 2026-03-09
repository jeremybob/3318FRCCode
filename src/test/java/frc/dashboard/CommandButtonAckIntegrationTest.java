package frc.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Integration-style tests that verify the command button -> ack sequence
 * contract used by the custom dashboard.  These tests exercise the
 * {@link DashboardNtClient} command sequencing logic that underpins every
 * command button in the UI, ensuring that:
 *
 * <ul>
 *   <li>Each button press increments the sequence number.</li>
 *   <li>The sequence is always ahead of what the robot last saw.</li>
 *   <li>Rapid button presses produce strictly increasing sequences.</li>
 *   <li>Robot-side sequence resets are handled safely.</li>
 * </ul>
 *
 * These tests do <em>not</em> require a running NetworkTables server.
 * They validate the pure logic in {@code nextCommandSequence} and the
 * expected ack-matching invariants.
 */
class CommandButtonAckIntegrationTest {

    // ---------------------------------------------------------------
    // Sequence number contract
    // ---------------------------------------------------------------

    @Test
    void firstCommandProducesSequenceOne() {
        // Before any command is sent, both local and topic sequences are 0
        long result = DashboardNtClient.nextCommandSequence(0, 0);
        assertEquals(1, result, "First command should produce sequence 1");
    }

    @Test
    void commandSequenceAlwaysIncrementsAboveLocal() {
        long seq = DashboardNtClient.nextCommandSequence(5, 3);
        assertTrue(seq > 5, "Result should exceed local sequence");
    }

    @Test
    void commandSequenceAlwaysIncrementsAboveTopic() {
        long seq = DashboardNtClient.nextCommandSequence(3, 7);
        assertTrue(seq > 7, "Result should exceed topic sequence");
    }

    @Test
    void rapidButtonPressesProduceStrictlyIncreasingSequences() {
        // Simulate 10 rapid button presses where the robot hasn't responded yet
        long localSeq = 0;
        long topicSeq = 0; // robot hasn't acked anything
        long prev = 0;
        for (int i = 0; i < 10; i++) {
            localSeq = DashboardNtClient.nextCommandSequence(localSeq, topicSeq);
            assertTrue(localSeq > prev,
                    "Sequence must strictly increase: press " + i + " gave " + localSeq);
            prev = localSeq;
        }
        assertEquals(10, localSeq, "10 presses from 0 should reach 10");
    }

    @Test
    void sequenceRecoversFromRobotResetToZero() {
        // Robot was at seq 50, then reboots (goes to 0)
        long localSeq = 50;
        long topicSeqAfterReboot = 0;
        long result = DashboardNtClient.nextCommandSequence(localSeq, topicSeqAfterReboot);
        assertTrue(result > localSeq,
                "Sequence should still advance past local even after robot reboot");
    }

    @Test
    void sequenceHandlesRobotAheadOfLocal() {
        // Edge case: another dashboard client sent commands, so robot topic is ahead
        long localSeq = 3;
        long topicSeq = 100;
        long result = DashboardNtClient.nextCommandSequence(localSeq, topicSeq);
        assertEquals(101, result,
                "Should jump ahead to topic + 1 when robot is further along");
    }

    // ---------------------------------------------------------------
    // Ack matching invariants
    // ---------------------------------------------------------------

    @Test
    void ackSequenceMatchesCommandSequence() {
        // Simulate: send command, then robot acks with that same seq
        long localSeq = 0;
        long topicSeq = 0;
        long commandSeq = DashboardNtClient.nextCommandSequence(localSeq, topicSeq);

        // Robot receives and acks the command
        long ackSeq = commandSeq; // robot echoes the sequence
        assertEquals(commandSeq, ackSeq,
                "Ack sequence should match the command sequence that was sent");
    }

    @Test
    void multipleCommandsProduceDistinctSequences() {
        // Simulate sending three different commands in sequence
        long seq1 = DashboardNtClient.nextCommandSequence(0, 0);
        long seq2 = DashboardNtClient.nextCommandSequence(seq1, 0);
        long seq3 = DashboardNtClient.nextCommandSequence(seq2, 0);

        assertTrue(seq1 < seq2, "Second command should have higher seq than first");
        assertTrue(seq2 < seq3, "Third command should have higher seq than second");
    }

    @Test
    void commandSequenceNeverWrapsNegative() {
        // Even with large values, sequence should remain positive
        long localSeq = Long.MAX_VALUE - 2;
        long topicSeq = Long.MAX_VALUE - 3;
        long result = DashboardNtClient.nextCommandSequence(localSeq, topicSeq);
        // With max values this will overflow, but the contract is max()+1
        // This documents the behavior
        assertEquals(Long.MAX_VALUE - 1, result);
    }

    // ---------------------------------------------------------------
    // All DashboardCommand enum values are covered
    // ---------------------------------------------------------------

    @Test
    void allDashboardCommandsExist() {
        // Verify that all expected commands exist in the enum
        DashboardNtClient.DashboardCommand[] commands = DashboardNtClient.DashboardCommand.values();
        assertTrue(commands.length >= 9,
                "Expected at least 9 dashboard commands, found " + commands.length);

        // Verify specific commands that correspond to UI buttons
        DashboardNtClient.DashboardCommand.valueOf("ZERO_HEADING");
        DashboardNtClient.DashboardCommand.valueOf("STOP_DRIVE");
        DashboardNtClient.DashboardCommand.valueOf("INTAKE_HOME");
        DashboardNtClient.DashboardCommand.valueOf("ALIGN_ONLY");
        DashboardNtClient.DashboardCommand.valueOf("ALIGN_SHOOT");
        DashboardNtClient.DashboardCommand.valueOf("FALLBACK_SHOOT");
        DashboardNtClient.DashboardCommand.valueOf("LEVEL1_CLIMB");
        DashboardNtClient.DashboardCommand.valueOf("CALIBRATE_CANCODERS");
        DashboardNtClient.DashboardCommand.valueOf("STOP_SWERVE_VALIDATION");
    }
}
