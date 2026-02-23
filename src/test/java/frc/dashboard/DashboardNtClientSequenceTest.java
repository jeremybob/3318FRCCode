package frc.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DashboardNtClientSequenceTest {

    @Test
    void nextCommandSequenceUsesTopicValueWhenGreater() {
        assertEquals(6, DashboardNtClient.nextCommandSequence(2, 5));
    }

    @Test
    void nextCommandSequenceUsesLocalValueWhenGreater() {
        assertEquals(8, DashboardNtClient.nextCommandSequence(7, 3));
    }
}
