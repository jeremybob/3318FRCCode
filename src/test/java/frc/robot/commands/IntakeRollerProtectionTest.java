package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IntakeRollerProtectionTest {

    @Test
    void entersReversingAfterSustainedStall() {
        IntakeRollerProtection protection = new IntakeRollerProtection();

        IntakeRollerProtection.Update update = protection.update(
                40.0,
                35.0,
                true,
                false,
                3);

        assertEquals(IntakeRollerProtection.State.REVERSING, protection.state());
        assertEquals(1, protection.retryCount());
        assertTrue(update.restartReverseTimer());
        assertTrue(update.enteredReversing());
        assertFalse(update.enteredLockout());
        assertEquals(-0.5, protection.commandedPower(0.6, -0.5), 1e-9);
    }

    @Test
    void locksOutAfterRetriesAreExhausted() {
        IntakeRollerProtection protection = new IntakeRollerProtection();

        for (int attempt = 0; attempt < 3; attempt++) {
            protection.update(40.0, 35.0, true, false, 3);
            protection.update(0.0, 35.0, false, true, 3);
        }

        IntakeRollerProtection.Update update = protection.update(
                40.0,
                35.0,
                true,
                false,
                3);

        assertEquals(IntakeRollerProtection.State.LOCKED_OUT, protection.state());
        assertEquals(3, protection.retryCount());
        assertTrue(update.enteredLockout());
        assertTrue(protection.isLockedOut());
        assertEquals(0.0, protection.commandedPower(0.6, -0.5), 1e-9);
    }
}
