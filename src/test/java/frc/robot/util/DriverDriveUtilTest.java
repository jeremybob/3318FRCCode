package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class DriverDriveUtilTest {

    @Test
    void translationStillRespondsInsideNewRotationOnlyGap() {
        DriverDriveUtil.DriveRequest request = DriverDriveUtil.shapeDrive(
                0.12,
                0.0,
                0.12,
                false,
                true);

        assertTrue(request.xVelocityMps() > 0.0);
        assertEquals(0.0, request.omegaRadPerSec(), 1e-9);
    }

    @Test
    void turnInputBelowRotationDeadbandDoesNotCommandOmega() {
        DriverDriveUtil.DriveRequest request = DriverDriveUtil.shapeDrive(
                0.0,
                0.0,
                Constants.Swerve.ROTATION_JOYSTICK_DEADBAND - 0.01,
                false,
                true);

        assertEquals(0.0, request.omegaRadPerSec(), 1e-9);
    }

    @Test
    void turnInputAboveRotationDeadbandCommandsOmega() {
        DriverDriveUtil.DriveRequest request = DriverDriveUtil.shapeDrive(
                0.0,
                0.0,
                Constants.Swerve.ROTATION_JOYSTICK_DEADBAND + 0.05,
                false,
                true);

        assertTrue(request.omegaRadPerSec() > 0.0);
    }
}
