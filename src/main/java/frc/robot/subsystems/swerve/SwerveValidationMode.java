package frc.robot.subsystems.swerve;

import frc.robot.Constants;

public enum SwerveValidationMode {
    STEER_POSITIVE("STEER_POSITIVE", "Steer +",
            0.0, Constants.Swerve.VALIDATION_STEER_DUTY_CYCLE),
    STEER_NEGATIVE("STEER_NEGATIVE", "Steer -",
            0.0, -Constants.Swerve.VALIDATION_STEER_DUTY_CYCLE),
    DRIVE_FORWARD("DRIVE_FORWARD", "Drive +",
            Constants.Swerve.VALIDATION_DRIVE_DUTY_CYCLE, 0.0),
    DRIVE_REVERSE("DRIVE_REVERSE", "Drive -",
            -Constants.Swerve.VALIDATION_DRIVE_DUTY_CYCLE, 0.0);

    private final String token;
    private final String displayName;
    private final double drivePercent;
    private final double steerPercent;

    SwerveValidationMode(String token, String displayName, double drivePercent, double steerPercent) {
        this.token = token;
        this.displayName = displayName;
        this.drivePercent = drivePercent;
        this.steerPercent = steerPercent;
    }

    public String token() {
        return token;
    }

    public String displayName() {
        return displayName;
    }

    public double drivePercent() {
        return drivePercent;
    }

    public double steerPercent() {
        return steerPercent;
    }

    public static SwerveValidationMode fromToken(String token) {
        if (token == null) {
            return null;
        }
        for (SwerveValidationMode mode : values()) {
            if (mode.token.equalsIgnoreCase(token.trim())
                    || mode.displayName.equalsIgnoreCase(token.trim())) {
                return mode;
            }
        }
        return null;
    }
}
