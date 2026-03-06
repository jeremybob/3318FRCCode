package frc.robot.subsystems.swerve;

public enum SwerveCorner {
    FL("FL", "Front Left"),
    FR("FR", "Front Right"),
    BL("BL", "Back Left"),
    BR("BR", "Back Right");

    private final String token;
    private final String displayName;

    SwerveCorner(String token, String displayName) {
        this.token = token;
        this.displayName = displayName;
    }

    public String token() {
        return token;
    }

    public String displayName() {
        return displayName;
    }

    public static SwerveCorner fromToken(String token) {
        if (token == null) {
            return null;
        }
        for (SwerveCorner corner : values()) {
            if (corner.token.equalsIgnoreCase(token.trim())
                    || corner.displayName.equalsIgnoreCase(token.trim())) {
                return corner;
            }
        }
        return null;
    }
}
