// ============================================================================
// FILE: src/main/java/frc/robot/Constants.java
//
// PURPOSE: One central place for every number used anywhere in the robot code.
//   If you need to change a CAN ID, a gear ratio, a PID gain, etc., you do it
//   HERE — not scattered across 10 files.
//
// TUNING NOTES FOR STUDENTS:
//   Values marked "TUNE ME" need to be measured or adjusted on the real robot.
//   Everything else should work as-is for MK4 L2 + Falcon 500 (TalonFX) + Pigeon 2.
// ============================================================================
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // Private constructor — nobody should create a "Constants" object.
    // This class is just a container for static values.
    private Constants() {}

    // =========================================================================
    // CAN BUS IDs
    // These numbers must match whatever you set in Phoenix Tuner X and REV
    // Hardware Client. Change them here to match your actual robot wiring.
    // =========================================================================
    public static final class CAN {
        // Pigeon 2 IMU (gyro) — used for field-oriented drive
        public static final int PIGEON = 0;

        // ---- Swerve drive motors & encoders ----
        // Each corner has 3 devices: Drive TalonFX, Steer TalonFX, CANcoder
        public static final int FRONT_LEFT_DRIVE    = 1;
        public static final int FRONT_LEFT_STEER    = 2;
        public static final int FRONT_LEFT_CANCODER = 3;

        public static final int FRONT_RIGHT_DRIVE    = 4;
        public static final int FRONT_RIGHT_STEER    = 5;
        public static final int FRONT_RIGHT_CANCODER = 6;

        public static final int BACK_LEFT_DRIVE    = 7;
        public static final int BACK_LEFT_STEER    = 8;
        public static final int BACK_LEFT_CANCODER = 9;

        // NOTE: Back-right IDs jump from 9 to 12 — IDs 10 and 11 are shooter.
        public static final int BACK_RIGHT_DRIVE    = 12;
        public static final int BACK_RIGHT_STEER    = 13;
        public static final int BACK_RIGHT_CANCODER = 14;

        // ---- Shooter (TalonFX / Kraken X60) ----
        public static final int SHOOTER_LEFT  = 10;
        public static final int SHOOTER_RIGHT = 11;

        // ---- Intake ----
        public static final int INTAKE_TILT_NEO = 15;  // REV SparkMax
        public static final int INTAKE_ROLLER   = 16;  // TalonFX (Kraken)

        // ---- Hopper & Feeder (REV SparkMax) ----
        public static final int HOPPER_FLOOR_NEO = 17;
        public static final int FEEDER_NEO       = 18;

        // ---- Climber (TalonFX) ----
        public static final int CLIMBER_LEADER   = 20;
        public static final int CLIMBER_FOLLOWER = 21;
    }

    // =========================================================================
    // DIGITAL I/O PORTS (on the roboRIO)
    // =========================================================================
    public static final class DIO {
        // Intake tilt home limit switch — plugs into DIO port 0
        public static final int INTAKE_HOME_SWITCH = 0;
    }

    // =========================================================================
    // SWERVE DRIVE CONSTANTS
    //
    // Hardware: SDS MK4 L2 modules, Falcon 500 motors (TalonFX), CTRE CANcoder, Pigeon 2
    //
    // GEAR RATIOS (from SDS documentation):
    //   Drive:  6.75 : 1  (6.75 motor rotations per 1 wheel rotation)
    //   Steer: 12.8  : 1  (12.8 motor rotations per 1 wheel rotation)
    //
    // WHEEL: 4-inch diameter billet wheel
    //
    // PID SOURCE: CTRE official Phoenix 6 TunerConstants defaults for MK4/Falcon 500
    //   with FusedCANcoder, validated across multiple competition teams (364, 6328, etc.)
    // =========================================================================
    public static final class Swerve {

        // ---- Physical dimensions ----
        // Robot frame is 27" × 27" (686mm × 686mm).
        // SDS MK4 modules: wheel center is approximately 2.625" inboard from the
        // outer frame rail. Track/wheelbase = 27" - 2 × 2.625" = 21.75" = 0.5525m.
        // Source: SDS MK4 module drawing — wheel center offset from mounting face.
        public static final double TRACK_WIDTH_M = Units.inchesToMeters(21.75);  // 0.5525 m

        // Distance between the FRONT and BACK wheel centers (meters)
        public static final double WHEEL_BASE_M = Units.inchesToMeters(21.75);   // 0.5525 m

        // ---- Wheel & gear math ----
        // 4-inch wheel = 0.1016 m diameter
        public static final double WHEEL_DIAMETER_M     = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M; // ~0.319 m

        // MK4 L2 gear ratios from the SDS datasheet
        public static final double DRIVE_GEAR_RATIO = 6.75;  // motor turns per 1 wheel turn
        public static final double STEER_GEAR_RATIO = 12.8;  // motor turns per 1 wheel turn

        // ---- Speed limits ----
        // Falcon 500 free speed ≈ 6380 RPM = 106.33 RPS
        // Wheel free speed = 106.33 / 6.75 = 15.75 RPS
        // Max speed = 15.75 × 0.319 ≈ 5.02 m/s theoretical
        // We cap at 4.5 to leave headroom for battery sag.
        public static final double MAX_TRANSLATION_MPS = 4.5;   // m/s
        public static final double MAX_ROTATION_RADPS  = 10.0;  // rad/s

        // The maximum physical RPS the drive motor can spin (used for velocity commands)
        // Falcon 500 free speed = 6380 RPM = 106.33 RPS at 12V
        public static final double DRIVE_MOTOR_FREE_SPEED_RPS = 6380.0 / 60.0;

        // ---- Module wheel locations (relative to robot center) ----
        // WPILib uses +X = forward, +Y = left
        public static final Translation2d FRONT_LEFT_LOCATION  =
                new Translation2d( WHEEL_BASE_M / 2.0,  TRACK_WIDTH_M / 2.0);
        public static final Translation2d FRONT_RIGHT_LOCATION =
                new Translation2d( WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0);
        public static final Translation2d BACK_LEFT_LOCATION   =
                new Translation2d(-WHEEL_BASE_M / 2.0,  TRACK_WIDTH_M / 2.0);
        public static final Translation2d BACK_RIGHT_LOCATION  =
                new Translation2d(-WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0);

        // ---- CANcoder offsets (in ROTATIONS, -0.5 to +0.5) ----
        // HOW TO FIND THESE:
        //   1. Physically align all wheels pointing STRAIGHT FORWARD.
        //   2. Open Phoenix Tuner X → select each CANcoder.
        //   3. Read the "Absolute Position No Offset" signal.
        //   4. NEGATE that reading and enter it below.
        //      Example: if Tuner X shows 0.104, set -0.104 here.
        //
        // CALIBRATE BEFORE DRIVING! These MUST be set per-robot.
        // Use the "CalibrateCANcoders" auto in SmartDashboard to print current
        // raw readings — then negate and paste here.
        public static final double FL_CANCODER_OFFSET_ROT = 0.0;  // CALIBRATE ME
        public static final double FR_CANCODER_OFFSET_ROT = 0.0;  // CALIBRATE ME
        public static final double BL_CANCODER_OFFSET_ROT = 0.0;  // CALIBRATE ME
        public static final double BR_CANCODER_OFFSET_ROT = 0.0;  // CALIBRATE ME

        // ---- Drive motor PID (VelocityVoltage, Phoenix 6) ----
        // Source: CTRE official Phoenix 6 TunerConstants.java (driveGains).
        // kS = voltage to overcome static friction. CTRE default = 0 for new motors.
        // kV = voltage per RPS feedforward. 0.124 V/RPS = 12V / ~97 RPS free speed.
        //      This is the primary speed controller — get it right first.
        // kP = proportional correction. 0.1 works well for most MK4 setups.
        //      Increase if the wheel doesn't track commanded speed closely.
        public static final double DRIVE_kS = 0.0;    // Volts (CTRE default)
        public static final double DRIVE_kV = 0.124;   // V/RPS (CTRE default for Falcon 500)
        public static final double DRIVE_kP = 0.1;     // V/RPS error (CTRE default)

        // ---- Steer motor PID (PositionVoltage + FusedCANcoder, Phoenix 6) ----
        // Source: CTRE official Phoenix 6 TunerConstants.java (steerGains).
        // These values are tuned for Phoenix Pro + FusedCANcoder feedback.
        // kP = 100 V/rotation — modules snap to target angle quickly.
        // kD = 0.5 — damping prevents oscillation around the setpoint.
        // kS = 0.1 — overcomes static friction in the steer gearbox.
        // kV = 1.91 — feedforward for the steer motor velocity.
        // If NOT using Phoenix Pro (using Phoenix 6 non-Pro), reduce kP to ~50.
        public static final double STEER_kP = 100.0;   // V/rotation (CTRE Phoenix Pro default)
        public static final double STEER_kD = 0.5;     // V/(rotation/sec) (CTRE default)
        public static final double STEER_kS = 0.1;     // Volts (CTRE default)
        public static final double STEER_kV = 1.91;    // V/RPS (CTRE default)

        // ---- Joystick deadband ----
        // Joystick axes within this range of zero are treated as zero.
        // This prevents the robot from creeping when the stick isn't centered.
        public static final double JOYSTICK_DEADBAND = 0.1;

        // ---- Precision mode ----
        // Hold driver right trigger to scale max speed down for fine positioning.
        // 0.25 = 25% of normal speed when precision mode is active.
        public static final double PRECISION_SPEED_SCALE = 0.25;
    }

    // =========================================================================
    // SHOOTER CONSTANTS
    //
    // Hardware: Two Kraken X60 (TalonFX), 4-inch wheels, 1:1 gearing
    // =========================================================================
    public static final class Shooter {
        // ---- Shooter wheel hardware specs ----
        // Kraken X60 (TalonFX), 4-inch wheels, 1:1 gearing.
        public static final double WHEEL_DIAMETER_M      = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M; // ~0.319m
        public static final double GEAR_RATIO             = 1.0;  // motor:wheel

        // ---- Distance-based shot physics ----
        // The shooter fires at a fixed mechanical angle. Given the distance to the
        // HUB (from vision), we solve projectile motion for the required surface
        // velocity, then convert to wheel RPS.
        //
        // Equation:
        //   v² = (g * d²) / (2 * cos²θ * (d * tanθ - Δh))
        //   where Δh = target_height - shooter_exit_height
        //
        // TUNE ME: Measure the actual launch angle with a protractor or slow-mo video.
        public static final double SHOT_ANGLE_DEG        = 45.0;  // TUNE ME
        // Height of the shooter exit above the floor (20 inches).
        public static final double SHOOTER_EXIT_HEIGHT_M = Units.inchesToMeters(20.0);
        // Height of the HUB scoring opening (may differ from tag height of 1.124m).
        public static final double HUB_SCORING_HEIGHT_M  = 1.124; // TUNE ME

        // RPS clamps — keep within motor/mechanism limits.
        // Kraken free speed ≈ 100 RPS at 12V; leave headroom for voltage droop.
        public static final double MIN_SHOT_RPS = 20.0;
        public static final double MAX_SHOT_RPS = 90.0;

        // Default warmup speed used during SPIN_UP before distance is known.
        // Also used as fallback if the distance calculation fails.
        public static final double TARGET_RPS = 60.0;  // TUNE ME
        // Driver override speed when shooting without alignment/vision checks.
        public static final double FALLBACK_RPS = 52.0; // TUNE ME

        // Shooter wheel PID (VelocityVoltage)
        // Kraken X60 at 1:1, 4" wheel, 12V supply:
        //   kV = 12V / 100 RPS ≈ 0.12 (primary feedforward)
        //   kS = static friction overcome. Measured on typical Kraken = ~0.15V.
        //   kP = proportional correction for steady-state error.
        // Source: Typical Kraken X60 shooter values from competition robots.
        public static final double SHOOTER_kS = 0.15;   // Volts (measured typical Kraken)
        public static final double SHOOTER_kV = 0.12;   // 12V / 100 RPS free speed
        public static final double SHOOTER_kP = 0.08;   // V/RPS error (slightly aggressive for fast spinup)

        // How close the wheels need to be to target before we consider "at speed"
        public static final double TOLERANCE_RPS = 1.5;

        // Max time to wait for wheels to reach speed before shooting anyway
        public static final double AT_SPEED_TIMEOUT_SEC = 1.5;

        // "Clear" pulse: briefly run feeder backward to prevent double-feeding
        public static final double CLEAR_POWER    = -0.25;
        public static final double CLEAR_TIME_SEC =  0.15;

        // Feed: all three mechanisms push the game piece into the shooter
        public static final double FEED_POWER    = 0.80;
        public static final double FEED_TIME_SEC = 0.75;
    }

    // =========================================================================
    // INTAKE CONSTANTS
    // =========================================================================
    public static final class Intake {
        // How many degrees the encoder reports per motor revolution.
        // With a 10:1 reduction, each motor rev = 360/10 = 36 degrees.
        // TUNE ME: Check your actual tilt gearbox ratio!
        public static final double TILT_POS_CONV_DEG = 360.0 / 10.0;  // TUNE ME

        // Tilt position PID (SparkMax built-in)
        public static final double TILT_kP = 0.05;  // TUNE ME

        // Power used to slowly drive toward the home limit switch
        public static final double HOME_POWER       = -0.15;
        public static final double HOME_TIMEOUT_SEC =  2.0;

        // Target angle for the intake to be "down" and collecting game pieces
        public static final double INTAKE_DOWN_DEG = 45.0;  // TUNE ME

        // Target angle for the intake to be stowed (up / home position)
        public static final double INTAKE_STOW_DEG = 0.0;

        // Software limits for the tilt arm (in degrees from the homed position).
        // Prevents commanding the arm into the chassis or past its mechanical travel.
        // TUNE ME: Set these to the actual min/max safe travel of YOUR intake arm.
        public static final double TILT_MIN_DEG = -5.0;   // small margin past home
        public static final double TILT_MAX_DEG = 90.0;   // fully deployed

        // Current limit to protect the NEO and gearbox during homing stalls
        public static final int TILT_CURRENT_LIMIT_A = 40;

        // Roller current limit (TalonFX / Kraken)
        public static final int ROLLER_STATOR_LIMIT_A = 60;  // prevents jam burnout

        // ---- Stall detection (roller motor) ----
        // If roller current stays above STALL_CURRENT_THRESHOLD_A for STALL_TIME_SEC,
        // the roller is jammed and will automatically reverse and retry.
        public static final double STALL_CURRENT_THRESHOLD_A = 35.0;  // TUNE ME
        public static final double STALL_TIME_SEC             = 0.3;
        public static final double STALL_REVERSE_POWER        = -0.5;
        public static final double STALL_REVERSE_TIME_SEC     = 0.25;
        public static final int    STALL_MAX_RETRIES          = 3;
    }

    // =========================================================================
    // HOPPER CONSTANTS
    // =========================================================================
    public static final class Hopper {
        // Mechanical reduction from motor to hopper roller/floor.
        public static final double GEAR_RATIO = 8.0;
    }

    // =========================================================================
    // REV NEO MOTOR DEFAULTS
    // =========================================================================
    public static final class NeoMotors {
        // Generic current limit applied to all SparkMax motors without a specific one
        public static final int DEFAULT_CURRENT_LIMIT_A = 40;
    }

    // =========================================================================
    // VISION / PHOTONVISION CONSTANTS
    // =========================================================================
    public static final class Vision {
        // Name must match what you set in PhotonVision's web interface
        public static final String CAMERA_NAME = "PiCamera";  // TUNE ME

        // Alignment is "good enough" once yaw error is within this many degrees
        public static final double YAW_TOLERANCE_DEG = 2.0;
        // Tolerate brief vision dropouts instead of immediately canceling a shot.
        public static final double TARGET_LOSS_TOLERANCE_SEC = 0.35; // TUNE ME
        // Feasible vertical angle band for a valid shot solution from the camera.
        public static final double MIN_SHOT_PITCH_DEG = -12.0; // TUNE ME
        public static final double MAX_SHOT_PITCH_DEG =  18.0; // TUNE ME

        // PD controller for rotating toward a vision target
        public static final double TURN_kP     = 0.05;
        public static final double TURN_kD     = 0.005;
        public static final double MAX_ROT_CMD = 0.6;  // max rotation power during alignment

        // ---- Camera mount position (for AprilTag pose estimation) ----
        // Transform from robot center to camera lens.
        // TUNE ME: Measure the actual camera mount position on your robot!
        // Translation: (forward_m, left_m, up_m) from robot center
        // Rotation: (roll, pitch, yaw) in radians
        public static final double CAMERA_FORWARD_M  = 0.25;  // TUNE ME
        public static final double CAMERA_LEFT_M     = 0.0;   // TUNE ME
        public static final double CAMERA_UP_M       = 0.50;  // TUNE ME
        public static final double CAMERA_PITCH_RAD  = Math.toRadians(-15.0); // tilted down, TUNE ME
        public static final double CAMERA_YAW_RAD    = 0.0;

        // Vision pose estimation trust (standard deviations).
        // Higher values = less trust in vision, more trust in wheel odometry.
        // Start conservative and tighten once you validate vision accuracy on the field.
        public static final double VISION_STD_DEV_X_M          = 0.5;  // meters
        public static final double VISION_STD_DEV_Y_M          = 0.5;  // meters
        public static final double VISION_STD_DEV_HEADING_RAD  = Math.toRadians(10); // radians

        // Maximum pose ambiguity for single-tag results (0.0 = perfect, 1.0 = worst).
        // Multi-tag results are inherently more accurate and skip this filter.
        public static final double MAX_POSE_AMBIGUITY = 0.2;

        // ---- Alliance-specific HUB tag IDs for targeting ----
        // AlignAndShootCommand should ONLY aim at your alliance's HUB tags.
        // These are the z=1.124m (44.25in) tags mounted on each alliance's HUB.
        // Each HUB has 4 faces with 2 tags per face = 8 tags per HUB.
        // Red HUB center ≈ (11.92, 4.03), Blue HUB center ≈ (4.63, 4.03).
        public static final int[] RED_HUB_TAG_IDS  = {2, 3, 4, 5, 8, 9, 10, 11};
        public static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
    }

    // =========================================================================
    // AUTONOMOUS CONSTANTS
    // =========================================================================
    public static final class Auto {
        // Timeout for AutoShoot named command in auto mode.
        public static final double AUTO_SHOOT_TIMEOUT_SEC = 6.0;
    }

    // =========================================================================
    // CLIMBER CONSTANTS
    // =========================================================================
    public static final class Climber {
        // Software limits — climber cannot go past these positions (in motor rotations)
        public static final double REV_SOFT_LIMIT = 0.0;    // fully retracted
        public static final double FWD_SOFT_LIMIT = 100.0;  // fully extended

        // Target position for Level 1 automatic climb
        public static final double LEVEL1_TARGET_ROT = 45.0;  // TUNE ME
        public static final double LEVEL1_TOLERANCE_ROT = 1.0;
        public static final double LEVEL1_TIMEOUT_SEC = 4.0;

        // Position PID for the winch
        public static final double CLIMBER_kP = 30.0;  // TUNE ME
        public static final double CLIMBER_kD = 0.0;

        // Stator current limit for the winch motors (they pull hard!)
        public static final int STATOR_LIMIT_A = 60;

        // Manual joystick power multiplier for operator control
        public static final double MANUAL_POWER_SCALE = 0.6;
    }

    // =========================================================================
    // OPERATOR INTERFACE (controller port numbers)
    // =========================================================================
    public static final class OI {
        public static final int DRIVER_PORT   = 0;  // Driver Xbox controller
        public static final int OPERATOR_PORT = 1;  // Operator Xbox controller
    }
}
