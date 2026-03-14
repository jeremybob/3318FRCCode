// ============================================================================
// FILE: src/main/java/frc/robot/Constants.java
//
// PURPOSE: One central place for every number used anywhere in the robot code.
//   If you need to change a CAN ID, a gear ratio, a PID gain, etc., you do it
//   HERE — not scattered across 10 files.
//
// TUNING NOTES FOR STUDENTS:
//   Values marked "TUNE ME" need to be measured or adjusted on the real robot.
//   Everything else should work as-is for MK4i L2 + Falcon 500 (TalonFX) + Pigeon 2.
// ============================================================================
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.LinkedHashMap;
import java.util.Map;

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
        // CTRE CAN bus name:
        //   "rio" for roboRIO CAN, or your CANivore name (e.g. "canivore")
        public static final String CTRE_CAN_BUS = "rio";

        // Pigeon 2 IMU (gyro) — used for field-oriented drive
        public static final int PIGEON = 13;

        // ---- Swerve drive motors & encoders ----
        // Each corner has 3 devices: Drive TalonFX, Steer TalonFX, CANcoder
        public static final int FRONT_LEFT_DRIVE    = 3;
        public static final int FRONT_LEFT_STEER    = 4;
        public static final int FRONT_LEFT_CANCODER = 9;

        public static final int FRONT_RIGHT_DRIVE    = 1;
        public static final int FRONT_RIGHT_STEER    = 2;
        public static final int FRONT_RIGHT_CANCODER = 10;

        public static final int BACK_LEFT_DRIVE    = 7;
        public static final int BACK_LEFT_STEER    = 8;
        public static final int BACK_LEFT_CANCODER = 12;

        // NOTE: Back-right IDs are 5/6/11. IDs 16-17 are shooter, 19 is hopper.
        public static final int BACK_RIGHT_DRIVE    = 5;
        public static final int BACK_RIGHT_STEER    = 6;
        public static final int BACK_RIGHT_CANCODER = 11;

        // ---- Shooter (TalonFX / Kraken X60) ----
        public static final int SHOOTER_LEFT  = 16;
        public static final int SHOOTER_RIGHT = 17;

        // ---- Intake ----
        public static final int INTAKE_TILT_NEO = 14;  // REV SparkMax
        public static final int INTAKE_ROLLER   = 15;  // TalonFX (Kraken)

        // ---- Hopper & Feeder (REV SparkMax) ----
        public static final int HOPPER_FLOOR_NEO = 19;
        public static final int FEEDER_NEO       = 18;

        // ---- Climber (TalonFX) — DISABLED: no climber hardware installed ----
        // public static final int CLIMBER_LEADER   = 20;
        // public static final int CLIMBER_FOLLOWER = 21;

        // Fails fast on startup if any CAN IDs are duplicated in code.
        public static void validateUniqueCanIds() {
            Map<Integer, String> used = new LinkedHashMap<>();
            registerCanId(used, PIGEON, "PIGEON");
            registerCanId(used, FRONT_LEFT_DRIVE, "FRONT_LEFT_DRIVE");
            registerCanId(used, FRONT_LEFT_STEER, "FRONT_LEFT_STEER");
            registerCanId(used, FRONT_LEFT_CANCODER, "FRONT_LEFT_CANCODER");
            registerCanId(used, FRONT_RIGHT_DRIVE, "FRONT_RIGHT_DRIVE");
            registerCanId(used, FRONT_RIGHT_STEER, "FRONT_RIGHT_STEER");
            registerCanId(used, FRONT_RIGHT_CANCODER, "FRONT_RIGHT_CANCODER");
            registerCanId(used, BACK_LEFT_DRIVE, "BACK_LEFT_DRIVE");
            registerCanId(used, BACK_LEFT_STEER, "BACK_LEFT_STEER");
            registerCanId(used, BACK_LEFT_CANCODER, "BACK_LEFT_CANCODER");
            registerCanId(used, BACK_RIGHT_DRIVE, "BACK_RIGHT_DRIVE");
            registerCanId(used, BACK_RIGHT_STEER, "BACK_RIGHT_STEER");
            registerCanId(used, BACK_RIGHT_CANCODER, "BACK_RIGHT_CANCODER");
            registerCanId(used, SHOOTER_LEFT, "SHOOTER_LEFT");
            registerCanId(used, SHOOTER_RIGHT, "SHOOTER_RIGHT");
            registerCanId(used, INTAKE_TILT_NEO, "INTAKE_TILT_NEO");
            registerCanId(used, INTAKE_ROLLER, "INTAKE_ROLLER");
            registerCanId(used, HOPPER_FLOOR_NEO, "HOPPER_FLOOR_NEO");
            registerCanId(used, FEEDER_NEO, "FEEDER_NEO");
            // --- CLIMBER DISABLED ---
            // registerCanId(used, CLIMBER_LEADER, "CLIMBER_LEADER");
            // registerCanId(used, CLIMBER_FOLLOWER, "CLIMBER_FOLLOWER");
        }

        private static void registerCanId(Map<Integer, String> used, int id, String name) {
            String existing = used.putIfAbsent(id, name);
            if (existing != null) {
                throw new IllegalStateException(
                        "Duplicate CAN ID " + id + " for " + existing + " and " + name
                                + ". Fix Constants.CAN map.");
            }
        }
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
    // Hardware: SDS MK4i L2 modules, Falcon 500 motors (TalonFX), CTRE CANcoder, Pigeon 2
    //
    // GEAR RATIOS (from CTRE Tuner X measurement):
    //   Drive:  6.746 : 1  (motor rotations per 1 wheel rotation)
    //   Steer: 21.43  : 1  (motor rotations per 1 wheel rotation — MK4i, NOT MK4)
    //
    // WHEEL: 4-inch diameter billet wheel
    //
    // PID SOURCE: CTRE Tuner X generated TunerConstants for this specific robot
    // =========================================================================
    public static final class Swerve {

        // ---- Physical dimensions ----
        // Module positions from CTRE Tuner X: 11 inches from center in each direction.
        // Track width = 22 inches, wheelbase = 22 inches.
        public static final double TRACK_WIDTH_M = Units.inchesToMeters(22.0);   // 0.5588 m

        // Distance between the FRONT and BACK wheel centers (meters)
        public static final double WHEEL_BASE_M = Units.inchesToMeters(22.0);    // 0.5588 m

        // ---- Wheel & gear math ----
        // 4-inch wheel = 0.1016 m diameter
        public static final double WHEEL_DIAMETER_M     = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M; // ~0.319 m

        // Gear ratios from CTRE Tuner X measurement on this robot.
        // Drive: ~6.75 (SDS MK4i L2 spec = 6.75, Tuner X measured 6.746).
        // Steer: 21.43 (SDS MK4i spec — NOT 12.8 which is the MK4 non-i).
        //   Getting the steer ratio wrong causes the RotorToSensorRatio to be
        //   wrong, which makes the PID loop see the wrong position and oscillate.
        public static final double DRIVE_GEAR_RATIO = 6.746031746031747;  // motor turns per 1 wheel turn
        public static final double STEER_GEAR_RATIO = 21.428571428571427; // motor turns per 1 wheel turn

        // Every 1 rotation of the azimuth results in this many drive motor turns.
        // Used to compensate drive encoder readings during steering motion.
        public static final double COUPLE_RATIO = 3.5714285714285716;

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

        // Phoenix Pro unlocks CTRE's fused sensor mode for steer feedback.
        // Leave false unless this robot's CTRE devices are actually licensed.
        public static final boolean USE_PHOENIX_PRO_FEATURES = false;

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
        // Values from CTRE Tuner X calibration (TunerConstants).
        // Re-run CalibrateCANcodersCommand if wheels have been removed/reinstalled.
        public static final double FL_CANCODER_OFFSET_ROT = -0.358642578125;
        public static final double FR_CANCODER_OFFSET_ROT =  0.199462890625;
        public static final double BL_CANCODER_OFFSET_ROT = -0.026611328125;
        public static final double BR_CANCODER_OFFSET_ROT =  0.365234375;

        //public static final double FL_CANCODER_OFFSET_ROT = -0.368408;
        //public static final double FR_CANCODER_OFFSET_ROT =  0.198730;
        //public static final double BL_CANCODER_OFFSET_ROT = -0.017578;
        //public static final double BR_CANCODER_OFFSET_ROT =  0.371826;

        // ---- CANcoder direction ----
        // Calibration only sets the zero point. If a module runs away while
        // steering, the encoder sign is wrong for that module's mounting.
        public static final boolean FL_CANCODER_CLOCKWISE_POSITIVE = false;
        public static final boolean FR_CANCODER_CLOCKWISE_POSITIVE = false;
        public static final boolean BL_CANCODER_CLOCKWISE_POSITIVE = false;
        public static final boolean BR_CANCODER_CLOCKWISE_POSITIVE = false;

        // ---- Motor inversion ----
        // From CTRE Tuner X TunerConstants: left drive NOT inverted, right drive inverted.
        // ALL steer motors inverted (MK4i bevel gear flips steer direction).
        // Getting these wrong causes the module to "fight itself" and oscillate wildly.
        public static final boolean FL_DRIVE_INVERTED = false;
        public static final boolean FR_DRIVE_INVERTED = true;   // right side inverted
        public static final boolean BL_DRIVE_INVERTED = false;
        public static final boolean BR_DRIVE_INVERTED = true;   // right side inverted

        public static final boolean FL_STEER_INVERTED = true;   // MK4i: all steer inverted
        public static final boolean FR_STEER_INVERTED = true;   // MK4i: all steer inverted
        public static final boolean BL_STEER_INVERTED = true;   // MK4i: all steer inverted
        public static final boolean BR_STEER_INVERTED = true;   // MK4i: all steer inverted

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

        // ---- Steer motor PID (PositionVoltage + CANcoder feedback, Phoenix 6) ----
        // Source: CTRE official Phoenix 6 TunerConstants.java (steerGains).
        // Phoenix Pro's FusedCANcoder tolerates a more aggressive tune than
        // Phoenix 6 non-Pro RemoteCANcoder. Keep the non-Pro path in a
        // plausible final-tune range, just somewhat below the fused default
        // because the remote feedback path has more latency.
        public static final double STEER_kP_PRO = 100.0;       // V/rotation
        public static final double STEER_kP_NON_PRO = 70.0;    // near-tuned starting target for RemoteCANcoder
        public static final double STEER_kD_PRO = 0.5;         // V/(rotation/sec)
        public static final double STEER_kD_NON_PRO = 0.2;     // moderate damping without matching fused-sensor aggressiveness
        public static final double STEER_kS = 0.1;             // Volts
        public static final double STEER_kV = 2.66;            // V/RPS (from Tuner X measurement)

        // ---- Low-speed anti-jitter ----
        // Hold the last steer angle when the requested wheel speed is tiny so
        // the modules don't hunt for a new heading while effectively stopped.
        public static final double ANGLE_HOLD_SPEED_MPS = MAX_TRANSLATION_MPS * 0.01;

        // ---- Validation mode ----
        // Conservative open-loop outputs for one-module-at-a-time sign checks.
        public static final double VALIDATION_DRIVE_DUTY_CYCLE = 0.08;
        public static final double VALIDATION_STEER_DUTY_CYCLE = 0.05;

        // ---- Joystick deadband ----
        // Joystick axes within this range of zero are treated as zero.
        // This prevents the robot from creeping when the stick isn't centered.
        public static final double JOYSTICK_DEADBAND = 0.1;

        // ---- Precision mode ----
        // Hold driver right trigger to scale max speed down for fine positioning.
        // 0.25 = 25% of normal speed when precision mode is active.
        public static final double PRECISION_SPEED_SCALE = 0.25;

        // ---- Current limits (centralized) ----
        public static final int DRIVE_STATOR_CURRENT_LIMIT_A = 60;
        public static final int DRIVE_SUPPLY_CURRENT_LIMIT_A = 40;
        public static final int STEER_STATOR_CURRENT_LIMIT_A = 40;
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

        // ---- Distance-based shot model ----
        // Anchor the curve to the robot's measured close shot:
        // Right Bumper at 4.5 ft scores reliably at 52 RPS.
        //
        // Preserve the existing 60 RPS warmup value at a representative midrange
        // shot, then interpolate between/through those two points until more
        // measured shot data is available.
        public static final double SHOT_ANGLE_DEG        = 60.0;  // TUNE ME
        // Height of the shooter exit above the floor (20 inches).
        public static final double SHOOTER_EXIT_HEIGHT_M = Units.inchesToMeters(16.5);
        // AprilTags are mounted at 44.25 in, but the scoring opening is the upper HUB.
        public static final double HUB_SCORING_HEIGHT_M  = Units.inchesToMeters(104.0);

        // RPS clamps — keep within motor/mechanism limits.
        // Kraken free speed ≈ 100 RPS at 12V; leave headroom for voltage droop.
        public static final double MIN_SHOT_RPS = 60.0;
        public static final double MAX_SHOT_RPS = 90.0;

        // Default warmup speed used during initial spinup before distance is known.
        // Also used as fallback if the distance calculation fails.
        public static final double TARGET_RPS = 60.0;  // TUNE ME
        // Driver override speed when shooting without alignment/vision checks.
        public static final double FALLBACK_RPS = 60.0; // TUNE ME
        public static final double MEASURED_CLOSE_SHOT_DISTANCE_M = Units.feetToMeters(4.5);
        public static final double MIDRANGE_REFERENCE_DISTANCE_M = 2.4;
        public static final double MIDRANGE_REFERENCE_RPS = 68.0;
        public static final double EMPIRICAL_SHOT_SLOPE_RPS_PER_M =
                (MIDRANGE_REFERENCE_RPS - FALLBACK_RPS)
                        / (MIDRANGE_REFERENCE_DISTANCE_M - MEASURED_CLOSE_SHOT_DISTANCE_M);

        // Shooter wheel PID (VelocityVoltage)
        // Kraken X60 at 1:1, 4" wheel, 12V supply:
        //   kV = 12V / 100 RPS ≈ 0.12 (primary feedforward)
        //   kS = static friction overcome. Measured on typical Kraken = ~0.15V.
        //   kP = proportional correction for steady-state error.
        // Source: Typical Kraken X60 shooter values from competition robots.
        public static final double SHOOTER_kS = 0.15;   // Volts (measured typical Kraken)
        public static final double SHOOTER_kV = 0.12;   // 12V / 100 RPS free speed
        public static final double SHOOTER_kP = 0.08;   // V/RPS error (slightly aggressive for fast spinup)

        // ---- Current limits (centralized) ----
        public static final int STATOR_CURRENT_LIMIT_A = 80;
        public static final int SUPPLY_CURRENT_LIMIT_A = 60;

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
        // With a 96:1 reduction, each motor rev = 360/96 = 3.75 degrees.
        public static final double TILT_POS_CONV_DEG = 360.0 / 96.0;

        // Tilt position PID (SparkMax built-in, units are power-per-degree)
        // 96:1 reduction uses MAXMotion profiling + modest PD gains.
        public static final double TILT_kP = 0.08;  // TUNE ME on robot
        public static final double TILT_kD = 0.004; // TUNE ME on robot
        // Closed-loop output caps for position mode (Y-toggle/autonomous setpoints).
        // Positive = tilt up/toward home, negative = tilt down/away from home.
        public static final double TILT_PID_MAX_OUTPUT_UP = 0.30;
        public static final double TILT_PID_MAX_OUTPUT_DOWN = 0.10;
        // Extra-soft profile to slow B-button deploy and reduce arm slam.
        public static final double TILT_MAX_MOTION_CRUISE_VEL_DEG_PER_SEC = 50.0;
        public static final double TILT_MAX_MOTION_ACCEL_DEG_PER_SEC2 = 100.0;
        public static final double TILT_MAX_MOTION_ALLOWED_ERROR_DEG = 1.5;

        // Sign convention for this robot:
        //   stow/home is near 0 deg, deployed intake is negative degrees.
        // Therefore positive power moves toward home (limit switch).
        public static final double HOME_POWER       = 0.20;
        public static final double HOME_TIMEOUT_SEC =  3.0;
        public static final double HOME_SWITCH_DEBOUNCE_SEC = 0.04;

        // Target angle for the intake to be "down" and collecting game pieces
        public static final double INTAKE_DOWN_DEG = -76.0;  // TUNE ME

        // Target angle for the intake to be stowed (up / home position)
        public static final double INTAKE_STOW_DEG = 0.0;

        // Software limits for the tilt arm (in degrees from the homed position).
        // Prevents commanding the arm into the chassis or past its mechanical travel.
        // TUNE ME: Set these to the actual min/max safe travel of YOUR intake arm.
        public static final double TILT_MIN_DEG = -90.0;  // fully deployed
        public static final double TILT_MAX_DEG = 5.0;    // small margin past home
        public static final double TILT_SOFT_LIMIT_HYSTERESIS_DEG = 1.0;

        // Manual right-stick deadband hysteresis for intake tilt.
        // Enter movement above ENGAGE; return to idle below RELEASE.
        public static final double MANUAL_TILT_ENGAGE_DEADBAND = 0.12;
        public static final double MANUAL_TILT_RELEASE_DEADBAND = 0.08;
        // Override threshold for canceling auto tilt toggles (higher than manual
        // engage deadband to reject small stick drift).
        public static final double MANUAL_TILT_TOGGLE_CANCEL_DEADBAND = 0.25;
        // Y-toggle completion criteria.
        public static final double TILT_TOGGLE_AT_TARGET_TOLERANCE_DEG = 2.0;
        public static final double TILT_TOGGLE_SAFETY_TIMEOUT_SEC = 5.0;
        // Manual tilt output caps (as motor percent output).
        // Positive = tilt up/toward home, negative = tilt down/away from home.
        public static final double MANUAL_TILT_MAX_POWER_UP = 0.45;
        public static final double MANUAL_TILT_MAX_POWER_DOWN = 0.25;

        // Current limit to protect the NEO and gearbox during homing stalls
        public static final int TILT_CURRENT_LIMIT_A = 40;

        // Roller current limit (TalonFX / Kraken)
        public static final int ROLLER_STATOR_LIMIT_A = 60;  // prevents jam burnout
        // Positive roller power should pull FUEL inward toward the hopper.
        public static final boolean ROLLER_MOTOR_INVERTED = true;
        // Roller geometry/model for speed matching against robot forward speed.
        public static final double ROLLER_WHEEL_DIAMETER_IN = 3.0;
        public static final double ROLLER_WHEEL_CIRCUMFERENCE_M =
                Math.PI * Units.inchesToMeters(ROLLER_WHEEL_DIAMETER_IN);
        // Kraken X60 free speed at 12V is ~6000 RPM = 100 RPS.
        public static final double ROLLER_FREE_SPEED_RPS = 100.0;
        // Operator intake speed matching: keep a low floor at slow/stopped speed,
        // then ramp toward speed-matched roller surface speed as robot drives forward.
        public static final double ROLLER_MATCH_MIN_POWER = 0.38;
        public static final double ROLLER_MATCH_FORWARD_DEADBAND_MPS = 0.10;
        public static final double ROLLER_MATCH_RATIO = 1.0;
        public static final double ROLLER_MATCH_MAX_POWER = 0.60;

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

        // Wired opposite the feeder motor, so invert the SparkMax and keep
        // positive commanded power meaning "toward feeder" in code.
        public static final boolean MOTOR_INVERTED = true;
    }

    // =========================================================================
    // REV NEO MOTOR DEFAULTS
    // =========================================================================
    public static final class NeoMotors {
        // Generic current limit applied to all SparkMax motors without a specific one
        public static final int DEFAULT_CURRENT_LIMIT_A = 40;
    }

    // =========================================================================
    // PATHPLANNER FALLBACK CONSTANTS
    // Used if deploy/pathplanner/settings.json is missing or malformed.
    // These values are based on common 2026 FRC swerve setups similar to this robot.
    // =========================================================================
    public static final class PathPlanner {
        public static final double ROBOT_MASS_KG = 54.43; // ~120 lb with battery + bumpers
        public static final double ROBOT_MOI = 6.0;
        public static final double WHEEL_COF = 1.0;
        public static final double DRIVE_CURRENT_LIMIT_A = 60.0;
    }

    // =========================================================================
    // VISION CONSTANTS — USB camera (Logitech C920 HD Pro) on roboRIO 2
    //
    // Fallback strategy: AprilTag detect-only in a background thread.
    // No coprocessor required. See docs/RIO_CAMERA_FALLBACK_PLAN.md.
    // =========================================================================
    public static final class Vision {
        // Set false for electrical bring-up when no camera is present.
        public static final boolean ENABLE_VISION = true;

        // ---- USB camera settings (Logitech C920 / C920 HD Pro) ----
        public static final int CAMERA_DEVICE_ID = 0;     // /dev/video0
        // 640x480 gives the AprilTag detector far more pixels to work with than
        // 320x240. We trade some frame rate to extend useful tag range.
        public static final int CAMERA_WIDTH     = 640;
        public static final int CAMERA_HEIGHT    = 480;
        public static final int CAMERA_FPS       = 10;
        public static final int CAMERA_RAW_STREAM_PORT = 1181;
        public static final int CAMERA_OVERLAY_STREAM_PORT = 1182;

        // ---- C920 camera intrinsics (640x480, 4:3 crop) ----
        // Horizontal FOV derived from C920 native 16:9 FOV (70.42°) adjusted
        // for 4:3 crop: sensor crops 1920→1440 wide, so
        // HFOV = 2 * atan((1440/1920) * tan(70.42°/2)) ≈ 55.8°.
        public static final double HORIZONTAL_FOV_DEG = 55.8;
        public static final double VERTICAL_FOV_DEG   = 43.3;   // TUNE ME
        // Focal length in pixels — calibrate once per camera:
        //   Place robot at known distance d from a tag, measure tag pixel height px,
        //   then f = px * d / TAG_HEIGHT_M
        public static final double FOCAL_LENGTH_PIXELS = 600.0;  // CALIBRATE ME
        // Range calibration model applied after pinhole distance estimation:
        //   calibrated = raw * DISTANCE_CALIBRATION_SCALE + DISTANCE_CALIBRATION_OFFSET_M
        // Fit from on-field samples (actual, calc) captured on 2026-03-14:
        // (3.00,3.10), (2.60,2.67), (2.40,2.39), (2.00,1.98), (1.80,1.76), (1.50,1.32)
        // Least-squares affine fit: actual ~= 0.8552 * calc + 0.3324 (RMSE ~0.028 m).
        public static final double DISTANCE_CALIBRATION_SCALE = 0.8552; // TUNE ME
        public static final double DISTANCE_CALIBRATION_OFFSET_M = 0.3324; // TUNE ME

        // AprilTag detector tuning. WPILib's defaults are tuned for speed, not
        // long-range detection on a low-res stream. These values keep more detail.
        public static final int APRILTAG_NUM_THREADS = 2;
        public static final float APRILTAG_QUAD_DECIMATE = 1.0f;
        public static final float APRILTAG_QUAD_SIGMA = 0.0f;
        public static final double APRILTAG_DECODE_SHARPENING = 0.25;
        public static final int APRILTAG_MIN_CLUSTER_PIXELS = 60;
        public static final int APRILTAG_MAX_NUM_MAXIMA = 10;
        public static final double APRILTAG_CRITICAL_ANGLE_RAD = Math.PI / 4.0;
        public static final float APRILTAG_MAX_LINE_FIT_MSE = 10.0f;
        public static final int APRILTAG_MIN_WHITE_BLACK_DIFF = 5;
        public static final boolean APRILTAG_DEGLITCH = false;

        // Standard FRC AprilTag size (6.5 inches outer, 36h11 family)
        public static final double TAG_HEIGHT_M = 0.1651;
        // When only one HUB tag is visible, bias the aim point away from the
        // single tag center toward the likely HUB center (toward image center).
        // Units: pixels of center shift per pixel of detected tag height.
        public static final double SINGLE_TAG_CENTER_BIAS_PX_PER_TAG_HEIGHT = 0.60; // TUNE ME

        // Alignment is "good enough" once yaw error is within this many degrees.
        // Wider than PhotonVision because pixel-based yaw is noisier.
        public static final double YAW_TOLERANCE_DEG = 3.5;
        // After entering the alignment window, allow a slightly wider band
        // before resuming turn corrections so camera jitter does not cause
        // left-right hunting around center.
        public static final double YAW_BREAK_TOLERANCE_DEG = 5.0; // TUNE ME
        // Tolerate brief vision dropouts instead of immediately canceling a shot.
        // Slightly longer to reduce false "target lost" transitions on noisy frames.
        public static final double TARGET_LOSS_TOLERANCE_SEC = 0.75; // TUNE ME
        // Camera health should track frame heartbeat, not whether a tag is visible.
        public static final double CAMERA_HEARTBEAT_TIMEOUT_SEC = 2.0;
        // Feasible vertical angle band for a valid shot solution from the camera.
        public static final double MIN_SHOT_PITCH_DEG = -20.0; // TUNE ME
        // Verify this band on-robot after camera pitch is finalized.
        public static final double MAX_SHOT_PITCH_DEG =  22.0; // TUNE ME

        // Filter raw camera yaw before feeding it into the turn controller.
        // 0 = no filtering, closer to 1 = heavier smoothing.
        public static final double YAW_FILTER_ALPHA = 0.50; // TUNE ME
        // P-only turn control works better on this low-rate vision signal than a
        // noisy derivative term. Keep authority high, then smooth the input.
        public static final double TURN_kP     = 0.14;   // TUNE ME
        public static final double TURN_kD     = 0.0;    // TUNE ME
        public static final double MAX_ROT_CMD = 0.75;   // rad/s cap during alignment

        // ---- Camera mount position ----
        // Used for pitch-based distance estimation.
        // Robot frame convention is +X forward, +Y left, +Z up.
        // Measured mount: 16.5 in high and 9 in to the RIGHT of robot center
        // when facing forward. Robot frame uses +Y to the LEFT, so this is negative.
        // Used to compensate yaw aim so shooter center, not camera center, is aimed.
        public static final double CAMERA_UP_M       = Units.inchesToMeters(16.5);
        public static final double CAMERA_LATERAL_OFFSET_M = -Units.inchesToMeters(9.0);
        public static final double CAMERA_PITCH_RAD  = Math.toRadians(10.5); // tilted up, approximate

        // ---- Alliance-specific HUB tag IDs for targeting ----
        // AlignAndShootCommand should ONLY aim at your alliance's HUB tags.
        // These are the z=1.124m (44.25in) tags mounted on each alliance's HUB.
        // Each HUB has 4 faces with 2 tags per face = 8 tags per HUB.
        // Red HUB center ≈ (11.92, 4.03), Blue HUB center ≈ (4.63, 4.03).
        public static final int[] RED_HUB_TAG_IDS  = {2, 3, 4, 5, 8, 9, 10, 11};
        public static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
    }

    // =========================================================================
    // ALIGN-AND-SHOOT (stationary auto-align then shoot)
    // =========================================================================
    public static final class AlignShoot {
        // AlignAndShoot runs with shooter/feeder spin-up noise present, so use a
        // slightly more damped turn loop than AlignOnly to avoid center hunting.
        public static final double TURN_kP = 0.09; // TUNE ME
        public static final double TURN_kD = 0.0;  // TUNE ME
        public static final double YAW_FILTER_ALPHA = 0.78; // stronger smoothing
        public static final double MAX_AUTO_AIM_OMEGA_RADPS = 0.75;
        // While the target is out of frame, keep sweeping at a controlled rate.
        public static final double SEARCH_OMEGA_RADPS = Math.toRadians(35.0); // TUNE ME
        // After losing a previously seen target, briefly hold still to ride out
        // camera jitter before resuming sweep-based reacquisition.
        public static final double TARGET_LOSS_WAIT_BEFORE_RESEEK_SEC = 0.75; // TUNE ME
        // Larger yaw errors are acquisition problems, not impossible shot geometry.
        public static final double ACQUIRE_YAW_MAX_DEG = 30.0; // TUNE ME

        // Slightly wider hold window prevents unlock/relock chatter near center.
        public static final double YAW_TOLERANCE_DEG = 3.5; // TUNE ME
        public static final double YAW_BREAK_TOLERANCE_DEG = 6.0; // TUNE ME
        // Require break-threshold error to persist briefly before unlocking.
        public static final double LOCK_BREAK_DEBOUNCE_SEC = 0.18; // TUNE ME
        public static final double RPS_TOLERANCE_RPS = 1.5; // TUNE ME
        public static final double SETTLE_TIME_SEC = 0.20; // TUNE ME
        // During continuous hold-to-shoot, tolerate brief target/yaw dropouts
        // before stopping feed and forcing a full re-align.
        public static final double CONTINUOUS_FEED_REACQUIRE_SEC = 0.25; // TUNE ME
    }

    // =========================================================================
    // AUTONOMOUS CONSTANTS
    // =========================================================================
    public static final class Auto {
        // Timeout for AutoShoot named command in auto mode.
        public static final double AUTO_SHOOT_TIMEOUT_SEC = 6.0;
    }

    // =========================================================================
    // CLIMBER CONSTANTS — DISABLED: no climber hardware installed
    // =========================================================================
    // public static final class Climber {
    //     // Software limits — climber cannot go past these positions (in motor rotations)
    //     public static final double REV_SOFT_LIMIT = 0.0;    // fully retracted
    //     public static final double FWD_SOFT_LIMIT = 100.0;  // fully extended
    //
    //     // Target position for Level 1 automatic climb
    //     public static final double LEVEL1_TARGET_ROT = 45.0;  // TUNE ME
    //     public static final double LEVEL1_TOLERANCE_ROT = 1.0;
    //     public static final double LEVEL1_TIMEOUT_SEC = 4.0;
    //
    //     // Position PID for the winch
    //     public static final double CLIMBER_kP = 30.0;  // TUNE ME
    //     public static final double CLIMBER_kD = 0.0;
    //
    //     // Stator current limit for the winch motors (they pull hard!)
    //     public static final int STATOR_LIMIT_A = 60;
    //
    //     // Manual joystick power multiplier for operator control
    //     public static final double MANUAL_POWER_SCALE = 0.6;
    // }

    // =========================================================================
    // ROBOT-LEVEL CONSTANTS
    // =========================================================================
    public static final class RobotConstants {
        // Alert the operator when battery drops below this voltage.
        // FRC brownout threshold is 6.3V; we alert earlier to allow reaction time.
        public static final double BROWNOUT_ALERT_VOLTAGE = 7.0;

        // Maximum duration for an autonomous command before it is forcibly stopped.
        // Prevents a hung auto from running into teleop.
        public static final double AUTO_TIMEOUT_SEC = 15.5;
    }

    // =========================================================================
    // OPERATOR INTERFACE (controller port numbers)
    // =========================================================================
    public static final class OI {
        public static final int DRIVER_PORT   = 0;  // Driver Xbox controller
        public static final int OPERATOR_PORT = 1;  // Operator Xbox controller
    }
}
