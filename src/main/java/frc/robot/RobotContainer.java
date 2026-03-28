// ============================================================================
// FILE: src/main/java/frc/robot/RobotContainer.java
//
// PURPOSE: The central wiring file for the robot.
//   - Creates all subsystems (hardware objects)
//   - Assigns driver and operator controller bindings
//   - Configures PathPlanner for autonomous
//   - Provides the getAutonomousCommand() method called by Robot.java
//
// KEY FIXES FROM v1:
//   1. Added teleop drive default command (robot was undrivable before!)
//   2. Added AutoBuilder.configure() so PathPlanner autos work
//   3. Added an auto selector on SmartDashboard (multiple auto options)
//   4. Added operator controller with manual subsystem bindings
//   5. Added gyro zero button for the driver
//   6. Camera is created here and shared — no longer re-created per command
//   7. Joystick deadband is applied HERE on raw axis values (0-1 range),
//      not inside the subsystem after scaling
// ============================================================================
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.stream.Collectors;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AlignOnlyCommand;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.CalibrateCANcodersCommand;
import frc.robot.commands.IntakeHomeCommand;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.commands.ValidateSwerveModuleCommand;
import frc.robot.dashboard.DashboardSnapshot;
import frc.robot.dashboard.ReadyToScoreEvaluator;
import frc.robot.dashboard.ReadyToScoreResult;
import frc.robot.dashboard.RobotDashboardService;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveValidationMode;
import frc.robot.util.DriverDriveUtil;
import frc.robot.util.DriverHeadingHoldController;
import edu.wpi.first.math.geometry.Translation2d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;

public class RobotContainer implements RobotRuntimeContainer {

    // =========================================================================
    // VISION — Arducam OV9281 on Raspberry Pi 4, processed by PhotonVision.
    // Tag sightings correct the robot's pose estimate. Alignment to the HUB
    // is calculated from the corrected pose, not from raw tag yaw.
    // =========================================================================
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private int lastVisionTagId = -1;
    private int lastVisionTagCount = 0;

    // =========================================================================
    // SUBSYSTEMS — created once here, shared with commands
    // =========================================================================
    private final SwerveSubsystem  swerve  = new SwerveSubsystem();
    private final IntakeSubsystem  intake  = new IntakeSubsystem();
    private final HopperSubsystem  hopper  = new HopperSubsystem();
    private final FeederSubsystem  feeder  = new FeederSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    // --- CLIMBER DISABLED: no climber hardware installed ---
    // private final ClimberSubsystem climber = new ClimberSubsystem();

    // =========================================================================
    // CONTROLLERS
    // Port 0 = Driver controller (driving only)
    // Port 1 = Operator controller (shooting, intake, hopper, climber)
    // =========================================================================
    private final CommandXboxController driverController   = new CommandXboxController(Constants.OI.DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.OI.OPERATOR_PORT);
    private final DriverHeadingHoldController driverHeadingHoldController = new DriverHeadingHoldController();

    // =========================================================================
    // AUTONOMOUS SELECTOR
    // Shows up in SmartDashboard / Shuffleboard so you can pick an auto before
    // the match without redeploying code.
    // =========================================================================
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final AutoSelectionState autoSelection = new AutoSelectionState();
    private boolean pathPlannerConfigured = false;
    private boolean pathPlannerUsingFallbackConfig = false;
    private Command currentAutoCommand;
    private final RobotDashboardService dashboardService;
    private static final double TRIGGER_ACTIVE_THRESHOLD = 0.20;
    private long controlEventSeq = 0;
    private double controlEventTimestampSec = 0.0;
    private String controlEventMessage = "";
    private String driverCommandSummary = "drive idle";
    private String operatorCommandSummary = "operator idle";
    // --- CLIMBER DISABLED ---
    // private double lastClimberPower = 0.0;
    private double lastManualShooterTargetRps = 0.0;
    private double lastManualHopperPower = 0.0;
    private double lastManualFeederPower = 0.0;
    private double lastIntakeTiltPower = 0.0;
    private double lastDriverRawTurnInput = 0.0;
    private double lastDriverCommandedTranslationMps = 0.0;
    private double lastDriverCommandedOmegaRadPerSec = 0.0;
    private boolean lastDriverFieldRelativeEnabled = true;
    private boolean lastHeadingHoldActive = false;
    private double lastHeadingHoldTargetDeg = Double.NaN;
    private double lastHeadingHoldErrorDeg = Double.NaN;
    private double lastHeadingHoldCorrectionOmegaRadPerSec = 0.0;
    private double lastDriverDriveLoopTimestampSec = Double.NaN;
    private boolean intakeTiltManualAxisActive = false;
    // private boolean lastClimbArmed = false;
    private Command currentSwerveValidationCommand;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public RobotContainer() {
        configurePathPlanner();  // Must be before registerPathPlannerCommands!
        registerPathPlannerCommands();
        configureAutoChooser();
        configureBindings();
        configureCommandEventLogging();

        // Connect to PhotonVision on the Raspberry Pi 4 via NetworkTables.
        // Tag detections are used to correct the robot's pose estimate.
        if (Constants.Vision.ENABLE_VISION) {
            photonCamera = new PhotonCamera(Constants.Vision.PHOTON_CAMERA_NAME);
            try {
                AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                photonPoseEstimator = new PhotonPoseEstimator(
                        fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        Constants.Vision.ROBOT_TO_CAMERA);
                photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            } catch (Exception ex) {
                System.err.println("[Vision] Failed to load field layout: " + ex.getMessage());
                System.err.println("[Vision] Pose estimation DISABLED — alignment will use odometry only.");
            }
        }

        // Intake homing is handled by:
        //   - Auto routines: via the "HomeIntake" named command in PathPlanner events.
        //   - Teleop: scheduled in teleopInit() to avoid conflicting with auto commands.
        //   - IntakeSubsystem.periodic(): auto-homes if the robot boots on the limit switch.

        dashboardService = new RobotDashboardService(new RobotDashboardService.Actions() {
            @Override
            public void zeroHeading() {
                RobotContainer.this.zeroHeading();
            }

            @Override
            public void stopDrive() {
                RobotContainer.this.stopDrive();
            }

            @Override
            public void scheduleIntakeHome() {
                RobotContainer.this.scheduleIntakeHome();
            }

            @Override
            public void scheduleAlignOnly() {
                RobotContainer.this.scheduleAlignOnly();
            }

            @Override
            public void scheduleAlignAndShoot() {
                RobotContainer.this.scheduleAlignAndShoot();
            }

            @Override
            public void scheduleFallbackShoot() {
                RobotContainer.this.scheduleFallbackShoot();
            }

            @Override
            public void scheduleLevel1Climb() {
                RobotContainer.this.scheduleLevel1Climb();
            }

            @Override
            public void scheduleCANcoderCalibration() {
                RobotContainer.this.scheduleCANcoderCalibration();
            }

            @Override
            public void requestSwerveValidation(String moduleName, String modeName) {
                RobotContainer.this.requestSwerveValidation(moduleName, modeName);
            }

            @Override
            public void stopSwerveValidation() {
                RobotContainer.this.stopSwerveValidation();
            }

            @Override
            public void selectAutoByName(String autoName) {
                RobotContainer.this.selectAutoByName(autoName, "CUSTOM DASHBOARD");
            }
        });
    }

    /**
     * Polls PhotonVision for the latest result and feeds any pose estimate
     * into the swerve drive pose estimator to correct odometry drift.
     */
    private void updateVision() {
        if (photonCamera == null || photonPoseEstimator == null) return;

        PhotonPipelineResult result = photonCamera.getLatestResult();

        // Track which tags are visible for telemetry
        if (result.hasTargets()) {
            lastVisionTagId = result.getBestTarget().getFiducialId();
            lastVisionTagCount = result.getTargets().size();
        }

        // Use PhotonPoseEstimator to compute robot field pose from tag sightings
        photonPoseEstimator.setReferencePose(swerve.getPose());
        var poseResult = photonPoseEstimator.update(result);
        if (poseResult.isPresent()) {
            EstimatedRobotPose estimated = poseResult.get();
            swerve.addVisionMeasurement(
                    estimated.estimatedPose.toPose2d(),
                    estimated.timestampSeconds);
        }
    }

    /** Returns the alliance HUB center on the field. */
    static Translation2d getAllianceHubCenter() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Constants.Vision.RED_HUB_CENTER;
        }
        return Constants.Vision.BLUE_HUB_CENTER;
    }

    private void configureCommandEventLogging() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command ->
                logControlEvent("CMD INIT", describeCommand(command)));
        scheduler.onCommandFinish(command ->
                logControlEvent("CMD FINISH", describeCommand(command)));
        scheduler.onCommandInterrupt(command ->
                logControlEvent("CMD INTERRUPT", describeCommand(command)));
    }

    // =========================================================================
    // PATHPLANNER SETUP
    //
    // AutoBuilder.configure() tells PathPlanner:
    //   - How to read the robot's current field position (getPose)
    //   - How to reset that position (resetPose)
    //   - How to read current wheel speeds (getRobotRelativeSpeeds)
    //   - How to command wheel speeds (driveRobotRelative)
    //   - What PID gains to use for path following
    //   - How to detect red alliance (to flip paths automatically)
    // =========================================================================
    private void configurePathPlanner() {
        try {
            // Primary path: load the 2026 PathPlanner GUI settings from deploy.
            // Fallback path: build a local config from robot constants.
            RobotConfig robotConfig = loadPathPlannerRobotConfig();

            AutoBuilder.configure(
                    swerve::getPose,                    // tells PathPlanner where we are
                    swerve::resetPose,                  // lets PathPlanner correct our position
                    swerve::getRobotRelativeSpeeds,     // tells PathPlanner how fast we're going
                    // tells PathPlanner how to drive the robot
                    (speeds, feedforwards) -> swerve.driveRobotRelative(speeds),

                    new PPHolonomicDriveController(
                            // Translation PID: how hard to correct X/Y position errors
                            // kP = 5.0 is a reasonable starting point — TUNE ME
                            new PIDConstants(5.0, 0, 0),
                            // Rotation PID: how hard to correct heading errors
                            new PIDConstants(5.0, 0, 0)
                    ),

                    robotConfig,

                    // Flip paths for red alliance (since PathPlanner paths are drawn for blue)
                    // This lambda runs every loop to check if we're on the red side.
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent()
                                && alliance.get() == DriverStation.Alliance.Red;
                    },

                    swerve   // the swerve subsystem is required during auto paths
            );
            pathPlannerConfigured = true;
            if (pathPlannerUsingFallbackConfig) {
                System.err.println("[RobotContainer] PathPlanner configured with fallback robot config.");
                System.err.println("  → Update deploy/pathplanner/settings.json to match your measured robot.");
            }
        } catch (Exception e) {
            pathPlannerConfigured = false;
            // If AutoBuilder setup fails, log and continue with teleop only.
            // The robot will still work for teleop, just not for autos.
            System.err.println("[RobotContainer] PATHPLANNER CONFIG FAILED: " + e.getMessage());
            System.err.println("  → Check deploy/pathplanner/settings.json (2026 format) and auto files.");
        }
    }

    private RobotConfig loadPathPlannerRobotConfig() {
        try {
            pathPlannerUsingFallbackConfig = false;
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            pathPlannerUsingFallbackConfig = true;
            System.err.println("[RobotContainer] PathPlanner GUI settings unavailable: " + e.getMessage());
            System.err.println("  → Falling back to constants-based swerve config.");
            return buildFallbackRobotConfig();
        }
    }

    private RobotConfig buildFallbackRobotConfig() {
        ModuleConfig moduleConfig = new ModuleConfig(
                Constants.Swerve.WHEEL_DIAMETER_M / 2.0,
                Constants.Swerve.MAX_TRANSLATION_MPS,
                Constants.PathPlanner.WHEEL_COF,
                DCMotor.getFalcon500(1),
                Constants.Swerve.DRIVE_GEAR_RATIO,
                Constants.PathPlanner.DRIVE_CURRENT_LIMIT_A,
                1);

        return new RobotConfig(
                Constants.PathPlanner.ROBOT_MASS_KG,
                Constants.PathPlanner.ROBOT_MOI,
                moduleConfig,
                Constants.Swerve.FRONT_LEFT_LOCATION,
                Constants.Swerve.FRONT_RIGHT_LOCATION,
                Constants.Swerve.BACK_LEFT_LOCATION,
                Constants.Swerve.BACK_RIGHT_LOCATION);
    }

    // =========================================================================
    // PATHPLANNER NAMED COMMANDS
    //
    // These are actions that can be triggered from inside a PathPlanner auto path.
    // In the PathPlanner GUI, you add these by name as "Events" on a path.
    // =========================================================================
    private void registerPathPlannerCommands() {

        // HomeIntake: re-home the intake at the start of auto (belt-and-suspenders)
        NamedCommands.registerCommand(RobotAutoCatalog.NAMED_HOME_INTAKE, buildIntakeHomeCommand());

        // IntakeFuel: deploy intake, spin rollers to pick up FUEL from the ground.
        NamedCommands.registerCommand(RobotAutoCatalog.NAMED_INTAKE_FUEL, buildIntakeGamePieceCommand());
        // Split variants for path-level control:
        //  - IntakeDeployOnly: home-if-needed then move tilt to pickup angle.
        //  - IntakeBalls: run intake rollers only (stall-protected, timed).
        NamedCommands.registerCommand(
                RobotAutoCatalog.NAMED_INTAKE_DEPLOY_ONLY,
                buildAutoIntakeDeployOnlyCommand());
        NamedCommands.registerCommand(
                RobotAutoCatalog.NAMED_INTAKE_BALLS,
                buildAutoIntakeBallsCommand());

        // AutoShoot: align to HUB via vision, then keep feeding/reacquiring
        // continuously until timeout (bounded by Constants.Auto).
        NamedCommands.registerCommand(RobotAutoCatalog.NAMED_AUTO_SHOOT,
                buildAlignAndShootCommand(true).withTimeout(Constants.Auto.AUTO_SHOOT_TIMEOUT_SEC));
        // AutoManualDistanceShoot: no-align shot for PathPlanner that uses the
        // same distance-to-RPS calculation as operator manual distance shooting.
        NamedCommands.registerCommand(
                RobotAutoCatalog.NAMED_AUTO_MANUAL_DISTANCE_SHOOT,
                buildAutoManualDistanceShootCommand());

        // --- CLIMBER DISABLED ---
        // Level1Climb: automatically extends climber to Level 1 height.
        // In REBUILT, Level 1 climb is worth 15 pts in auto (max 2 robots).
        // NamedCommands.registerCommand("Level1Climb",
        //         Commands.runOnce(climber::autoClimbLevel1, climber));
    }

    // =========================================================================
    // AUTO CHOOSER SETUP
    //
    // Adds options to the SmartDashboard dropdown for selecting autonomous mode.
    // Each .auto file name must match a file in deploy/pathplanner/autos/
    // =========================================================================
    private void configureAutoChooser() {
        // Default option (no auto — safe if something breaks)
        Command doNothing = Commands.none();
        registerAutoOption("Do Nothing", doNothing, true);

        if (!pathPlannerConfigured) {
            System.err.println("[RobotContainer] PathPlanner autos disabled: AutoBuilder is not configured.");
            registerAutoOption("Calibrate CANcoders", new CalibrateCANcodersCommand(swerve), false);
            autoChooser.onChange(command -> selectAutoCommand(command, "SMARTDASHBOARD"));
            SmartDashboard.putData("Auto Selector", autoChooser);
            return;
        }

        // IMPORTANT: The string names below MUST exactly match your .auto file names
        // in deploy/pathplanner/autos/ (case sensitive, no .auto extension needed)
        //
        // 2026 REBUILT game: scoring elements are FUEL (5.91" foam balls).
        // Auto strategies for REBUILT:
        //   - Both HUBs are active during autonomous (20 seconds)
        //   - Robots can preload up to 8 FUEL
        //   - Winning auto determines HUB shift order in teleop
        //   - Level 1 climb is available in auto (15 pts, max 2 robots)
        for (RobotAutoCatalog.PathPlannerAutoSpec autoSpec : RobotAutoCatalog.competitionPathPlannerAutos()) {
            addPathPlannerAutoOption(autoSpec);
        }

        // Calibration utility: reads CANcoder offsets and prints to console.
        // Align all wheels forward, select this auto, and enable briefly.
        registerAutoOption("Calibrate CANcoders", new CalibrateCANcodersCommand(swerve), false);

        autoChooser.onChange(command -> selectAutoCommand(command, "SMARTDASHBOARD"));

        // Publish the chooser so it shows up in SmartDashboard / Shuffleboard
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    private void addPathPlannerAutoOption(RobotAutoCatalog.PathPlannerAutoSpec autoSpec) {
        try {
            PathPlannerAuto auto = new PathPlannerAuto(autoSpec.autoFileName());
            registerAutoOption(autoSpec.chooserName(), auto, false, auto.getStartingPose());
        } catch (Exception e) {
            System.err.println("[RobotContainer] Skipping auto '" + autoSpec.autoFileName() + "': " + e.getMessage());
        }
    }

    private void registerAutoOption(String chooserName, Command command, boolean isDefault) {
        registerAutoOption(chooserName, command, isDefault, null);
    }

    private void registerAutoOption(String chooserName, Command command, boolean isDefault, Pose2d startingPose) {
        if (isDefault) {
            autoChooser.setDefaultOption(chooserName, command);
        } else {
            autoChooser.addOption(chooserName, command);
        }
        autoSelection.registerOption(chooserName, command, isDefault, startingPose);
    }

    private void selectAutoByName(String autoName, String source) {
        if (!autoSelection.hasAutoName(autoName)) {
            logControlEvent("Dashboard", "selectAutoByName() rejected: unknown auto '" + autoName + "'");
            return;
        }
        selectAutoCommand(autoSelection.getCommandForName(autoName), source);
    }

    private void selectAutoCommand(Command command, String source) {
        if (command == null || !autoSelection.hasCommand(command)) {
            return;
        }

        Command previousCommand = autoSelection.getSelectedAutoCommand();
        String previousName = autoSelection.getSelectedAutoName();
        String previousSource = autoSelection.getSelectedAutoSource();

        autoSelection.selectCommand(command, source);

        String autoName = autoSelection.getSelectedAutoName();
        boolean changed = command != previousCommand
                || !autoName.equals(previousName)
                || !source.equals(previousSource);
        boolean shouldLog = previousCommand != null && changed;

        if (shouldLog) {
            logControlEvent("Auto", "Selected '" + autoName + "' via " + source);
        }
    }

    // =========================================================================
    // CONTROLLER BINDINGS
    //
    // CURRENT CONTROL MAP (2026 REBUILT — climber disabled):
    //
    //  DRIVER (Port 0):
    //    Left Stick ........... Field-relative translation (forward/strafe)
    //    Right Stick .......... Field-relative rotation
    //    Right Trigger ........ Precision mode (25% speed)
    //    Left Bumper (hold) ... Robot-relative mode override
    //    Y button ............. Zero gyro heading
    //    B button ............. Emergency stop drive
    //    X button (hold) ...... X-Lock (resist pushing)
    //
    //  OPERATOR (Port 1):
    //    Left Stick Y ......... Manual shooter wheel speed
    //    Right Stick Y ........ Manual intake tilt (was climber — changed when climber disabled)
    //    Right Trigger ........ Vision align-and-shoot
    //    Right Bumper ......... Fallback shoot (no vision)
    //    Y button (hold) ...... Manual distance-based shoot (no alignment)
    //    Left Trigger (hold) .. Intake roller forward (stall-protected)
    //    Left Bumper (hold) ... Intake roller reverse / eject
    //    D-pad Up ............. Intake tilt up/stow (home angle)
    //    B button ............. Intake tilt down/deploy (pickup angle)
    //    X button ............. Re-home intake
    //    A button ............. Align-only (vision yaw test, no shot)
    //
    // =========================================================================
    private void configureBindings() {

        // ---- DRIVER CONTROLLER BINDINGS ----
        //
        // DEFAULT COMMAND: Field-relative drive with left stick (translate) + right stick (rotate)
        //
        // This is the most important binding — it's what lets the robot move!
        // "setDefaultCommand" means this runs whenever no other command has the swerve subsystem.
        //
        // MathUtil.applyDeadband() filters out small joystick values near zero.
        // This prevents the robot from slowly creeping when the stick isn't perfectly centered.
        // DEADBAND IS APPLIED HERE on raw axis values (0.0–1.0), NOT after scaling.
        swerve.setDefaultCommand(
                Commands.run(() -> {
                    double rawForward = getDriverForwardInput();
                    double rawLeft = getDriverLeftInput();
                    double rawTurn = getDriverTurnInput();
                    DriverDriveUtil.DriveRequest driveRequest = getDriverDriveRequest(
                            rawForward,
                            rawLeft,
                            rawTurn);
                    double nowSec = Timer.getFPGATimestamp();
                    double currentHeadingDeg = swerve.getPigeonYawDeg();
                    if (!Double.isFinite(lastDriverDriveLoopTimestampSec)
                            || nowSec - lastDriverDriveLoopTimestampSec > Constants.Swerve.HEADING_HOLD_STALE_RESET_SEC) {
                        driverHeadingHoldController.reset(currentHeadingDeg);
                    }
                    DriverHeadingHoldController.Output headingHoldOutput = driverHeadingHoldController.update(
                            currentHeadingDeg,
                            driveRequest.translationSpeedMps(),
                            driveRequest.omegaRadPerSec());
                    swerve.drive(
                            driveRequest.xVelocityMps(),
                            driveRequest.yVelocityMps(),
                            headingHoldOutput.commandedOmegaRadPerSec(),
                            driveRequest.fieldRelative());
                    lastDriverRawTurnInput = rawTurn;
                    lastDriverCommandedTranslationMps = driveRequest.translationSpeedMps();
                    lastDriverCommandedOmegaRadPerSec = headingHoldOutput.commandedOmegaRadPerSec();
                    lastDriverFieldRelativeEnabled = driveRequest.fieldRelative();
                    lastHeadingHoldActive = headingHoldOutput.active();
                    lastHeadingHoldTargetDeg = headingHoldOutput.targetHeadingDeg();
                    lastHeadingHoldErrorDeg = headingHoldOutput.headingErrorDeg();
                    lastHeadingHoldCorrectionOmegaRadPerSec = headingHoldOutput.correctionOmegaRadPerSec();
                    lastDriverDriveLoopTimestampSec = nowSec;
                    publishDriverDriveTelemetry(rawForward, rawLeft, rawTurn, driveRequest, headingHoldOutput);

                    driverCommandSummary = "drive x=" + formatSigned(driveRequest.xVelocityMps())
                            + "m/s y=" + formatSigned(driveRequest.yVelocityMps())
                            + "m/s omega=" + formatSigned(headingHoldOutput.commandedOmegaRadPerSec())
                            + "rad/s field=" + yesNo(driveRequest.fieldRelative())
                            + " precision=" + yesNo(driveRequest.precisionMode())
                            + " hold=" + yesNo(headingHoldOutput.active())
                            + " err=" + formatSigned(headingHoldOutput.headingErrorDeg())
                            + " corr=" + formatSigned(headingHoldOutput.correctionOmegaRadPerSec());
                }, swerve).withName("DriverFieldDriveDefault"));

        // Y button: Zero the gyro heading.
        // Use this when field-oriented drive drifts — face the robot away from you and press Y.
        // No subsystem requirement so it works even during swerve validation.
        driverController.y().onTrue(
                Commands.runOnce(() -> {
                    logControlEvent("Driver:Y", "zeroHeading()");
                    zeroHeading();
                }));

        // B button: Emergency stop — hold to keep drive stopped.
        // Uses whileTrue so the robot stays stopped as long as B is held,
        // preventing the default drive command from resuming immediately.
        driverController.b().whileTrue(
                Commands.run(() -> swerve.stop(), swerve)
                        .beforeStarting(() -> logControlEvent("Driver:B", "stopDrive() hold start"))
                        .finallyDo(() -> logControlEvent("Driver:B", "stopDrive() hold end")));

        // X button: X-Lock — all wheels point inward at 45° to resist being pushed.
        // Hold to maintain the lock; releasing returns to normal drive.
        driverController.x().whileTrue(
                Commands.run(swerve::xLock, swerve)
                        .beforeStarting(() -> logControlEvent("Driver:X", "xLock() hold start"))
                        .finallyDo(() -> logControlEvent("Driver:X", "xLock() hold end")));

        // ---- OPERATOR CONTROLLER BINDINGS ----

        // --- CLIMBER DISABLED: manual climber control commented out ---
        // Right stick Y: Manual climber control
        // Safety gate: climber only moves while BOTH Start + Back are held.
        // climber.setDefaultCommand(
        //         Commands.run(() -> {
        //             boolean climbArmed = operatorController.start().getAsBoolean()
        //                     && operatorController.back().getAsBoolean();
        //             double climbPower = climbArmed
        //                     ? MathUtil.applyDeadband(-operatorController.getRightY(), 0.1)
        //                     : 0.0;
        //             climber.setWinchPower(climbPower);
        //
        //             lastClimbArmed = climbArmed;
        //             lastClimberPower = climbPower;
        //             refreshOperatorCommandSummary();
        //         }, climber).withName("OperatorClimberManualDefault"));

        // Left stick Y: Manual shooter wheel speed.
        // Push forward to command shooter RPS; release or pull back to stop.
        shooter.setDefaultCommand(
                Commands.run(() -> {
                    double manualShooterTargetRps = getOperatorManualShooterTargetRps();
                    if (manualShooterTargetRps > 0.0) {
                        shooter.setShooterVelocity(manualShooterTargetRps);
                    } else {
                        shooter.stop();
                    }

                    lastManualShooterTargetRps = manualShooterTargetRps;
                    refreshOperatorCommandSummary();
                }, shooter).withName("OperatorShooterManualDefault"));

        // Mirror the manual left-stick shooter state so hopper + feeder advance balls
        // during manual shooting, but yield immediately to real shot commands.
        hopper.setDefaultCommand(
                Commands.run(() -> {
                    double manualHopperPower = getManualShooterHopperPower();
                    if (manualHopperPower > 0.0) {
                        hopper.setPower(manualHopperPower);
                    } else {
                        hopper.stop();
                    }

                    lastManualHopperPower = manualHopperPower;
                    refreshOperatorCommandSummary();
                }, hopper).withName("OperatorHopperManualShooterDefault"));

        feeder.setDefaultCommand(
                Commands.run(() -> {
                    double manualFeederPower = getManualShooterFeederPower();
                    if (manualFeederPower > 0.0) {
                        feeder.setPower(manualFeederPower);
                    } else {
                        feeder.stop();
                    }

                    lastManualFeederPower = manualFeederPower;
                    refreshOperatorCommandSummary();
                }, feeder).withName("OperatorFeederManualShooterDefault"));

        // Right stick Y: Manual intake tilt control with deadband to prevent jitter.
        intake.setDefaultCommand(
                Commands.run(() -> {
                    double tiltPower = getOperatorIntakeTiltManualPower();
                    intake.setTiltPowerManual(tiltPower);

                    lastIntakeTiltPower = tiltPower;
                    refreshOperatorCommandSummary();
                }, intake).withName("OperatorIntakeTiltManualDefault"));

        // --- CLIMBER DISABLED: old climb bindings removed ---
        // A button: Align-only test (no shooter/feed motors).
        operatorController.a().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> logControlEvent("Operator:A", "AlignOnly requested")),
                        buildAlignOnlyCommand()));

        // Previous A binding: Automatic Level 1 climb (requires climb gate held)
        // operatorController.a()
        //         .and(operatorController.start())
        //         .and(operatorController.back())
        //         .onTrue(
        //                 Commands.sequence(
        //                         Commands.runOnce(() -> logControlEvent("Operator:A+Start+Back", "Level1 climb requested")),
        //                         buildLevel1ClimbCommand()));
        //
        // // B button: Stop climber immediately
        // operatorController.b().onTrue(
        //         Commands.runOnce(() -> {
        //             logControlEvent("Operator:B", "climber.stop()");
        //             climber.stop();
        //         }, climber));

        // Right Trigger: Vision-required auto-align, then feed continuously while held.
        // Threshold matches TRIGGER_ACTIVE_THRESHOLD so dashboard and actual trigger agree.
        operatorController.rightTrigger(TRIGGER_ACTIVE_THRESHOLD).whileTrue(
                withTeleopShotPriority(
                        buildAlignAndShootCommand(true)
                                .beforeStarting(() -> logControlEvent("Operator:RT", "AlignAndShoot requested"))));

        // Right Bumper: OVERRIDE shot at fallback speed (no alignment/vision required).
        // Spins up once, clears once, then keeps feeding continuously until release.
        operatorController.rightBumper().whileTrue(
                withTeleopShotPriority(
                        buildContinuousFallbackShootCommand()
                                .beforeStarting(() -> logControlEvent("Operator:RB", "Fallback shot requested"))));

        // Y button: Manual shoot with vision-calculated speed (no alignment turn).
        operatorController.y().whileTrue(withTeleopShotPriority(buildManualDistanceShootCommand()));

        // Left Trigger: Manual intake roller — speed-match to robot forward motion
        // with a low-speed floor, plus stall detection/recovery.
        operatorController.leftTrigger(TRIGGER_ACTIVE_THRESHOLD).whileTrue(
                new IntakeRollerCommand(intake, this::getSpeedMatchedIntakeRollerForwardPower)
                        .beforeStarting(() -> logControlEvent("Operator:LT", "IntakeRollerCommand start"))
                        .finallyDo(() -> logControlEvent("Operator:LT", "IntakeRollerCommand end")));

        // Left Bumper: Manual intake roller — reverse / eject
        operatorController.leftBumper().whileTrue(
                Commands.run(() -> intake.setRollerPower(-0.4), intake)
                        .beforeStarting(() -> logControlEvent("Operator:LB", "Manual reverse start"))
                        .finallyDo(() -> {
                            intake.setRollerPower(0);
                            logControlEvent("Operator:LB", "Manual reverse end");
                        }));

        // X button: Re-home intake (operator can trigger this too)
        operatorController.x().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> logControlEvent("Operator:X", "IntakeHome requested")),
                        buildIntakeHomeCommand()));

        // D-pad Up: Intake tilt up/stow (home angle)
        operatorController.povUp().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> logControlEvent("Operator:POV_UP", "Intake tilt stow requested")),
                        buildIntakeTiltMoveCommand(
                                Constants.Intake.INTAKE_STOW_DEG,
                                "IntakeTiltStow")));

        // B button: Intake tilt down/deploy (pickup angle)
        operatorController.b().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> logControlEvent("Operator:B", "Intake tilt deploy requested")),
                        buildIntakeTiltMoveCommand(
                                Constants.Intake.INTAKE_DOWN_DEG,
                                "IntakeTiltDeploy")));
    }

    // =========================================================================
    // getAutonomousCommand()
    //
    // Called by Robot.java during autonomousInit(). Returns the selected auto.
    // =========================================================================
    public Command getAutonomousCommand() {
        return autoSelection.getSelectedAutoCommand();
    }

    public String getSelectedAutoName() {
        return autoSelection.getSelectedAutoName();
    }

    public String getSelectedAutoSource() {
        return autoSelection.getSelectedAutoSource();
    }

    public String[] getAvailableAutoNames() {
        return autoSelection.getAvailableAutoNames();
    }

    public void setCurrentAutoCommand(Command cmd) {
        this.currentAutoCommand = cmd;
    }

    // =========================================================================
    // getIntakeHomeCommand()
    //
    // Exposed for any explicit homing workflows/tests that need the command.
    // =========================================================================
    public Command getIntakeHomeCommand() {
        return buildIntakeHomeCommand();
    }

    public void periodicDashboard() {
        if (Constants.Vision.ENABLE_VISION) {
            updateVision();
        }
        dashboardService.periodic(buildDashboardSnapshot());
    }

    private DashboardSnapshot buildDashboardSnapshot() {
        double nowSec = Timer.getFPGATimestamp();
        var pose = swerve.getPose();
        double shooterTargetRps = Constants.Shooter.TARGET_RPS;
        if (AlignAndShootCommand.isTelemetryCommandActive()) {
            double alignTargetRps = AlignAndShootCommand.getTelemetryTargetRps();
            if (Double.isFinite(alignTargetRps) && alignTargetRps > 0.0) {
                shooterTargetRps = alignTargetRps;
            }
        }
        boolean shooterAtTargetSpeed = shooter.isAtSpeed(shooterTargetRps);
        ReadyToScoreResult ready = ReadyToScoreEvaluator.evaluate(
                new ReadyToScoreEvaluator.Inputs(
                        intake.isHomed(),
                        shooterAtTargetSpeed,
                        AlignAndShootCommand.isTelemetryCommandActive(),
                        AlignAndShootCommand.getTelemetryState(),
                        AlignAndShootCommand.telemetryHasTarget(),
                        AlignAndShootCommand.telemetryGeometryFeasible(),
                        AlignAndShootCommand.telemetryHasShootableTarget(),
                        AlignAndShootCommand.getTelemetryAimErrorDeg(),
                        Constants.AlignShoot.YAW_TOLERANCE_DEG,
                        AlignAndShootCommand.telemetryFeedGateReady()));

        boolean photonConnected = photonCamera != null && photonCamera.isConnected();
        boolean visionActive = swerve.isVisionActive();
        Translation2d hubCenter = getAllianceHubCenter();
        double visionHeadingErrorDeg = swerve.getHeadingErrorDegTo(hubCenter);
        double visionDistanceM = swerve.getDistanceTo(hubCenter);

        double batteryVoltage = RobotController.getBatteryVoltage();
        var canStatus = RobotController.getCANStatus();
        double[] swerveAngles = swerve.getModuleAnglesDeg();
        double[] ccPos = swerve.getCANcoderPositionsRot();
        double[] ccAbsRaw = swerve.getCANcoderAbsoluteRawRot();
        boolean[] ccOk = swerve.getCANcoderOkStates();
        double[] driveTemps = swerve.getDriveTemperaturesC();
        boolean autoRunning = currentAutoCommand != null && currentAutoCommand.isScheduled();
        SwerveSubsystem.ValidationStatus validationStatus = swerve.getValidationStatus();

        return new DashboardSnapshot(
                nowSec,
                getRobotMode(),
                DriverStation.isEnabled(),
                getAllianceName(),
                DriverStation.getMatchTime(),
                pose.getX(),
                pose.getY(),
                swerve.getHeading().getDegrees(),
                swerve.getPigeonYawDeg(),
                swerve.getPigeonPitchDeg(),
                swerve.getPigeonRollDeg(),
                lastDriverRawTurnInput,
                lastDriverCommandedTranslationMps,
                lastDriverCommandedOmegaRadPerSec,
                swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond,
                lastDriverFieldRelativeEnabled,
                lastHeadingHoldActive,
                lastHeadingHoldTargetDeg,
                lastHeadingHoldErrorDeg,
                lastHeadingHoldCorrectionOmegaRadPerSec,
                shooter.getLeftRPS(),
                shooter.getRightRPS(),
                shooterAtTargetSpeed,
                intake.isHomed(),
                intake.getLimitSwitchPressed(),
                intake.getTiltPositionDeg(),
                intake.getRollerCurrentAmps(),
                feeder.getCurrentAmps(),
                hopper.getCurrentAmps(),
                // --- CLIMBER DISABLED: passing defaults ---
                false, // isClimberArmed()
                0.0,   // climber.getWinchPositionRot()
                0.0,   // climber.getCurrentAmps()
                AlignAndShootCommand.getTelemetryState(),
                AlignAndShootCommand.isTelemetryCommandActive(),
                AlignAndShootCommand.telemetryHasTarget(),
                AlignAndShootCommand.telemetryGeometryFeasible(),
                AlignAndShootCommand.telemetryHasShootableTarget(),
                AlignAndShootCommand.getTelemetryHeadingErrorDeg(),
                AlignAndShootCommand.getTelemetryAimErrorDeg(),
                AlignAndShootCommand.getTelemetryDistanceM(),
                AlignAndShootCommand.getTelemetryTargetRps(),
                AlignAndShootCommand.telemetryFeedGateReady(),
                AlignAndShootCommand.getTelemetryLastAbortReason(),
                ready.ready(),
                ready.reason(),
                // 2026 REBUILT: HUB shift activity
                HubActivityTracker.isOurHubActive(),
                HubActivityTracker.secondsUntilNextShiftChange(),
                // System health
                batteryVoltage,
                batteryVoltage < Constants.RobotConstants.BROWNOUT_ALERT_VOLTAGE,
                RobotController.isBrownedOut(),
                // Auto
                getSelectedAutoName(),
                getSelectedAutoSource(),
                getAvailableAutoNames(),
                autoRunning,
                // Expected auto starting pose (blue-side; dashboard flips for red)
                autoSelection.getExpectedAutoStartX(),
                autoSelection.getExpectedAutoStartY(),
                autoSelection.getExpectedAutoStartHeadingDeg(),
                // Match info
                DriverStation.getMatchNumber(),
                DriverStation.getEventName(),
                // Camera (PhotonVision on Pi 4)
                photonConnected,
                photonConnected ? "PHOTON_CONNECTED" : "PHOTON_DISCONNECTED",
                Constants.Vision.PHOTON_CAMERA_NAME,
                // Vision pose estimation
                lastVisionTagId,
                visionActive,
                visionHeadingErrorDeg,
                visionDistanceM,
                lastVisionTagCount,
                nowSec,
                // CAN health
                canStatus.percentBusUtilization,
                canStatus.receiveErrorCount,
                canStatus.transmitErrorCount,
                // Swerve module angles
                swerveAngles[0], swerveAngles[1], swerveAngles[2], swerveAngles[3],
                // CANCoder health
                ccPos[0], ccPos[1], ccPos[2], ccPos[3],
                ccAbsRaw[0], ccAbsRaw[1], ccAbsRaw[2], ccAbsRaw[3],
                ccOk[0], ccOk[1], ccOk[2], ccOk[3],
                // Motor temperatures
                driveTemps[0], driveTemps[1], driveTemps[2], driveTemps[3],
                shooter.getLeftTemperatureC(),
                shooter.getRightTemperatureC(),
                // Controller diagnostics
                formatControlState(activeControls(driverController), driverCommandSummary),
                formatControlState(activeControls(operatorController), operatorCommandSummary),
                controlEventSeq,
                controlEventTimestampSec,
                controlEventMessage,
                validationStatus.active(),
                validationStatus.moduleToken(),
                validationStatus.moduleDisplayName(),
                validationStatus.modeToken(),
                validationStatus.modeDisplayName(),
                validationStatus.drivePercent(),
                validationStatus.steerPercent(),
                validationStatus.startAngleDeg(),
                validationStatus.angleDeltaDeg(),
                validationStatus.startCANcoderRot(),
                validationStatus.cancoderDeltaRot());
    }

    private Command buildAlignAndShootCommand() {
        return buildAlignAndShootCommand(true);
    }

    private Command buildAlignAndShootCommand(boolean continuousFeedUntilInterrupted) {
        if (!Constants.Vision.ENABLE_VISION) {
            return Commands.print("[RobotContainer] AlignAndShoot unavailable: vision is disabled in Constants.");
        }
        return new AlignAndShootCommand(
                swerve,
                shooter,
                feeder,
                hopper,
                intake,
                continuousFeedUntilInterrupted)
                .withName("AlignAndShoot");
    }

    private Command buildAlignOnlyCommand() {
        if (!Constants.Vision.ENABLE_VISION) {
            return Commands.print("[RobotContainer] AlignOnly unavailable: vision is disabled in Constants.");
        }
        return new AlignOnlyCommand(swerve)
                .withName("AlignOnly");
    }

    private Command buildFallbackShootCommand() {
        return shooter.buildShootRoutine(feeder, hopper, intake, Constants.Shooter.FALLBACK_RPS)
                .withName("FallbackShootRoutine");
    }

    private Command buildContinuousFallbackShootCommand() {
        return shooter.buildContinuousShootRoutine(feeder, hopper, intake, Constants.Shooter.FALLBACK_RPS)
                .withName("FallbackShootContinuous");
    }

    private Command buildManualDistanceShootCommand() {
        return Commands.defer(() -> {
            double targetRps = getManualDistanceShotTargetRps();
            return shooter.buildContinuousShootRoutine(feeder, hopper, intake, targetRps)
                    .beforeStarting(() -> logControlEvent(
                            "Operator:Y",
                            "Manual distance shoot requested targetRps=" + formatSigned(targetRps)))
                    .withName("ManualDistanceShootActive");
        }, Set.of(shooter, feeder, hopper, intake)).withName("ManualDistanceShoot");
    }

    private Command buildAutoManualDistanceShootCommand() {
        return Commands.defer(() -> {
            double targetRps = 52.0;
            //double targetRps = getManualDistanceShotTargetRps();
            System.out.println("[AutoManualDistanceShoot] targetRps=" + formatSigned(targetRps));
            return shooter.buildContinuousShootRoutine(feeder, hopper, intake, targetRps)
                    .withName("AutoManualDistanceShootActive");
        }, Set.of(shooter, feeder, hopper, intake))
                .withTimeout(Constants.Auto.AUTO_SHOOT_TIMEOUT_SEC)
                .withName("AutoManualDistanceShoot");
    }

    private Command withTeleopShotPriority(Command command) {
        return command.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private DriverDriveUtil.DriveRequest getDriverDriveRequest() {
        return getDriverDriveRequest(
                getDriverForwardInput(),
                getDriverLeftInput(),
                getDriverTurnInput());
    }

    private DriverDriveUtil.DriveRequest getDriverDriveRequest(
            double rawForward,
            double rawLeft,
            double rawTurn) {
        return DriverDriveUtil.shapeDrive(
                rawForward,
                rawLeft,
                rawTurn,
                isDriverPrecisionMode(),
                isDriverFieldRelative());
    }

    private double getDriverForwardInput() {
        return -driverController.getLeftY();
    }

    private double getDriverLeftInput() {
        return -driverController.getLeftX();
    }

    private double getDriverTurnInput() {
        return -driverController.getRightX();
    }

    private void publishDriverDriveTelemetry(
            double rawForward,
            double rawLeft,
            double rawTurn,
            DriverDriveUtil.DriveRequest driveRequest,
            DriverHeadingHoldController.Output headingHoldOutput) {
        SmartDashboard.putNumber("Drive/RawForwardInput", rawForward);
        SmartDashboard.putNumber("Drive/RawLeftInput", rawLeft);
        SmartDashboard.putNumber("Drive/RawTurnInput", rawTurn);
        SmartDashboard.putNumber("Drive/CommandedTranslationMps", driveRequest.translationSpeedMps());
        SmartDashboard.putNumber("Drive/CommandedOmegaRadPerSec", headingHoldOutput.commandedOmegaRadPerSec());
        SmartDashboard.putBoolean("Drive/FieldRelativeEnabled", driveRequest.fieldRelative());
        SmartDashboard.putBoolean("Drive/HeadingHoldActive", headingHoldOutput.active());
        SmartDashboard.putNumber("Drive/HeadingHoldTargetDeg", headingHoldOutput.targetHeadingDeg());
        SmartDashboard.putNumber("Drive/HeadingHoldErrorDeg", headingHoldOutput.headingErrorDeg());
        SmartDashboard.putNumber(
                "Drive/HeadingHoldCorrectionRadPerSec",
                headingHoldOutput.correctionOmegaRadPerSec());
    }

    private boolean isDriverPrecisionMode() {
        return driverController.rightTrigger(TRIGGER_ACTIVE_THRESHOLD).getAsBoolean();
    }

    private boolean isDriverFieldRelative() {
        return !driverController.leftBumper().getAsBoolean();
    }

    private double getOperatorIntakeTiltManualPower() {
        double rawTiltInput = -operatorController.getRightY();
        double absTiltInput = Math.abs(rawTiltInput);

        // Hysteresis avoids run/stop chatter when the stick hovers near deadband.
        if (intakeTiltManualAxisActive) {
            if (absTiltInput < Constants.Intake.MANUAL_TILT_RELEASE_DEADBAND) {
                intakeTiltManualAxisActive = false;
            }
        } else if (absTiltInput > Constants.Intake.MANUAL_TILT_ENGAGE_DEADBAND) {
            intakeTiltManualAxisActive = true;
        }

        if (!intakeTiltManualAxisActive) {
            return 0.0;
        }
        double manualPower = MathUtil.applyDeadband(rawTiltInput, Constants.Intake.MANUAL_TILT_RELEASE_DEADBAND);
        if (manualPower > 0.0) {
            return Math.min(manualPower, Constants.Intake.MANUAL_TILT_MAX_POWER_UP);
        }
        if (manualPower < 0.0) {
            return Math.max(manualPower, -Constants.Intake.MANUAL_TILT_MAX_POWER_DOWN);
        }
        return 0.0;
    }

    private double getSpeedMatchedIntakeRollerForwardPower() {
        double minPower = Constants.Intake.ROLLER_MATCH_MIN_POWER;
        double forwardMps = Math.max(0.0, swerve.getRobotRelativeSpeeds().vxMetersPerSecond);
        if (!Double.isFinite(forwardMps)
                || forwardMps <= Constants.Intake.ROLLER_MATCH_FORWARD_DEADBAND_MPS) {
            return minPower;
        }

        double targetRollerRps =
                (forwardMps / Constants.Intake.ROLLER_WHEEL_CIRCUMFERENCE_M)
                        * Constants.Intake.ROLLER_MATCH_RATIO;
        double matchedPower = targetRollerRps / Constants.Intake.ROLLER_FREE_SPEED_RPS;
        return MathUtil.clamp(
                Math.max(minPower, matchedPower),
                minPower,
                Constants.Intake.ROLLER_MATCH_MAX_POWER);
    }

    private double getOperatorManualShooterTargetRps() {
        return ShooterSubsystem.manualStickToTargetRps(-operatorController.getLeftY());
    }

    private double getManualShooterHopperPower() {
        return getOperatorManualShooterTargetRps() > 0.0
                ? Constants.Shooter.MANUAL_HOPPER_POWER
                : 0.0;
    }

    private double getManualShooterFeederPower() {
        return getOperatorManualShooterTargetRps() > 0.0
                ? Constants.Shooter.MANUAL_FEEDER_POWER
                : 0.0;
    }

    private double getManualDistanceShotTargetRps() {
        return getAlignAndShootTargetRps();
    }

    private double getAlignAndShootTargetRps() {
        Translation2d hubCenter = getAllianceHubCenter();
        double distanceM = swerve.getDistanceTo(hubCenter);
        if (!Double.isFinite(distanceM) || distanceM <= 0.0) {
            return Constants.Shooter.TARGET_RPS;
        }

        double targetRps = ShooterSubsystem.calculateTargetRPS(distanceM);
        if (!Double.isFinite(targetRps) || targetRps <= 0.0) {
            return Constants.Shooter.TARGET_RPS;
        }
        return targetRps;
    }

    private Command buildIntakeTiltMoveCommand(double targetDegrees, String commandName) {
        final double clampedTargetDeg = Math.max(
                Constants.Intake.TILT_MIN_DEG,
                Math.min(Constants.Intake.TILT_MAX_DEG, targetDegrees));
        Command moveWhenHomed = Commands.sequence(
                Commands.runOnce(() -> intake.setTiltPosition(clampedTargetDeg), intake),
                // Hold subsystem ownership so manual default command cannot
                // override position control until complete/override/timeout.
                Commands.run(() -> { }, intake)
                        .until(() -> {
                            boolean manualOverride =
                                    Math.abs(operatorController.getRightY())
                                            > Constants.Intake.MANUAL_TILT_TOGGLE_CANCEL_DEADBAND;
                            boolean atTarget =
                                    Math.abs(intake.getTiltPositionDeg() - clampedTargetDeg)
                                            <= Constants.Intake.TILT_TOGGLE_AT_TARGET_TOLERANCE_DEG;
                            return manualOverride || atTarget;
                        })
                        .withTimeout(Constants.Intake.TILT_TOGGLE_SAFETY_TIMEOUT_SEC)
        ).withName(commandName + "Active");

        Command notHomed = Commands.runOnce(
                () -> System.out.println("[RobotContainer] " + commandName
                        + " ignored: intake is not homed."),
                intake).withName(commandName + "NotHomed");

        // IMPORTANT: Evaluate homed state at button press time, not startup.
        return Commands.either(moveWhenHomed, notHomed, intake::isHomed)
                .withName(commandName);
    }

    private Command buildIntakeHomeCommand() {
        return new IntakeHomeCommand(intake).withName("IntakeHome");
    }

    private Command buildSwerveValidationCommand(SwerveCorner corner, SwerveValidationMode mode) {
        return new ValidateSwerveModuleCommand(swerve, corner, mode)
                .withName("ValidateSwerve" + corner.token() + mode.token());
    }

    private Command buildAutoIntakeDeployOnlyCommand() {
        return Commands.sequence(
                // Only home if needed.
                Commands.either(
                        buildIntakeHomeCommand(),
                        Commands.none(),
                        () -> !intake.isHomed()),
                // Deploy only if homing succeeded; otherwise report and skip.
                Commands.either(
                        Commands.runOnce(() -> intake.setTiltPosition(Constants.Intake.INTAKE_DOWN_DEG), intake),
                        Commands.runOnce(
                                () -> System.out.println(
                                        "[RobotContainer] AutoIntakeDeployOnly aborted: intake not homed after homing attempt."),
                                intake),
                        intake::isHomed))
                .withName("AutoIntakeDeployOnly");
    }

    private Command buildAutoIntakeBallsCommand() {
        // 4-second timeout allows time to drive over fuel and intake it.
        return new IntakeRollerCommand(intake, this::getSpeedMatchedIntakeRollerForwardPower)
                .withTimeout(4.0)
                .withName("AutoIntakeBalls");
    }

    private Command buildIntakeGamePieceCommand() {
        return Commands.sequence(
                // Only home if we haven't already done it
                Commands.either(
                        buildIntakeHomeCommand(),
                        Commands.none(),
                        () -> !intake.isHomed()),
                // Continue only if homing succeeded; otherwise abort loudly.
                Commands.either(
                        Commands.sequence(
                                // Deploy arm to pickup position
                                Commands.runOnce(() -> intake.setTiltPosition(Constants.Intake.INTAKE_DOWN_DEG), intake),
                                // Spin rollers with stall detection — auto-reverses if jammed.
                                // 4-second timeout allows time to drive over fuel and intake it.
                                new IntakeRollerCommand(intake, this::getSpeedMatchedIntakeRollerForwardPower)
                                        .withTimeout(4.0),
                                // Stop rollers and leave arm down (ready to stow)
                                Commands.runOnce(() -> intake.setRollerPower(0.0), intake)),
                        Commands.runOnce(() -> {
                            intake.setRollerPower(0.0);
                            System.out.println("[RobotContainer] AutoIntakeFuel aborted: intake not homed after homing attempt.");
                        }, intake),
                        intake::isHomed))
                .withName("AutoIntakeFuel");
    }

    private void scheduleAlignAndShoot() {
        if (!DriverStation.isTeleopEnabled()) {
            logControlEvent("Dashboard", "scheduleAlignAndShoot() rejected: teleop required");
            return;
        }
        logControlEvent("Dashboard", "scheduleAlignAndShoot()");
        CommandScheduler.getInstance().schedule(buildAlignAndShootCommand());
    }

    private void scheduleAlignOnly() {
        if (!DriverStation.isTeleopEnabled()) {
            logControlEvent("Dashboard", "scheduleAlignOnly() rejected: teleop required");
            return;
        }
        logControlEvent("Dashboard", "scheduleAlignOnly()");
        CommandScheduler.getInstance().schedule(buildAlignOnlyCommand());
    }

    private void scheduleFallbackShoot() {
        if (!DriverStation.isTeleopEnabled()) {
            logControlEvent("Dashboard", "scheduleFallbackShoot() rejected: teleop required");
            return;
        }
        logControlEvent("Dashboard", "scheduleFallbackShoot()");
        CommandScheduler.getInstance().schedule(buildFallbackShootCommand());
    }

    private void scheduleIntakeHome() {
        if (!DriverStation.isEnabled()
                || !(DriverStation.isTeleopEnabled() || DriverStation.isTestEnabled())) {
            logControlEvent("Dashboard", "scheduleIntakeHome() rejected: enabled teleop/test required");
            return;
        }
        logControlEvent("Dashboard", "scheduleIntakeHome()");
        CommandScheduler.getInstance().schedule(buildIntakeHomeCommand());
    }

    private void scheduleLevel1Climb() {
        // --- CLIMBER DISABLED ---
        // if (!DriverStation.isTeleopEnabled() || !isClimberArmed()) {
        //     logControlEvent("Dashboard", "scheduleLevel1Climb() rejected: teleop+arm gate required");
        //     return;
        // }
        // logControlEvent("Dashboard", "scheduleLevel1Climb()");
        // CommandScheduler.getInstance().schedule(buildLevel1ClimbCommand());
        logControlEvent("Dashboard", "scheduleLevel1Climb() rejected: climber disabled");
    }

    private void scheduleCANcoderCalibration() {
        logControlEvent("Dashboard", "scheduleCANcoderCalibration()");
        CommandScheduler.getInstance().schedule(new CalibrateCANcodersCommand(swerve).withName("CalibrateCANcoders"));
    }

    private void requestSwerveValidation(String moduleName, String modeName) {
        SwerveCorner corner = SwerveCorner.fromToken(moduleName);
        SwerveValidationMode mode = SwerveValidationMode.fromToken(modeName);
        if (corner == null || mode == null) {
            logControlEvent("Dashboard",
                    "requestSwerveValidation() rejected: module=" + moduleName + " mode=" + modeName);
            return;
        }
        if (!DriverStation.isEnabled()
                || !(DriverStation.isTeleopEnabled() || DriverStation.isTestEnabled())) {
            logControlEvent("Dashboard", "requestSwerveValidation() rejected: enabled teleop/test required");
            return;
        }

        stopSwerveValidation();
        currentSwerveValidationCommand = buildSwerveValidationCommand(corner, mode);
        logControlEvent("Dashboard", "requestSwerveValidation(" + corner.token() + ", " + mode.token() + ")");
        CommandScheduler.getInstance().schedule(currentSwerveValidationCommand);
    }

    private void stopSwerveValidation() {
        if (currentSwerveValidationCommand != null && currentSwerveValidationCommand.isScheduled()) {
            currentSwerveValidationCommand.cancel();
        } else {
            swerve.stopValidation();
        }
        currentSwerveValidationCommand = null;
    }

    // --- CLIMBER DISABLED ---
    // private Command buildLevel1ClimbCommand() {
    //     return Commands.run(climber::autoClimbLevel1, climber)
    //             .until(climber::isAtLevel1Target)
    //             .withTimeout(Constants.Climber.LEVEL1_TIMEOUT_SEC)
    //             .finallyDo(climber::stop)
    //             .withName("Level1ClimbAuto");
    // }

    private void zeroHeading() {
        swerve.zeroHeading();
        driverHeadingHoldController.reset(swerve.getPigeonYawDeg());
        lastDriverDriveLoopTimestampSec = Double.NaN;
    }

    private void stopDrive() {
        stopSwerveValidation();
        driverHeadingHoldController.reset(swerve.getPigeonYawDeg());
        lastDriverDriveLoopTimestampSec = Double.NaN;
        swerve.stop();
    }

    // --- CLIMBER DISABLED ---
    // private boolean isClimberArmed() {
    //     return operatorController.start().getAsBoolean() && operatorController.back().getAsBoolean();
    // }

    private void logControlEvent(String source, String detail) {
        controlEventSeq++;
        controlEventTimestampSec = Timer.getFPGATimestamp();
        controlEventMessage = source + " -> " + detail;
    }

    private void refreshOperatorCommandSummary() {
        // --- CLIMBER DISABLED: removed climber fields from summary ---
        operatorCommandSummary = "manualShooterRps=" + formatSigned(lastManualShooterTargetRps)
                + " hopperPower=" + formatSigned(lastManualHopperPower)
                + " feederPower=" + formatSigned(lastManualFeederPower)
                + " tiltPower=" + formatSigned(lastIntakeTiltPower);
        // operatorCommandSummary = "climberPower=" + formatSigned(lastClimberPower)
        //         + " manualShooterRps=" + formatSigned(lastManualShooterTargetRps)
        //         + " climbArmed=" + yesNo(lastClimbArmed);
    }

    private static String formatControlState(String activeControls, String commandSummary) {
        String controls = activeControls == null || activeControls.isBlank() ? "--" : activeControls;
        String command = commandSummary == null || commandSummary.isBlank() ? "--" : commandSummary;
        return "active[" + controls + "] cmd[" + command + "]";
    }

    private static String describeCommand(Command command) {
        String requirements = command.getRequirements().stream()
                .map(req -> req.getClass().getSimpleName())
                .sorted()
                .collect(Collectors.joining(","));
        if (requirements.isBlank()) {
            requirements = "none";
        }
        return command.getName() + " class=" + command.getClass().getSimpleName()
                + " reqs=[" + requirements + "]";
    }

    private static String activeControls(CommandXboxController controller) {
        var hid = controller.getHID();
        List<String> pressed = new ArrayList<>(16);

        if (hid.getAButton()) pressed.add("A");
        if (hid.getBButton()) pressed.add("B");
        if (hid.getXButton()) pressed.add("X");
        if (hid.getYButton()) pressed.add("Y");
        if (hid.getLeftBumperButton()) pressed.add("LB");
        if (hid.getRightBumperButton()) pressed.add("RB");
        if (hid.getBackButton()) pressed.add("BACK");
        if (hid.getStartButton()) pressed.add("START");
        if (hid.getLeftStickButton()) pressed.add("LS");
        if (hid.getRightStickButton()) pressed.add("RS");

        double leftTrigger = hid.getLeftTriggerAxis();
        if (leftTrigger > TRIGGER_ACTIVE_THRESHOLD) {
            pressed.add("LT");
        }
        double rightTrigger = hid.getRightTriggerAxis();
        if (rightTrigger > TRIGGER_ACTIVE_THRESHOLD) {
            pressed.add("RT");
        }

        int pov = hid.getPOV();
        if (pov >= 0) {
            pressed.add("POV " + pov + "deg");
        }

        final double translationAxisThreshold = Constants.Swerve.JOYSTICK_DEADBAND;
        final double rotationAxisThreshold = Constants.Swerve.ROTATION_JOYSTICK_DEADBAND;
        double leftX = hid.getLeftX();
        if (Math.abs(leftX) > translationAxisThreshold) {
            pressed.add("LX " + formatSigned(leftX));
        }
        double leftY = hid.getLeftY();
        if (Math.abs(leftY) > translationAxisThreshold) {
            pressed.add("LY " + formatSigned(leftY));
        }
        double rightX = hid.getRightX();
        if (Math.abs(rightX) > rotationAxisThreshold) {
            pressed.add("RX " + formatSigned(rightX));
        }
        double rightY = hid.getRightY();
        if (Math.abs(rightY) > translationAxisThreshold) {
            pressed.add("RY " + formatSigned(rightY));
        }

        if (pressed.isEmpty()) {
            return "--";
        }
        return String.join(", ", pressed);
    }

    private static String formatSigned(double value) {
        return String.format(Locale.US, "%+.2f", value);
    }

    private static String yesNo(boolean value) {
        return value ? "YES" : "NO";
    }

    static final class AutoSelectionState {
        private final java.util.Map<Command, String> autoCommandNames = new java.util.IdentityHashMap<>();
        private final java.util.Map<String, Command> autoCommandsByName = new java.util.LinkedHashMap<>();
        private final java.util.Map<String, Pose2d> autoStartingPoses = new java.util.LinkedHashMap<>();

        private Command selectedAutoCommand;
        private String selectedAutoName = "Do Nothing";
        private String selectedAutoSource = "DEFAULT";
        private double expectedAutoStartX = Double.NaN;
        private double expectedAutoStartY = Double.NaN;
        private double expectedAutoStartHeadingDeg = Double.NaN;

        void registerOption(String chooserName, Command command, boolean isDefault, Pose2d startingPose) {
            autoCommandNames.put(command, chooserName);
            autoCommandsByName.put(chooserName, command);
            if (startingPose != null) {
                autoStartingPoses.put(chooserName, startingPose);
            }
            if (isDefault || selectedAutoCommand == null) {
                selectCommand(command, "DEFAULT");
            }
        }

        boolean hasAutoName(String autoName) {
            return autoCommandsByName.containsKey(autoName);
        }

        boolean hasCommand(Command command) {
            return autoCommandNames.containsKey(command);
        }

        Command getCommandForName(String autoName) {
            return autoCommandsByName.get(autoName);
        }

        void selectCommand(Command command, String source) {
            if (command == null) {
                return;
            }

            selectedAutoCommand = command;
            selectedAutoName = autoCommandNames.getOrDefault(command, "Unknown");
            selectedAutoSource = source;

            Pose2d startPose = autoStartingPoses.get(selectedAutoName);
            if (startPose != null) {
                expectedAutoStartX = startPose.getX();
                expectedAutoStartY = startPose.getY();
                expectedAutoStartHeadingDeg = startPose.getRotation().getDegrees();
            } else {
                expectedAutoStartX = Double.NaN;
                expectedAutoStartY = Double.NaN;
                expectedAutoStartHeadingDeg = Double.NaN;
            }
        }

        Command getSelectedAutoCommand() {
            return selectedAutoCommand;
        }

        String getSelectedAutoName() {
            return selectedAutoName;
        }

        String getSelectedAutoSource() {
            return selectedAutoSource;
        }

        String[] getAvailableAutoNames() {
            return autoCommandsByName.keySet().toArray(String[]::new);
        }

        double getExpectedAutoStartX() {
            return expectedAutoStartX;
        }

        double getExpectedAutoStartY() {
            return expectedAutoStartY;
        }

        double getExpectedAutoStartHeadingDeg() {
            return expectedAutoStartHeadingDeg;
        }
    }

    private static String getRobotMode() {
        if (DriverStation.isDisabled()) {
            return "DISABLED";
        }
        if (DriverStation.isAutonomousEnabled()) {
            return "AUTONOMOUS";
        }
        if (DriverStation.isTeleopEnabled()) {
            return "TELEOP";
        }
        if (DriverStation.isTestEnabled()) {
            return "TEST";
        }
        return "UNKNOWN";
    }

    private static String getAllianceName() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return "UNKNOWN";
        }
        return alliance.get().name().toUpperCase();
    }
}
