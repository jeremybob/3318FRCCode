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
//   4. Added operator controller with manual climber and hopper bindings
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
import java.util.stream.Collectors;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.cscore.VideoSource;
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
import frc.robot.vision.CameraDebugInfo;
import frc.robot.subsystems.swerve.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveValidationMode;
import frc.robot.util.DriverDriveUtil;
import frc.robot.vision.RioVisionThread;
import frc.robot.vision.VisionResult;

public class RobotContainer implements RobotRuntimeContainer {

    // =========================================================================
    // VISION — background thread running AprilTag detection on USB camera
    // Publishes VisionResult via AtomicReference (thread-safe, lock-free).
    // See docs/RIO_CAMERA_FALLBACK_PLAN.md for architecture details.
    // =========================================================================
    private final AtomicReference<VisionResult> visionResult = new AtomicReference<>();
    private final AtomicReference<Double> lastVisionFrameTimestampSec = new AtomicReference<>(Double.NaN);
    private final AtomicReference<CameraDebugInfo> cameraDebugInfo =
            new AtomicReference<>(CameraDebugInfo.defaultState());

    // =========================================================================
    // SUBSYSTEMS — created once here, shared with commands
    // =========================================================================
    private final SwerveSubsystem  swerve  = new SwerveSubsystem(lastVisionFrameTimestampSec);
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
    private double lastHopperPower = 0.0;
    private double lastIntakeTiltPower = 0.0;
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

        // Start the background vision thread (USB camera AprilTag detection).
        if (Constants.Vision.ENABLE_VISION) {
            UsbCamera visionCamera = startVisionCamera();
            if (visionCamera != null) {
                new RioVisionThread(visionCamera, visionResult, lastVisionFrameTimestampSec, cameraDebugInfo).start();
            } else {
                System.err.println("[RobotContainer] Vision camera failed to open. "
                        + "Vision-based shooting is UNAVAILABLE this match.");
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

    private UsbCamera startVisionCamera() {
        try {
            UsbCamera camera = CameraServer.startAutomaticCapture(Constants.Vision.CAMERA_DEVICE_ID);
            camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
            camera.setResolution(Constants.Vision.CAMERA_WIDTH, Constants.Vision.CAMERA_HEIGHT);
            camera.setFPS(Constants.Vision.CAMERA_FPS);
            CameraDebugInfo nextDebug = cameraDebugInfo.get().withStatus("CAPTURE_OPEN");
            UsbCameraInfo info = camera.getInfo();
            if (info != null) {
                nextDebug = nextDebug.withActiveCamera(info.dev, info.name, info.path);
            }
            cameraDebugInfo.set(nextDebug);
            return camera;
        } catch (Exception ex) {
            cameraDebugInfo.set(cameraDebugInfo.get().withError("CAPTURE_OPEN_FAILED", ex.getMessage()));
            System.err.println("[RobotContainer] Failed to open USB camera: " + ex.getMessage());
            ex.printStackTrace();
            return null;
        }
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
        // Replaces the old "IntakeGamePiece" name to match REBUILT terminology.
        NamedCommands.registerCommand(RobotAutoCatalog.NAMED_INTAKE_FUEL, buildIntakeGamePieceCommand());
        // Keep old name registered for backwards compatibility with existing .auto files
        NamedCommands.registerCommand(RobotAutoCatalog.NAMED_INTAKE_GAME_PIECE, buildIntakeGamePieceCommand());

        // AutoShoot: align to HUB via vision and shoot FUEL (timeout in Constants.Auto)
        NamedCommands.registerCommand(RobotAutoCatalog.NAMED_AUTO_SHOOT,
                buildAlignAndShootCommand(false).withTimeout(Constants.Auto.AUTO_SHOOT_TIMEOUT_SEC));

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
    //    Left Stick Y ......... Manual hopper control
    //    Right Stick Y ........ Manual intake tilt (was climber — changed when climber disabled)
    //    Right Trigger ........ Vision align-and-shoot
    //    Right Bumper ......... Fallback shoot (no vision)
    //    Left Trigger (hold) .. Intake roller forward (stall-protected)
    //    Left Bumper (hold) ... Intake roller reverse / eject
    //    Y button ............. Toggle intake tilt (deploy/stow)
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
                    DriverDriveUtil.DriveRequest driveRequest = getDriverDriveRequest();
                    swerve.drive(
                            driveRequest.xVelocityMps(),
                            driveRequest.yVelocityMps(),
                            driveRequest.omegaRadPerSec(),
                            driveRequest.fieldRelative());

                    driverCommandSummary = "drive x=" + formatSigned(driveRequest.xVelocityMps())
                            + "m/s y=" + formatSigned(driveRequest.yVelocityMps())
                            + "m/s omega=" + formatSigned(driveRequest.omegaRadPerSec())
                            + "rad/s field=" + yesNo(driveRequest.fieldRelative())
                            + " precision=" + yesNo(driveRequest.precisionMode());
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

        // Left stick Y: Manual hopper control
        // Lets the operator nudge game pieces if they get stuck
        hopper.setDefaultCommand(
                Commands.run(() -> {
                    double hopperPower = MathUtil.applyDeadband(
                            -operatorController.getLeftY(), 0.1);
                    hopper.setPower(hopperPower);

                    lastHopperPower = hopperPower;
                    refreshOperatorCommandSummary();
                }, hopper).withName("OperatorHopperManualDefault"));

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
                buildAlignAndShootCommand(true)
                        .beforeStarting(() -> logControlEvent("Operator:RT", "AlignAndShoot requested")));

        // Right Bumper: OVERRIDE shot at fallback speed (no alignment/vision required).
        // Spins up once, clears once, then keeps feeding continuously until release.
        operatorController.rightBumper().whileTrue(
                buildContinuousFallbackShootCommand()
                        .beforeStarting(() -> logControlEvent("Operator:RB", "Fallback shot requested")));

        // Left Trigger: Manual intake roller — speed-match to robot forward motion
        // with a low-speed floor, plus stall detection/recovery.
        operatorController.leftTrigger().whileTrue(
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

        // Y button: Toggle intake tilt between down (deployed) and up (stowed)
        operatorController.y().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> logControlEvent("Operator:Y", "Intake tilt toggle requested")),
                        buildIntakeTiltToggleCommand()));
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
        dashboardService.periodic(buildDashboardSnapshot());
    }

    private DashboardSnapshot buildDashboardSnapshot() {
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

        VisionResult latestVision = visionResult.get();
        CameraDebugInfo latestCameraDebug = cameraDebugInfo.get();
        int visionTagId = latestVision != null ? latestVision.tagId() : -1;
        boolean visionHasTarget = latestVision != null;
        double visionYawDeg = latestVision != null ? latestVision.yawDeg() : Double.NaN;
        double visionPitchDeg = latestVision != null ? latestVision.pitchDeg() : Double.NaN;
        double visionBestTagYawDeg = latestVision != null ? latestVision.bestTagYawDeg() : Double.NaN;
        double visionBestTagPitchDeg = latestVision != null ? latestVision.bestTagPitchDeg() : Double.NaN;
        double visionDistanceM = latestVision != null
                ? latestVision.estimateDistanceM(
                        Constants.Vision.TAG_HEIGHT_M,
                        Constants.Vision.FOCAL_LENGTH_PIXELS)
                : Double.NaN;
        double visionTagPixelHeightPx = latestVision != null ? latestVision.tagPixelHeight() : Double.NaN;
        int visionHubTagCount = latestVision != null ? latestVision.hubTagCount() : 0;
        int visionHubFaceCount = latestVision != null ? latestVision.hubFaceCount() : 0;
        double visionHubSpanPx = latestVision != null ? latestVision.hubSpanPx() : Double.NaN;
        double visionTargetTimestampSec = latestVision != null ? latestVision.timestampSec() : Double.NaN;

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
                Timer.getFPGATimestamp(),
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
                AlignAndShootCommand.getTelemetryYawDeg(),
                AlignAndShootCommand.getTelemetryAimErrorDeg(),
                AlignAndShootCommand.getTelemetryLeadYawDeg(),
                AlignAndShootCommand.getTelemetryPitchDeg(),
                AlignAndShootCommand.getTelemetryTargetRps(),
                AlignAndShootCommand.getTelemetryRadialVelocityMps(),
                AlignAndShootCommand.getTelemetryLateralVelocityMps(),
                AlignAndShootCommand.getTelemetryCommandedXVelocityMps(),
                AlignAndShootCommand.getTelemetryCommandedYVelocityMps(),
                AlignAndShootCommand.getTelemetryActiveTranslationCapMps(),
                AlignAndShootCommand.getTelemetryTimeOfFlightSec(),
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
                // Camera
                swerve.isCameraConnected(),
                latestCameraDebug.status(),
                latestCameraDebug.activeDeviceId(),
                latestCameraDebug.activeCameraName(),
                latestCameraDebug.activeCameraPath(),
                latestCameraDebug.enumeratedCameras(),
                latestCameraDebug.lastError(),
                latestCameraDebug.frameCount(),
                latestCameraDebug.lastFrameTimestampSec(),
                visionTagId,
                visionHasTarget,
                visionYawDeg,
                visionPitchDeg,
                visionBestTagYawDeg,
                visionBestTagPitchDeg,
                visionDistanceM,
                visionTagPixelHeightPx,
                visionHubTagCount,
                visionHubFaceCount,
                visionHubSpanPx,
                visionTargetTimestampSec,
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
                visionResult,
                continuousFeedUntilInterrupted)
                .withName("AlignAndShoot");
    }

    private Command buildAlignOnlyCommand() {
        if (!Constants.Vision.ENABLE_VISION) {
            return Commands.print("[RobotContainer] AlignOnly unavailable: vision is disabled in Constants.");
        }
        return new AlignOnlyCommand(swerve, visionResult)
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

    private DriverDriveUtil.DriveRequest getDriverDriveRequest() {
        return DriverDriveUtil.shapeDrive(
                getDriverForwardInput(),
                getDriverLeftInput(),
                getDriverTurnInput(),
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

    private boolean isDriverPrecisionMode() {
        return driverController.rightTrigger().getAsBoolean();
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

    private Command buildIntakeTiltToggleCommand() {
        Command toggleWhenHomed = Commands.startEnd(
                () -> {
                    double positionDeg = intake.getTiltPositionDeg();
                    boolean shouldDeploy =
                            Math.abs(positionDeg - Constants.Intake.INTAKE_STOW_DEG)
                                    < Math.abs(positionDeg - Constants.Intake.INTAKE_DOWN_DEG);
                    intake.setTiltPosition(
                            shouldDeploy ? Constants.Intake.INTAKE_DOWN_DEG
                                         : Constants.Intake.INTAKE_STOW_DEG);
                },
                () -> { /* stop is handled by default command resuming */ },
                intake
        ).until(() -> {
            // End when the operator moves the right stick (manual override)
            double tiltInput = Math.abs(operatorController.getRightY());
            return tiltInput > Constants.Intake.MANUAL_TILT_ENGAGE_DEADBAND;
        }).withTimeout(3.0).withName("IntakeTiltToggleActive");

        Command notHomed = Commands.runOnce(() ->
                System.out.println("[RobotContainer] Intake tilt toggle ignored: intake is not homed."),
                intake).withName("IntakeTiltToggleNotHomed");

        // IMPORTANT: Evaluate homed state at button press time, not startup.
        return Commands.either(toggleWhenHomed, notHomed, intake::isHomed)
                .withName("IntakeTiltToggle");
    }

    private Command buildIntakeHomeCommand() {
        return new IntakeHomeCommand(intake).withName("IntakeHome");
    }

    private Command buildSwerveValidationCommand(SwerveCorner corner, SwerveValidationMode mode) {
        return new ValidateSwerveModuleCommand(swerve, corner, mode)
                .withName("ValidateSwerve" + corner.token() + mode.token());
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
    }

    private void stopDrive() {
        stopSwerveValidation();
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
        operatorCommandSummary = "hopperPower=" + formatSigned(lastHopperPower)
                + " tiltPower=" + formatSigned(lastIntakeTiltPower);
        // operatorCommandSummary = "climberPower=" + formatSigned(lastClimberPower)
        //         + " hopperPower=" + formatSigned(lastHopperPower)
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

        final double axisThreshold = 0.15;
        double leftX = hid.getLeftX();
        if (Math.abs(leftX) > axisThreshold) {
            pressed.add("LX " + formatSigned(leftX));
        }
        double leftY = hid.getLeftY();
        if (Math.abs(leftY) > axisThreshold) {
            pressed.add("LY " + formatSigned(leftY));
        }
        double rightX = hid.getRightX();
        if (Math.abs(rightX) > axisThreshold) {
            pressed.add("RX " + formatSigned(rightX));
        }
        double rightY = hid.getRightY();
        if (Math.abs(rightY) > axisThreshold) {
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
