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

import org.photonvision.PhotonCamera;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
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

import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.CalibrateCANcodersCommand;
import frc.robot.commands.IntakeHomeCommand;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.dashboard.DashboardSnapshot;
import frc.robot.dashboard.ReadyToScoreEvaluator;
import frc.robot.dashboard.ReadyToScoreResult;
import frc.robot.dashboard.RobotDashboardService;
import frc.robot.subsystems.*;

public class RobotContainer {

    // =========================================================================
    // VISION — one shared camera instance for the whole robot
    // FIXED: Camera is created here, not inside each command.
    // The camera must be declared before SwerveSubsystem so it's initialized
    // first (Java fields initialize in declaration order).
    // =========================================================================
    private final PhotonCamera camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);

    // =========================================================================
    // SUBSYSTEMS — created once here, shared with commands
    // =========================================================================
    private final SwerveSubsystem  swerve  = new SwerveSubsystem(camera);
    private final IntakeSubsystem  intake  = new IntakeSubsystem();
    private final HopperSubsystem  hopper  = new HopperSubsystem();
    private final FeederSubsystem  feeder  = new FeederSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

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
    private final Map<Command, String> autoCommandNames = new IdentityHashMap<>();
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
    private double lastClimberPower = 0.0;
    private double lastHopperPower = 0.0;
    private boolean lastClimbArmed = false;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public RobotContainer() {
        configurePathPlanner();  // Must be before registerPathPlannerCommands!
        registerPathPlannerCommands();
        configureAutoChooser();
        configureBindings();
        configureCommandEventLogging();

        // Schedule intake homing at startup so the arm finds its zero position.
        // This runs once when the robot first enables (CommandScheduler won't
        // execute commands while disabled, so the actual homing happens when
        // the robot is first enabled in teleop or auto).
        CommandScheduler.getInstance().schedule(buildIntakeHomeCommand());

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
        });
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
        NamedCommands.registerCommand("HomeIntake", buildIntakeHomeCommand());

        // IntakeFuel: deploy intake, spin rollers to pick up FUEL from the ground.
        // Replaces the old "IntakeGamePiece" name to match REBUILT terminology.
        NamedCommands.registerCommand("IntakeFuel", buildIntakeGamePieceCommand());
        // Keep old name registered for backwards compatibility with existing .auto files
        NamedCommands.registerCommand("IntakeGamePiece", buildIntakeGamePieceCommand());

        // AutoShoot: align to HUB via vision and shoot FUEL (timeout in Constants.Auto)
        NamedCommands.registerCommand("AutoShoot",
                buildAlignAndShootCommand().withTimeout(Constants.Auto.AUTO_SHOOT_TIMEOUT_SEC));

        // Level1Climb: automatically extends climber to Level 1 height.
        // In REBUILT, Level 1 climb is worth 15 pts in auto (max 2 robots).
        NamedCommands.registerCommand("Level1Climb",
                Commands.runOnce(climber::autoClimbLevel1, climber));
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
        autoChooser.setDefaultOption("Do Nothing", doNothing);
        autoCommandNames.put(doNothing, "Do Nothing");

        if (!pathPlannerConfigured) {
            System.err.println("[RobotContainer] PathPlanner autos disabled: AutoBuilder is not configured.");
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
        addPathPlannerAutoOption("Eight Fuel Climb Auto", "EightFuelClimbAuto");
        addPathPlannerAutoOption("Taxi Only", "TaxiOnly");

        // Calibration utility: reads CANcoder offsets and prints to console.
        // Align all wheels forward, select this auto, and enable briefly.
        Command calibrate = new CalibrateCANcodersCommand();
        autoChooser.addOption("Calibrate CANcoders", calibrate);
        autoCommandNames.put(calibrate, "Calibrate CANcoders");

        // Publish the chooser so it shows up in SmartDashboard / Shuffleboard
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    private void addPathPlannerAutoOption(String chooserName, String autoFileName) {
        try {
            Command cmd = new PathPlannerAuto(autoFileName);
            autoChooser.addOption(chooserName, cmd);
            autoCommandNames.put(cmd, chooserName);
        } catch (Exception e) {
            System.err.println("[RobotContainer] Skipping auto '" + autoFileName + "': " + e.getMessage());
        }
    }

    // =========================================================================
    // CONTROLLER BINDINGS
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
                    // Get raw joystick values (-1.0 to 1.0)
                    double rawLeftY  = -driverController.getLeftY();   // forward = positive
                    double rawLeftX  = -driverController.getLeftX();   // left = positive
                    double rawRightX = -driverController.getRightX();  // CCW = positive

                    // Apply deadband to raw axis BEFORE scaling to m/s or rad/s
                    double filteredX     = MathUtil.applyDeadband(rawLeftY,  Constants.Swerve.JOYSTICK_DEADBAND);
                    double filteredY     = MathUtil.applyDeadband(rawLeftX,  Constants.Swerve.JOYSTICK_DEADBAND);
                    double filteredOmega = MathUtil.applyDeadband(rawRightX, Constants.Swerve.JOYSTICK_DEADBAND);

                    // Precision mode: hold right trigger to slow down for fine positioning
                    boolean precisionMode = driverController.rightTrigger().getAsBoolean();
                    double speedScale = precisionMode
                            ? Constants.Swerve.PRECISION_SPEED_SCALE
                            : 1.0;

                    // Scale to actual velocity units (with precision mode applied)
                    double xVelocity = filteredX     * Constants.Swerve.MAX_TRANSLATION_MPS * speedScale;
                    double yVelocity = filteredY     * Constants.Swerve.MAX_TRANSLATION_MPS * speedScale;
                    double omega     = filteredOmega * Constants.Swerve.MAX_ROTATION_RADPS  * speedScale;

                    // Drive in field-relative mode (true = joystick "up" always = away from driver)
                    // Hold left bumper to switch to robot-relative mode temporarily
                    boolean fieldRelative = !driverController.leftBumper().getAsBoolean();
                    swerve.drive(xVelocity, yVelocity, omega, fieldRelative);

                    driverCommandSummary = "drive x=" + formatSigned(xVelocity)
                            + "m/s y=" + formatSigned(yVelocity)
                            + "m/s omega=" + formatSigned(omega)
                            + "rad/s field=" + yesNo(fieldRelative)
                            + " precision=" + yesNo(precisionMode);
                }, swerve).withName("DriverFieldDriveDefault"));

        // Y button: Zero the gyro heading.
        // Use this when field-oriented drive drifts — face the robot away from you and press Y.
        driverController.y().onTrue(
                Commands.runOnce(() -> {
                    logControlEvent("Driver:Y", "zeroHeading()");
                    zeroHeading();
                }, swerve));

        // B button: Emergency stop — immediately stops ALL drive motors
        driverController.b().onTrue(Commands.runOnce(() -> {
            logControlEvent("Driver:B", "stopDrive()");
            stopDrive();
        }, swerve));

        // X button: X-Lock — all wheels point inward at 45° to resist being pushed.
        // Hold to maintain the lock; releasing returns to normal drive.
        driverController.x().whileTrue(
                Commands.run(swerve::xLock, swerve)
                        .beforeStarting(() -> logControlEvent("Driver:X", "xLock() hold start"))
                        .finallyDo(() -> logControlEvent("Driver:X", "xLock() hold end")));

        // ---- OPERATOR CONTROLLER BINDINGS ----

        // Right stick Y: Manual climber control
        // Safety gate: climber only moves while BOTH Start + Back are held.
        climber.setDefaultCommand(
                Commands.run(() -> {
                    boolean climbArmed = operatorController.start().getAsBoolean()
                            && operatorController.back().getAsBoolean();
                    double climbPower = climbArmed
                            ? MathUtil.applyDeadband(-operatorController.getRightY(), 0.1)
                            : 0.0;
                    climber.setWinchPower(climbPower);

                    lastClimbArmed = climbArmed;
                    lastClimberPower = climbPower;
                    refreshOperatorCommandSummary();
                }, climber).withName("OperatorClimberManualDefault"));

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

        // A button: Automatic Level 1 climb (requires climb gate held)
        // Uses a sustained command so the default manual climber command does not
        // immediately overwrite the position request.
        operatorController.a()
                .and(operatorController.start())
                .and(operatorController.back())
                .onTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> logControlEvent("Operator:A+Start+Back", "Level1 climb requested")),
                                buildLevel1ClimbCommand()));

        // B button: Stop climber immediately
        operatorController.b().onTrue(
                Commands.runOnce(() -> {
                    logControlEvent("Operator:B", "climber.stop()");
                    climber.stop();
                }, climber));

        // Right Trigger: Vision-required align-and-shoot (operator controls scoring).
        operatorController.rightTrigger().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> logControlEvent("Operator:RT", "AlignAndShoot requested")),
                        buildAlignAndShootCommand()));

        // Right Bumper: OVERRIDE shot at fallback speed (no alignment/vision required).
        operatorController.rightBumper().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> logControlEvent("Operator:RB", "Fallback shot requested")),
                        buildFallbackShootCommand()));

        // Left Trigger: Manual intake roller — spin forward with stall detection.
        // If the roller jams, it automatically reverses and retries (up to 3 times).
        operatorController.leftTrigger().whileTrue(
                new IntakeRollerCommand(intake, 0.6)
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
        return autoChooser.getSelected();
    }

    public String getSelectedAutoName() {
        Command selected = autoChooser.getSelected();
        return autoCommandNames.getOrDefault(selected, "Unknown");
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

        // 2026 REBUILT: Track HUB activity shifts for operator awareness.
        // Publishes to SmartDashboard so operators know when to shoot vs. collect.
        HubActivityTracker.isOurHubActive();
    }

    private DashboardSnapshot buildDashboardSnapshot() {
        var pose = swerve.getPose();
        ReadyToScoreResult ready = ReadyToScoreEvaluator.evaluate(
                new ReadyToScoreEvaluator.Inputs(
                        intake.isHomed(),
                        shooter.isAtSpeed(Constants.Shooter.TARGET_RPS),
                        AlignAndShootCommand.isTelemetryCommandActive(),
                        AlignAndShootCommand.getTelemetryState(),
                        AlignAndShootCommand.telemetryHasTarget(),
                        AlignAndShootCommand.telemetryGeometryFeasible(),
                        AlignAndShootCommand.telemetryHasShootableTarget(),
                        AlignAndShootCommand.getTelemetryYawDeg(),
                        Constants.Vision.YAW_TOLERANCE_DEG));

        double batteryVoltage = RobotController.getBatteryVoltage();
        var canStatus = RobotController.getCANStatus();
        double[] swerveAngles = swerve.getModuleAnglesDeg();
        double[] driveTemps = swerve.getDriveTemperaturesC();
        boolean autoRunning = currentAutoCommand != null && currentAutoCommand.isScheduled();

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
                shooter.isAtSpeed(Constants.Shooter.TARGET_RPS),
                intake.isHomed(),
                intake.getLimitSwitchPressed(),
                intake.getTiltPositionDeg(),
                intake.getRollerCurrentAmps(),
                feeder.getCurrentAmps(),
                hopper.getCurrentAmps(),
                isClimberArmed(),
                climber.getWinchPositionRot(),
                climber.getCurrentAmps(),
                AlignAndShootCommand.getTelemetryState(),
                AlignAndShootCommand.isTelemetryCommandActive(),
                AlignAndShootCommand.telemetryHasTarget(),
                AlignAndShootCommand.telemetryGeometryFeasible(),
                AlignAndShootCommand.telemetryHasShootableTarget(),
                AlignAndShootCommand.getTelemetryYawDeg(),
                AlignAndShootCommand.getTelemetryPitchDeg(),
                AlignAndShootCommand.getTelemetryLastAbortReason(),
                ready.ready(),
                ready.reason(),
                // System health
                batteryVoltage,
                batteryVoltage < 7.0,
                RobotController.isBrownedOut(),
                // Auto
                getSelectedAutoName(),
                autoRunning,
                // Match info
                DriverStation.getMatchNumber(),
                DriverStation.getEventName(),
                // Camera
                swerve.isCameraConnected(),
                // CAN health
                canStatus.percentBusUtilization,
                canStatus.receiveErrorCount,
                canStatus.transmitErrorCount,
                // Swerve module angles
                swerveAngles[0], swerveAngles[1], swerveAngles[2], swerveAngles[3],
                // Motor temperatures
                driveTemps[0], driveTemps[1], driveTemps[2], driveTemps[3],
                shooter.getLeftTemperatureC(),
                shooter.getRightTemperatureC(),
                // Controller diagnostics
                formatControlState(activeControls(driverController), driverCommandSummary),
                formatControlState(activeControls(operatorController), operatorCommandSummary),
                controlEventSeq,
                controlEventTimestampSec,
                controlEventMessage);
    }

    private Command buildAlignAndShootCommand() {
        if (!Constants.Vision.ENABLE_PHOTON) {
            return Commands.print("[RobotContainer] AlignAndShoot unavailable: vision is disabled in Constants.");
        }
        return new AlignAndShootCommand(swerve, shooter, feeder, hopper, intake, camera)
                .withName("AlignAndShoot");
    }

    private Command buildFallbackShootCommand() {
        return shooter.buildShootRoutine(feeder, hopper, intake, Constants.Shooter.FALLBACK_RPS)
                .withName("FallbackShootRoutine");
    }

    private Command buildIntakeTiltToggleCommand() {
        return Commands.runOnce(() -> {
            if (!intake.isHomed()) {
                System.out.println("[RobotContainer] Intake tilt toggle ignored: intake is not homed.");
                return;
            }

            double midpoint = (Constants.Intake.INTAKE_DOWN_DEG + Constants.Intake.INTAKE_STOW_DEG) / 2.0;
            boolean shouldDeploy = intake.getTiltPositionDeg() < midpoint;
            intake.setTiltPosition(
                    shouldDeploy ? Constants.Intake.INTAKE_DOWN_DEG
                                 : Constants.Intake.INTAKE_STOW_DEG);
        }, intake).withName("IntakeTiltToggle");
    }

    private Command buildIntakeHomeCommand() {
        return new IntakeHomeCommand(intake).withName("IntakeHome");
    }

    private Command buildIntakeGamePieceCommand() {
        return Commands.sequence(
                // Only home if we haven't already done it
                Commands.either(
                        buildIntakeHomeCommand(),
                        Commands.none(),
                        () -> !intake.isHomed()),
                // Deploy arm to pickup position
                Commands.runOnce(() -> intake.setTiltPosition(Constants.Intake.INTAKE_DOWN_DEG), intake),
                // Spin rollers with stall detection — auto-reverses if jammed
                new IntakeRollerCommand(intake, 0.8).withTimeout(2.0),
                // Stop rollers and leave arm down (ready to stow)
                Commands.runOnce(() -> intake.setRollerPower(0.0), intake))
                .withName("AutoIntakeFuel");
    }

    private void scheduleAlignAndShoot() {
        logControlEvent("Dashboard", "scheduleAlignAndShoot()");
        CommandScheduler.getInstance().schedule(buildAlignAndShootCommand());
    }

    private void scheduleFallbackShoot() {
        logControlEvent("Dashboard", "scheduleFallbackShoot()");
        CommandScheduler.getInstance().schedule(buildFallbackShootCommand());
    }

    private void scheduleIntakeHome() {
        logControlEvent("Dashboard", "scheduleIntakeHome()");
        CommandScheduler.getInstance().schedule(buildIntakeHomeCommand());
    }

    private void scheduleLevel1Climb() {
        logControlEvent("Dashboard", "scheduleLevel1Climb()");
        CommandScheduler.getInstance().schedule(buildLevel1ClimbCommand());
    }

    private Command buildLevel1ClimbCommand() {
        return Commands.run(climber::autoClimbLevel1, climber)
                .until(climber::isAtLevel1Target)
                .withTimeout(Constants.Climber.LEVEL1_TIMEOUT_SEC)
                .finallyDo(climber::stop)
                .withName("Level1ClimbAuto");
    }

    private void zeroHeading() {
        swerve.zeroHeading();
    }

    private void stopDrive() {
        swerve.stop();
    }

    private boolean isClimberArmed() {
        return operatorController.start().getAsBoolean() && operatorController.back().getAsBoolean();
    }

    private void logControlEvent(String source, String detail) {
        controlEventSeq++;
        controlEventTimestampSec = Timer.getFPGATimestamp();
        controlEventMessage = source + " -> " + detail;
    }

    private void refreshOperatorCommandSummary() {
        operatorCommandSummary = "climberPower=" + formatSigned(lastClimberPower)
                + " hopperPower=" + formatSigned(lastHopperPower)
                + " climbArmed=" + yesNo(lastClimbArmed);
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
