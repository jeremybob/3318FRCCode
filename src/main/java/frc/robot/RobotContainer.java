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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.IntakeHomeCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

    // =========================================================================
    // SUBSYSTEMS — created once here, shared with commands
    // =========================================================================
    private final SwerveSubsystem  swerve  = new SwerveSubsystem();
    private final IntakeSubsystem  intake  = new IntakeSubsystem();
    private final HopperSubsystem  hopper  = new HopperSubsystem();
    private final FeederSubsystem  feeder  = new FeederSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    // =========================================================================
    // VISION — one shared camera instance for the whole robot
    // FIXED: Camera is created here, not inside each command.
    // =========================================================================
    private final PhotonCamera camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);

    // =========================================================================
    // CONTROLLERS
    // Port 0 = Driver controller (translation + rotation + game actions)
    // Port 1 = Operator controller (mechanisms: climber, hopper, manual intake)
    // =========================================================================
    private final CommandXboxController driverController   = new CommandXboxController(Constants.OI.DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.OI.OPERATOR_PORT);

    // =========================================================================
    // AUTONOMOUS SELECTOR
    // Shows up in SmartDashboard / Shuffleboard so you can pick an auto before
    // the match without redeploying code.
    // =========================================================================
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public RobotContainer() {
        configurePathPlanner();  // Must be before registerPathPlannerCommands!
        registerPathPlannerCommands();
        configureAutoChooser();
        configureBindings();
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
            // RobotConfig loads from pathplanner/robots/<name>.json in your deploy folder
            // Create this file in the PathPlanner GUI under "Robot Config"
            RobotConfig robotConfig = RobotConfig.fromGUISettings();

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
        } catch (Exception e) {
            // If PathPlanner robot config file is missing, log an error.
            // The robot will still work for teleop, just not for autos.
            System.err.println("[RobotContainer] PATHPLANNER CONFIG FAILED: " + e.getMessage());
            System.err.println("  → Did you create a robot config in the PathPlanner GUI?");
            System.err.println("  → File should be at: deploy/pathplanner/robots/<name>.json");
        }
    }

    // =========================================================================
    // PATHPLANNER NAMED COMMANDS
    //
    // These are actions that can be triggered from inside a PathPlanner auto path.
    // In the PathPlanner GUI, you add these by name as "Events" on a path.
    // =========================================================================
    private void registerPathPlannerCommands() {

        // HomeIntake: re-home the intake at the start of auto (belt-and-suspenders)
        NamedCommands.registerCommand("HomeIntake",
                new IntakeHomeCommand(intake));

        // IntakeFuel: deploy intake and spin rollers to collect a game piece
        NamedCommands.registerCommand("IntakeFuel",
                Commands.sequence(
                        // Only home if we haven't already done it
                        Commands.either(
                                new IntakeHomeCommand(intake),
                                Commands.none(),
                                () -> !intake.isHomed()),
                        // Deploy arm to pickup position
                        Commands.runOnce(() -> intake.setTiltPosition(Constants.Intake.INTAKE_DOWN_DEG), intake),
                        // Spin rollers for 2 seconds to grab a game piece
                        Commands.run(() -> intake.setRollerPower(0.8), intake).withTimeout(2.0),
                        // Stop rollers and leave arm down (ready to stow)
                        Commands.runOnce(() -> intake.setRollerPower(0.0), intake)
                ));

        // AutoShoot: align to target and shoot (3-second timeout for safety)
        NamedCommands.registerCommand("AutoShoot",
                new AlignAndShootCommand(swerve, shooter, feeder, hopper, intake, camera)
                        .withTimeout(3.0));

        // Level1Climb: automatically extends climber to Level 1 height
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
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        // IMPORTANT: The string names below MUST exactly match your .auto file names
        // in deploy/pathplanner/autos/ (case sensitive, no .auto extension needed)
        autoChooser.addOption("Four Note Climb Auto",
                new PathPlannerAuto("FourNoteClimbAuto"));

        autoChooser.addOption("Two Note Auto",
                new PathPlannerAuto("TwoNoteAuto"));

        autoChooser.addOption("Taxi Only",
                new PathPlannerAuto("TaxiOnly"));

        // Publish the chooser so it shows up in SmartDashboard / Shuffleboard
        SmartDashboard.putData("Auto Selector", autoChooser);
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

                    // Scale to actual velocity units
                    double xVelocity = filteredX     * Constants.Swerve.MAX_TRANSLATION_MPS;
                    double yVelocity = filteredY     * Constants.Swerve.MAX_TRANSLATION_MPS;
                    double omega     = filteredOmega * Constants.Swerve.MAX_ROTATION_RADPS;

                    // Drive in field-relative mode (true = joystick "up" always = away from driver)
                    // Hold left bumper to switch to robot-relative mode temporarily
                    boolean fieldRelative = !driverController.leftBumper().getAsBoolean();
                    swerve.drive(xVelocity, yVelocity, omega, fieldRelative);

                }, swerve));

        // Y button: Zero the gyro heading.
        // Use this when field-oriented drive drifts — face the robot away from you and press Y.
        driverController.y().onTrue(
                Commands.runOnce(swerve::zeroHeading, swerve));

        // X button: Home the intake arm (find limit switch position)
        driverController.x().onTrue(
                new IntakeHomeCommand(intake));

        // Right Trigger: Full shoot routine (spin up + align + feed)
        driverController.rightTrigger().onTrue(
                shooter.buildShootRoutine(feeder, hopper, intake, Constants.Shooter.TARGET_RPS));

        // Y button (+ vision): Align and shoot using camera
        // NOTE: Y is used for gyro zero above. If you want both, use a different button.
        // Consider moving align-and-shoot to left trigger or back button.
        driverController.leftTrigger().onTrue(
                new AlignAndShootCommand(swerve, shooter, feeder, hopper, intake, camera));

        // B button: Emergency stop — immediately stops ALL drive motors
        driverController.b().onTrue(Commands.runOnce(swerve::stop, swerve));

        // ---- OPERATOR CONTROLLER BINDINGS ----

        // Right stick Y: Manual climber control
        // Positive = extend (robot goes up), negative = retract
        // Runs continuously while the subsystem is required
        climber.setDefaultCommand(
                Commands.run(() -> {
                    double climbPower = MathUtil.applyDeadband(
                            -operatorController.getRightY(), 0.1);
                    climber.setWinchPower(climbPower);
                }, climber));

        // Left stick Y: Manual hopper control
        // Lets the operator nudge game pieces if they get stuck
        hopper.setDefaultCommand(
                Commands.run(() -> {
                    double hopperPower = MathUtil.applyDeadband(
                            -operatorController.getLeftY(), 0.1);
                    hopper.setPower(hopperPower);
                }, hopper));

        // A button: Automatic Level 1 climb
        operatorController.a().onTrue(
                Commands.runOnce(climber::autoClimbLevel1, climber));

        // B button: Stop climber immediately
        operatorController.b().onTrue(
                Commands.runOnce(climber::stop, climber));

        // Right Bumper: Manual intake roller — spin forward
        operatorController.rightBumper().whileTrue(
                Commands.run(() -> intake.setRollerPower(0.6), intake)
                        .finallyDo(() -> intake.setRollerPower(0)));

        // Left Bumper: Manual intake roller — reverse / eject
        operatorController.leftBumper().whileTrue(
                Commands.run(() -> intake.setRollerPower(-0.4), intake)
                        .finallyDo(() -> intake.setRollerPower(0)));

        // X button: Re-home intake (operator can trigger this too)
        operatorController.x().onTrue(new IntakeHomeCommand(intake));
    }

    // =========================================================================
    // getAutonomousCommand()
    //
    // Called by Robot.java during autonomousInit(). Returns the selected auto.
    // =========================================================================
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // =========================================================================
    // getIntakeHomeCommand()
    //
    // Called by Robot.java during robotInit() to home the intake on startup.
    // =========================================================================
    public Command getIntakeHomeCommand() {
        return new IntakeHomeCommand(intake);
    }
}
