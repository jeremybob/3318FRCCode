# 3318 FRC Student User Guide (2026)

Last updated: 2026-02-21
Source of truth: `src/main/java/frc/robot/Constants.java`, `src/main/java/frc/robot/RobotContainer.java`

## 1. What this guide covers

This guide is for student drivers/programmers who need to:

- Set up software on the laptop
- Configure robot hardware IDs and controller settings
- Update required constants before first drive
- Build/deploy robot code safely
- Use the current two-controller teleop mapping

## 2. Audit summary (what was corrected)

The older setup guide in `docs/FRC_Robot_Setup_Guide.docx` had drift from current code. This guide corrects:

- Teleop controller mapping now matches current code in `RobotContainer`
- Shooter controls moved to operator controller
- Climber now includes a safety arm gate (`Start + Back`)
- Driver is now drive-focused only (no shoot/intake binds)
- Intake tilt conversion reflects current code default (`360.0 / 10.0`)

## 3. Required software setup (programming laptop)

Install these first:

1. WPILib 2026 (includes VS Code + GradleRIO)
2. CTRE Phoenix Tuner X
3. REV Hardware Client
4. PathPlanner app
5. PhotonVision (on coprocessor)

Then confirm vendor JSONs exist in `vendordeps/`:

- `PathplannerLib.json`
- `WPILibNewCommands.json`
- `photonlib.json`
- `REVLib-2026.json`
- `Phoenix6-frc2026-latest.json`

## 4. Robot hardware configuration

### 4.1 CAN IDs (must match `Constants.CAN`)

| Device | CAN ID |
| --- | --- |
| Pigeon 2 | 0 |
| Front Left Drive / Steer / CANcoder | 1 / 2 / 3 |
| Front Right Drive / Steer / CANcoder | 4 / 5 / 6 |
| Back Left Drive / Steer / CANcoder | 7 / 8 / 9 |
| Shooter Left / Right | 10 / 11 |
| Back Right Drive / Steer / CANcoder | 12 / 13 / 14 |
| Intake Tilt (SparkMax) / Roller (TalonFX) | 15 / 16 |
| Hopper Floor (SparkMax) | 17 |
| Feeder (SparkMax) | 18 |
| Climber Leader / Follower | 20 / 21 |

### 4.2 DIO ports

| Signal | Port |
| --- | --- |
| Intake home limit switch | 0 |

### 4.3 REV SparkMax policy

This codebase does **not** call `burnFlash()` or `restoreFactoryDefaults()` in robot code.

You must configure SparkMax devices in REV Hardware Client (one-time persistent setup):

- CAN ID
- Motor type (`Brushless` for NEO)
- Idle mode (`Brake` where expected)
- Current limits
- Intake tilt position conversion factor

## 5. Required constants to measure/configure before competition

Update these in `src/main/java/frc/robot/Constants.java` based on your real robot:

- `Swerve.TRACK_WIDTH_M`
- `Swerve.WHEEL_BASE_M`
- `Swerve.FL_CANCODER_OFFSET_ROT`
- `Swerve.FR_CANCODER_OFFSET_ROT`
- `Swerve.BL_CANCODER_OFFSET_ROT`
- `Swerve.BR_CANCODER_OFFSET_ROT`
- `Intake.TILT_POS_CONV_DEG` (code default is `360.0 / 10.0`; verify your actual ratio)
- `Intake.INTAKE_DOWN_DEG`
- `Vision.CAMERA_NAME`
- `Climber.FWD_SOFT_LIMIT`
- `Climber.LEVEL1_TARGET_ROT`
- Shooter/drive/steer PID values marked `TUNE ME`

## 6. Swerve offset calibration quick steps

1. Put all wheels physically straight forward.
2. In Phoenix Tuner X, read each CANcoder `Absolute Position` (no offset).
3. Negate each reading and write it into `Constants.Swerve.*_CANCODER_OFFSET_ROT`.
4. Deploy and verify wheel angles are near zero when wheels are straight.

## 7. PathPlanner and auto configuration

### 7.1 Robot config file

PathPlanner must have a robot config generated from its GUI. `RobotContainer.configurePathPlanner()` loads this at runtime.

If missing, teleop still works, but PathPlanner autos are disabled.

### 7.2 Auto chooser entries in code

Current auto options in `RobotContainer`:

- `Four Piece Climb Auto` -> `FourNoteClimbAuto` (legacy file name retained)
- `Two Piece Auto` -> `TwoNoteAuto` (legacy file name retained)
- `Taxi Only` -> `TaxiOnly`
- Default: `Do Nothing`

`.auto` files must exist in `src/main/deploy/pathplanner/autos/`.

### 7.3 Named commands for path events

Current named events available in paths:

- `HomeIntake`
- `IntakeGamePiece` (preferred)
- `IntakeFuel` (legacy alias retained for compatibility)
- `AutoShoot`
- `Level1Climb`

## 8. PhotonVision configuration

- Ensure PhotonVision camera pipeline is running on coprocessor.
- Camera name in PhotonVision UI must exactly match `Constants.Vision.CAMERA_NAME`.
- If name is wrong, vision-based shooting/alignment will fail.

## 9. Build and deploy workflow

From repo root:

```bash
./gradlew build
./gradlew deploy
```

If only checking compile:

```bash
./gradlew compileJava
```

On robot boot, intake homing is auto-scheduled in `Robot.robotInit()`.

## 10. Current two-controller teleop mapping (audited)

Controller ports (`Constants.OI`):

- Driver controller: port `0`
- Operator controller: port `1`

### 10.1 Driver (drive-focused)

- Left stick Y/X: field-relative translation
- Right stick X: rotate
- Hold Left Bumper: temporary robot-relative drive
- Y: zero heading
- B: emergency stop swerve drive motors

### 10.2 Operator (mechanisms)

- Right Trigger: vision align + shoot
- Right Bumper: fallback shoot (no vision alignment)
- Left Trigger (hold): intake roller in
- Left Bumper (hold): intake roller reverse/eject
- X: intake re-home
- Left Stick Y: manual hopper jog
- B: climber stop

### 10.3 Climber safety arm gate

Manual climber and auto Level 1 climb are safety-gated:

- Climb is armed only while **both** `Start + Back` are held
- Right Stick Y controls manual climber only when armed
- `A` triggers auto Level 1 climb only when `Start + Back` are also held

## 11. Pre-match setup and config checklist

1. CAN IDs verified in Phoenix Tuner X and REV Hardware Client
2. REV SparkMax one-time configs burned
3. CANcoder offsets measured and entered
4. `Constants.java` tune-me values reviewed and updated
5. PathPlanner robot config present and autos exist
6. PhotonVision camera name matches code
7. Code builds (`./gradlew build`) and deploys
8. Intake homes successfully at boot (`Intake/IsHomed = true`)
9. Driver/operator controllers on correct USB ports and mapped correctly

## 12. Common issues and fixes

- Robot crab-walks or points wrong direction: redo CANcoder offsets
- Field-relative feels wrong: zero heading (`Y`) while robot faces field forward
- Vision shoot aborts: check PhotonVision target visibility and camera name match
- Intake position commands ignored: intake not homed; re-home with operator `X`
- Climber not moving: ensure `Start + Back` gate is held
- Build errors for motor classes: missing vendor dependency JSON/install

## 13. Suggested team workflow

- Keep this file as the living student guide
- Keep tuning values/changes in commit messages
- Re-run controller mapping dry-runs after any teleop binding edits
