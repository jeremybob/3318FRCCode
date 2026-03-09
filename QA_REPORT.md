# QA Report: Team 3318 FRC 2026 (REBUILT) Robot Code
**Date:** 2026-03-09
**Reviewer:** Claude QA
**Scope:** Full codebase review (robot code, commands, subsystems, vision, custom dashboard)

---

## Summary

The codebase is well-structured and extensively documented. The custom Swing dashboard is impressively comprehensive. 24 issues identified below, ranging from critical bugs to code quality concerns, excluding tuning parameters as requested.

---

## CRITICAL BUGS

### 1. `CalibrateCANcodersCommand` leaks NetworkTables publishers on every execution
**File:** `src/main/java/frc/robot/commands/CalibrateCANcodersCommand.java:101-104`
**Issue:** The `publishCalibrationValue()` method creates a `DoublePublisher` with try-with-resources, which immediately closes it after `set()`. In NetworkTables 4, closing a publisher can remove the topic or make the value non-persistent. More critically, each invocation creates new publishers for the same topic keys, and the behavior of `publish()` on an already-published topic is problematic. The published values may not actually reach the dashboard because the publisher is closed before NT has flushed the value to subscribers.
**Impact:** Calibration values may not reliably appear on the custom dashboard. The SmartDashboard values (lines 70-77) will work, but the Dashboard contract table values may be transient.

### 2. `AlignAndShootCommand` does not filter by alliance HUB tags
**File:** `src/main/java/frc/robot/commands/AlignAndShootCommand.java`
**Issue:** The command reads `visionRef.get()` and uses whatever tag the `RioVisionThread` publishes as "best". The `RioVisionThread` correctly filters for alliance HUB tags (line 179-200), but when alliance is unknown (`getAllianceHubTagIds()` returns null at line 258), it accepts **any** HUB tag from either alliance. During early match startup or practice mode, the robot could aim at the opponent's HUB and score for them.
**Impact:** In practice mode or if alliance data arrives late, the robot may target the wrong HUB.

### 3. `IntakeSubsystem.setTiltPower()` software limits only active when homed
**File:** `src/main/java/frc/robot/subsystems/IntakeSubsystem.java:167-183`
**Issue:** The upper/lower software limits (`TILT_MIN_DEG` / `TILT_MAX_DEG`) are only enforced when `isHomed == true` (line 173). Before homing, the operator can command full tilt power in either direction with no software limits, only the limit switch protects the negative direction. There is no protection against over-extending the arm (positive direction) before homing.
**Impact:** If the operator moves the intake tilt via the right stick before homing completes, the arm could be driven past its mechanical travel in the positive direction.

### 4. `SwerveSubsystem` accepts `VisionResult` in constructor but never uses it
**File:** `src/main/java/frc/robot/subsystems/SwerveSubsystem.java:149-151`
**Issue:** The constructor accepts `AtomicReference<VisionResult> visionResult` but this parameter is never stored or used anywhere in the class. The import for `VisionResult` exists but the field is unused.
**Impact:** No vision pose correction is applied to odometry. This is documented as intentional ("USB camera fallback mode"), but the constructor signature is misleading. If someone adds vision pose fusion later, they might not realize the parameter is silently dropped.

---

## SIGNIFICANT BUGS

### 5. `DashboardNtClient` creates duplicate pub/sub on the same topic keys
**File:** `src/dashboard/java/frc/dashboard/DashboardNtClient.java:227-250`
**Issue:** For each command (e.g., `zero_heading_seq`), both a publisher AND a subscriber are created for the same topic key. Lines 227-238 create publishers (e.g., `zeroHeadingPub`), and lines 240-250 create subscribers on the **same** topic keys (e.g., `zeroHeadingSeqSub`). This is done so the dashboard can read the current sequence number before incrementing. However, creating a publisher and subscriber on the same NT topic within the same client means the publisher's writes are reflected back to the subscriber, which could cause the `nextCommandSequence()` logic to see its own writes and skip a valid command.
**Impact:** Race condition where a command button press may occasionally be missed if the publisher write propagates to the subscriber before the robot processes it.

### 6. `AlignAndShootCommand` ALIGN state can timeout with no target but no timeout path exists for "target seen but never aligned"
**File:** `src/main/java/frc/robot/commands/AlignAndShootCommand.java:136-180`
**Issue:** If the robot sees a target but can never align (e.g., the PID never converges, or the shooter never reaches speed), the ALIGN state has no timeout. The `ALIGN_TIMEOUT_SEC = 3.0` only applies when `!hasResult` (no target seen at all). Once a target IS seen, the timer is never checked again. The command will remain in ALIGN indefinitely until the operator's `AUTO_SHOOT_TIMEOUT_SEC` wrapper kicks in.
**Impact:** The command relies entirely on the external `.withTimeout()` wrapper for the stuck-with-target case. While the timeout exists in RobotContainer, the named command `AutoShoot` and operator trigger both have it, a direct `buildAlignAndShootCommand()` call from the dashboard could theoretically run forever.

### 7. `HubActivityTracker.isOurHubActive()` publishes to SmartDashboard with side effects
**File:** `src/main/java/frc/robot/HubActivityTracker.java:46-136`
**Issue:** This static utility method publishes to SmartDashboard (lines 112-116, 130-133) as a side effect. It's called both from `RobotContainer.periodicDashboard()` and from `AlignAndShootCommand.initialize()`. The SmartDashboard writes happen on every call, but the method is also called from `buildDashboardSnapshot()` (line 774) which runs every 20ms, doubling the SmartDashboard traffic for the same data. Additionally, the `SmartDashboard.putBoolean/putNumber` calls in the "gameDataAvailable=false" path (lines 112-116) silently return `true` to the caller, which masks the fact that the game data isn't actually available.
**Impact:** Minor performance issue from double publishing. No functional bug, but the code is misleading.

### 8. `buildIntakeGamePieceCommand()` doesn't stow the intake arm after completion
**File:** `src/main/java/frc/robot/RobotContainer.java:885-900`
**Issue:** The command deploys the arm to `INTAKE_DOWN_DEG` (line 893) and runs the roller, but after the roller stops (line 898), the arm is left in the down position. The comment says "Stop rollers and leave arm down (ready to stow)" but there is no subsequent stow command. In autonomous, this means the intake arm remains deployed after the `IntakeFuel` named command finishes, potentially interfering with driving or collision avoidance.
**Impact:** In autonomous, the intake arm stays deployed after picking up fuel. This could cause the arm to hit field elements or other robots.

---

## MODERATE ISSUES

### 9. `operatorController.rightTrigger().onTrue()` uses threshold detection inconsistently
**File:** `src/main/java/frc/robot/RobotContainer.java:611-616`
**Issue:** `rightTrigger()` uses the WPILib default threshold (0.5) for its boolean conversion. However, `driverController.rightTrigger().getAsBoolean()` (line 507) also uses the default threshold. The `TRIGGER_ACTIVE_THRESHOLD = 0.20` (line 121) is only used for dashboard diagnostics display (line 1047), not for actual trigger activation. This means the dashboard shows the trigger as "active" at 20% travel, but the actual command doesn't trigger until 50%.
**Impact:** Dashboard shows operator trigger active before the command actually fires, causing confusion.

### 10. `AlignAndShootCommand` uses `volatile` static fields for telemetry without atomicity
**File:** `src/main/java/frc/robot/commands/AlignAndShootCommand.java:36-43`
**Issue:** Multiple `volatile` static fields are written independently in `execute()` and `end()`. Since each write is independent, a reader on another thread (the dashboard service) could see an inconsistent snapshot (e.g., `telemetryState = "ALIGN"` but `telemetryHasTarget = false` from a previous cycle). `volatile` guarantees visibility but not atomicity across multiple fields.
**Impact:** Dashboard may briefly show inconsistent alignment state (e.g., "ALIGN" phase but stale target data). This is cosmetic but could confuse operators.

### 11. `DashboardFrame` refresh timer runs at 100ms (10 Hz) which is slower than robot loop
**File:** `src/dashboard/java/frc/dashboard/DashboardFrame.java:253`
**Issue:** The Swing `Timer` runs at 100ms intervals. Since the robot publishes at 50 Hz (20ms) and NT updates propagate faster, the dashboard only displays every 5th update. This is generally fine, but the trend charts (`TREND_CAPACITY = 1800` = "3 minutes at 10 Hz") and CSV logger sample at the dashboard rate, not the robot rate, so logged data has 100ms resolution.
**Impact:** No bug, but the CSV match log and trend data have lower resolution than what's available from NT. During fast transients (shooter spin-up, stall detection), the dashboard may miss brief events.

### 12. `VisionStreamPanel` and `TrendChartPanel` are not shown in QA scope but referenced
**File:** `src/dashboard/java/frc/dashboard/VisionStreamPanel.java`, `TrendChartPanel.java`
**Issue:** These panels fetch MJPEG streams from the robot. If the robot IP changes or the stream ports are wrong, the panel will silently fail with no user feedback beyond a status label.
**Impact:** Low severity, but operators may not realize the vision stream is down if they don't check the status label.

### 13. `RobotContainer.startVisionCamera()` returns null on failure, passed to thread
**File:** `src/main/java/frc/robot/RobotContainer.java:208-221, 147`
**Issue:** If `CameraServer.startAutomaticCapture()` throws an exception, `startVisionCamera()` returns `null`. This null is then passed to `RioVisionThread(visionCamera, ...)`. Inside `RioVisionThread.run()`, line 92 checks `if (usbCamera == null)` and throws, which is caught and logged, but the thread exits entirely. Meanwhile, `isCameraConnected()` on the SwerveSubsystem will always return false, and the dashboard will show the camera as "INIT_FAILED" but the dashboard still shows buttons like "Align + Shoot" as clickable.
**Impact:** If the camera fails to open, the align-and-shoot command will always abort with "No alliance HUB tag found" after 3 seconds. Not a crash, but there's no proactive warning to the operator that vision-based shooting is completely unavailable.

### 14. No red alliance auto paths defined
**File:** `src/main/java/frc/robot/RobotContainer.java:379-380`
**Issue:** Only `BlueDepot` and `BlueOutpost` auto paths are registered. PathPlanner's alliance flipping handles this via the `shouldFlipPath` lambda, but the auto names say "Blue Depot" and "Blue Outpost". When on the red alliance, the operator sees "Blue Depot" in the chooser and has to know that it auto-flips. This is functionally correct but could cause confusion at competition.
**Impact:** UX issue. Operators on red alliance see auto names referencing "Blue" which could cause hesitation or wrong auto selection.

### 15. `zeroHeading()` allowed in disabled mode from dashboard but button binds require swerve subsystem
**File:** `src/main/java/frc/robot/dashboard/RobotDashboardService.java:519-526`
**Issue:** The dashboard allows `zero_heading` in both disabled and teleop modes (`disabled || teleopEnabled`). However, the `zeroHeading()` method calls `swerve.zeroHeading()` which calls `pigeon.reset()` and `resetPose()`. Resetting the Pigeon 2 gyro while disabled is valid and useful (pre-match alignment), but the wording "Only allowed in disabled or teleop mode" is correct. No bug here, but the physical button Y on the driver controller calls `Commands.runOnce(() -> zeroHeading(), swerve)` which requires the swerve subsystem and therefore won't run if another command has swerve.
**Impact:** If a swerve validation command is running and the driver presses Y, the gyro zero will be silently blocked. The dashboard button bypasses this (no subsystem requirement) but the controller button does not.

---

## DASHBOARD-SPECIFIC ISSUES

### 16. `DashboardFrame` auto chooser combo doesn't sync bidirectionally on startup
**File:** `src/dashboard/java/frc/dashboard/DashboardFrame.java:684-727`
**Issue:** The `autoChooserCombo` is initialized empty (line 684) and populated during `refresh()` from robot-published options. However, `pendingAutoSelectionName` starts as null. If the user clicks "Apply" before selecting anything from the combo, the null check on line 705 prevents sending, but the combo shows the first item as visually selected even though `pendingAutoSelectionName` is null. The user may think they've selected an auto when they haven't clicked anything in the combo.
**Impact:** Minor UX issue. First-time apply click does nothing with no feedback.

### 17. Dashboard `level1ClimbButton` is visible but climber is disabled
**File:** `src/dashboard/java/frc/dashboard/DashboardFrame.java:751`
**Issue:** The "Level 1 Climb" button is created and displayed on the dashboard. When clicked, it sends the `LEVEL1_CLIMB` command to the robot, which is always rejected with "Climber disabled - no hardware installed" (line 570-572 of `RobotDashboardService`). The button gives no visual indication that it's permanently disabled.
**Impact:** Operator confusion. The button appears functional but always fails. Should be visually disabled or removed.

### 18. `DashboardData` record has climber fields that are always default values
**File:** `src/dashboard/java/frc/dashboard/DashboardData.java:26-28`
**Issue:** `climberArmed`, `climberPositionRot`, and `climberCurrentAmps` are always `false`, `0.0`, `0.0` because the climber subsystem is disabled. These fields are subscribed to on the dashboard side (`climberArmedSub`, etc.) and displayed in the Operator tab, but they always show zeroes.
**Impact:** Dashboard real estate wasted on non-functional climber data. Minor.

---

## CODE QUALITY / SAFETY CONCERNS

### 19. `IntakeRollerCommand` sets roller power twice per execute cycle
**File:** `src/main/java/frc/robot/commands/IntakeRollerCommand.java:100-109`
**Issue:** When the lockout auto-reset happens (lines 100-104), the code calls `intake.setRollerPower(forwardPower)`. Then immediately after (lines 107-108), it calls `intake.setRollerPower(protection.commandedPower(...))`. After reset, `protection.commandedPower()` returns `forwardPower` (since state is RUNNING), so the second call overwrites with the same value. This is harmless but wasteful. However, the real issue is that the lockout auto-reset sets the power before `protection.update()` could process the new state, leading to one cycle where the power is set to `forwardPower` by line 104 and then immediately overwritten by line 107.
**Impact:** No functional bug (same value written twice), but the code is confusing and could mask issues if the state machine logic changes.

### 20. `SwerveModule.getPosition()` coupling compensation may accumulate error
**File:** `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java:310-321`
**Issue:** The coupling compensation (`steerRotations * COUPLE_RATIO`) uses the cached steer angle, which is the steer motor's internal position. If the steer motor re-syncs from CANcoder (belt skip detection), the internal position jumps, which causes a corresponding jump in the coupling compensation. This jump would produce a sudden change in the reported drive distance, potentially causing a spike in odometry.
**Impact:** During a belt skip re-sync event, odometry may see a brief position jump. This is inherent to the coupling compensation approach and difficult to avoid, but worth being aware of during match replays.

### 21. `RobotContainer.periodicDashboard()` calls `HubActivityTracker.isOurHubActive()` for side effects only
**File:** `src/main/java/frc/robot/RobotContainer.java:692`
**Issue:** Line 692 calls `HubActivityTracker.isOurHubActive()` and discards the return value. This is solely for the SmartDashboard side effects inside that method. But the `buildDashboardSnapshot()` on line 688 already calls `HubActivityTracker.isOurHubActive()` (line 774), meaning the method is called twice per loop with redundant SmartDashboard writes.
**Impact:** Double SmartDashboard writes for HUB activity. Minor performance waste.

### 22. `SwerveSubsystem` belt-skip SmartDashboard indicator is never cleared
**File:** `src/main/java/frc/robot/subsystems/SwerveSubsystem.java:227-237`
**Issue:** When a belt skip is detected, `SmartDashboard.putBoolean("Swerve/" + mod.getName() + "_BeltSkip", true)` is set, but it is never reset to `false`. Once a belt skip is detected and corrected, the indicator remains `true` for the rest of the match, giving the false impression that the belt is still slipping.
**Impact:** Operators and pit crew cannot tell whether a belt skip is an ongoing issue or a one-time event that was already corrected.

### 23. `SwerveModule` CANcoder seed does not check status before seeding steer encoder
**File:** `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java:198-203`
**Issue:** The non-Pro steer encoder seeding calls `cancoderPosition.waitForUpdate(0.5).getValueAsDouble()` and immediately uses the result to seed the steer motor via `steerMotor.setPosition(cancoderRot)`. The code does not check whether `waitForUpdate()` returned an OK status. If CAN communication fails or times out, `getValueAsDouble()` returns a stale/default value (typically `0.0`), causing the steer motor to be seeded with an incorrect position.
**Impact:** On CAN bus congestion during startup, a module could be seeded to position 0.0 when the wheel is actually at a different angle. The robot would steer incorrectly until the periodic re-sync (every 1 second) detects and corrects the error.

### 24. `FeederSubsystem` lacks explicit motor inversion configuration
**File:** `src/main/java/frc/robot/subsystems/FeederSubsystem.java`
**Issue:** The feeder motor SparkMaxConfig does not call `config.inverted(...)`, unlike the adjacent `HopperSubsystem` which explicitly sets `config.inverted(Constants.Hopper.MOTOR_INVERTED)` with the comment "Wired opposite the feeder motor, so invert the SparkMax." This inconsistency suggests the feeder motor direction may rely on the physical wiring being correct rather than being explicitly controlled in software.
**Impact:** If the feeder motor wiring is changed or a replacement motor is installed with different wiring, the feeder will run backwards with no software-level inversion constant to adjust. Low risk but inconsistent with the hopper's explicit approach.

---

## RECOMMENDATIONS (non-bug, non-tuning)

1. **Add a "No Vision" indicator to the driver dashboard** when `ENABLE_VISION = false` or camera initialization fails, so operators know vision-based shooting won't work before they try it.

2. **Disable or hide the "Level 1 Climb" button** on the dashboard since the climber hardware is not installed, to avoid operator confusion.

3. **Consider adding alliance-neutral auto names** (e.g., "Depot Side" / "Outpost Side") since PathPlanner auto-flips for red alliance.

4. **Add an internal timeout to the ALIGN state** in `AlignAndShootCommand` for the "has target but can't converge" case, rather than relying solely on external `.withTimeout()` wrappers.

---

## FILES REVIEWED

### Robot Code (src/main/java/frc/robot/)
- `Main.java`, `Robot.java`, `RobotContainer.java`, `Constants.java`, `HubActivityTracker.java`

### Subsystems
- `SwerveSubsystem.java`, `ShooterSubsystem.java`, `IntakeSubsystem.java`, `FeederSubsystem.java`, `HopperSubsystem.java`

### Swerve Module
- `SwerveModule.java`, `SwerveCorner.java`, `SwerveValidationMode.java`, `SwerveCalibrationUtil.java`

### Commands
- `AlignAndShootCommand.java`, `IntakeRollerCommand.java`, `IntakeRollerProtection.java`, `IntakeHomeCommand.java`, `CalibrateCANcodersCommand.java`, `ValidateSwerveModuleCommand.java`

### Vision
- `RioVisionThread.java`, `VisionResult.java`, `VisionSupport.java`, `CameraDebugInfo.java`

### Dashboard Service (robot-side)
- `RobotDashboardService.java`, `DashboardSnapshot.java`, `ReadyToScoreEvaluator.java`, `ReadyToScoreResult.java`

### Custom Dashboard (src/dashboard/java/frc/dashboard/)
- `DashboardMain.java`, `DashboardFrame.java`, `DashboardData.java`, `DashboardNtClient.java`, `MatchCsvLogger.java`, `TrendDataStore.java`, `TrendChartPanel.java`, `VisionStreamPanel.java`, `EventReplayPanel.java`

### Build & Config
- `build.gradle`

### Tests
- All test files in `src/test/java/` (build environment unavailable to execute)
