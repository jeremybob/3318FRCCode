# QA Report: Team 3318 FRC 2026 (REBUILT) Robot Code
**Date:** 2026-03-09
**Reviewer:** Claude QA
**Scope:** Full codebase review (robot code, commands, subsystems, vision, custom dashboard)

---

## Fix Status Legend

| Status | Meaning |
|--------|---------|
| **FIXED** | Issue resolved in code |
| **NO FIX NEEDED** | Not a bug or accepted as-is |
| **DEFERRED** | Acknowledged, no action taken now |

---

## Summary

The codebase is well-structured and extensively documented. The custom Swing dashboard is impressively comprehensive. 31 issues identified below, ranging from critical bugs to code quality concerns, excluding tuning parameters as requested.

**22 issues fixed, 5 accepted as-is / not a bug, 4 deferred.**

---

## CRITICAL BUGS

### 1. `CalibrateCANcodersCommand` leaks NetworkTables publishers on every execution — **FIXED**
**File:** `src/main/java/frc/robot/commands/CalibrateCANcodersCommand.java`
**Issue:** The `publishCalibrationValue()` method created a `DoublePublisher` with try-with-resources, which immediately closed it after `set()`. Published values may not reach the dashboard because the publisher is closed before NT flushes.
**Fix:** Replaced try-with-resources publishers with persistent field-level publishers initialized in the constructor. Publishers remain open so calibration values persist on the dashboard.

### 2. `AlignAndShootCommand` does not filter by alliance HUB tags — **NO FIX NEEDED**
**File:** `src/main/java/frc/robot/commands/AlignAndShootCommand.java`
**Issue:** When alliance is unknown, the vision thread accepts any HUB tag from either alliance.
**Resolution:** Accepted as current behavior. The vision thread's fallback of accepting either alliance tag is preferable to refusing to shoot at all when alliance data is unavailable.

### 3. `IntakeSubsystem.setTiltPower()` software limits only active when homed — **DEFERRED**
**File:** `src/main/java/frc/robot/subsystems/IntakeSubsystem.java:167-183`
**Issue:** Software limits are only enforced when `isHomed == true`. Before homing, the operator can command full tilt power with no positive-direction software limit.
**Resolution:** Deferred. The limit switch protects the negative direction, and operators are trained to home the intake before use. Adding pre-home limits would require assumptions about the arm's starting position.

### 4. `SwerveSubsystem` accepts `VisionResult` in constructor but never uses it — **FIXED**
**File:** `src/main/java/frc/robot/subsystems/SwerveSubsystem.java`
**Issue:** Constructor accepted `AtomicReference<VisionResult>` but never stored or used it.
**Fix:** Removed unused `VisionResult` import and constructor parameter. Constructor now only takes `AtomicReference<Double> lastVisionFrameTimestampSec`.

---

## SIGNIFICANT BUGS

### 5. `DashboardNtClient` creates duplicate pub/sub on the same topic keys — **FIXED**
**File:** `src/dashboard/java/frc/dashboard/DashboardNtClient.java`
**Issue:** For each command topic, both a publisher and subscriber were created on the same key, causing potential race conditions where the publisher's writes reflect back to the subscriber.
**Fix:** Removed all 10 `IntegerSubscriber` fields that duplicated publisher topics. Changed `sendCommand()` to use simple `++localSeq` instead of reading from the subscriber. Removed the `nextCommandSequence` static method.

### 6. `AlignAndShootCommand` ALIGN state has no timeout for "target seen but can't converge" — **FIXED**
**File:** `src/main/java/frc/robot/commands/AlignAndShootCommand.java`
**Issue:** Once a target was seen, the ALIGN state had no internal timeout for PID/shooter convergence failure.
**Fix:** Added `alignOverallTimer` and `ALIGN_CONVERGENCE_TIMEOUT_SEC = 5.0`. When the target is visible but alignment (yaw, pitch, shooter RPM) can't converge within 5 seconds, the command aborts with "Alignment convergence timeout".

### 7. `HubActivityTracker.isOurHubActive()` publishes to SmartDashboard with side effects — **FIXED**
**File:** `src/main/java/frc/robot/HubActivityTracker.java`
**Issue:** Static utility method published to SmartDashboard as a side effect, called redundantly from multiple locations.
**Fix:** Completely rewritten as a pure function that only returns a boolean. All SmartDashboard publishing removed; the caller (`RobotDashboardService`) handles display.

### 8. `buildIntakeGamePieceCommand()` doesn't stow the intake arm after completion — **DEFERRED**
**File:** `src/main/java/frc/robot/RobotContainer.java:885-900`
**Issue:** After the roller stops, the arm is left in the down position.
**Resolution:** Deferred. The arm-down-after-intake behavior is intentional for the current game strategy. The autonomous paths handle stowing separately.

---

## MODERATE ISSUES

### 9. `operatorController.rightTrigger().onTrue()` uses threshold detection inconsistently — **FIXED**
**File:** `src/main/java/frc/robot/RobotContainer.java`
**Issue:** The operator trigger used the default 0.5 threshold while the dashboard showed it active at 0.20.
**Fix:** Changed to `operatorController.rightTrigger(TRIGGER_ACTIVE_THRESHOLD).onTrue(...)` so the command fires at the same threshold as the dashboard display.

### 10. `AlignAndShootCommand` uses `volatile` static fields for telemetry without atomicity — **FIXED**
**File:** `src/main/java/frc/robot/commands/AlignAndShootCommand.java`
**Issue:** Multiple volatile static fields could produce inconsistent snapshots when read from the dashboard thread.
**Fix:** Replaced 8 volatile static fields with an immutable `TelemetrySnapshot` record published via a single `volatile` reference. All fields are updated atomically in one assignment.

### 11. `DashboardFrame` refresh timer runs at 100ms (10 Hz) which is slower than robot loop — **NO FIX NEEDED**
**File:** `src/dashboard/java/frc/dashboard/DashboardFrame.java:253`
**Issue:** Dashboard samples at 10 Hz instead of the robot's 50 Hz.
**Resolution:** Not a problem. 10 Hz is standard for operator dashboards. CSV logging at 10 Hz is sufficient for post-match analysis, and the Swing UI cannot meaningfully render faster than this.

### 12. `VisionStreamPanel` silently fails with no user feedback — **FIXED**
**File:** `src/dashboard/java/frc/dashboard/VisionStreamPanel.java`
**Issue:** Stream errors showed minimal feedback and no progressive backoff.
**Fix:** Added a consecutive error counter with descriptive error messages and progressive backoff: `"STREAM ERROR (Nx): <message> — check robot IP / camera"` with `Math.min(1000L * consecutiveErrors, 5000L)` sleep.

### 13. `RobotContainer.startVisionCamera()` returns null on failure, passed to thread — **FIXED**
**File:** `src/main/java/frc/robot/RobotContainer.java`
**Issue:** If camera initialization fails, null was passed directly to `RioVisionThread` which would throw.
**Fix:** Added null check before thread start. If `visionCamera` is null, the vision thread is not started, and a warning is logged.

### 14. No red alliance auto paths defined — **NO FIX NEEDED**
**File:** `src/main/java/frc/robot/RobotContainer.java:379-380`
**Issue:** Auto names say "Blue Depot" and "Blue Outpost".
**Resolution:** PathPlanner's `shouldFlipPath` lambda automatically flips paths for red alliance. The naming convention is understood by the drive team.

### 15. `zeroHeading()` controller button requires swerve subsystem unnecessarily — **FIXED**
**File:** `src/main/java/frc/robot/RobotContainer.java`
**Issue:** The gyro zero button required the swerve subsystem, meaning it would be silently blocked if another command (e.g., swerve validation) was using swerve.
**Fix:** Removed the swerve subsystem requirement from the gyro zero button binding. The `pigeon.reset()` call doesn't need exclusive subsystem access.

---

## DASHBOARD-SPECIFIC ISSUES

### 16. `DashboardFrame` auto chooser combo doesn't sync on startup — **FIXED**
**File:** `src/dashboard/java/frc/dashboard/DashboardFrame.java`
**Issue:** `pendingAutoSelectionName` started as null, so the first "Apply" click did nothing even though the combo visually showed the first item.
**Fix:** When `desiredSelection` falls back to the first auto option (because `pendingAutoSelectionName` is null), `pendingAutoSelectionName` is now also set to that value, ensuring the Apply button works immediately.

### 17. Dashboard `level1ClimbButton` is visible but climber is disabled — **FIXED** (pre-existing)
**File:** `src/dashboard/java/frc/dashboard/DashboardFrame.java`
**Issue:** The "Level 1 Climb" button appeared functional but always failed.
**Resolution:** Already handled in the codebase. The button is permanently disabled (`setButtonState(..., false, ...)`) with tooltip "Climber disabled — no hardware installed". The climber label shows "DISABLED — no hardware". The climber code is retained (commented out) for future hardware installation.

### 18. `DashboardData` record has climber fields that are always default values — **DEFERRED**
**File:** `src/dashboard/java/frc/dashboard/DashboardData.java:26-28`
**Issue:** Climber fields always show zeroes since hardware is not installed.
**Resolution:** Deferred. Fields retained for future climber hardware. The dashboard display is already annotated as "DISABLED — no hardware".

---

## CODE QUALITY / SAFETY CONCERNS

### 19. `IntakeRollerCommand` sets roller power twice per execute cycle — **FIXED**
**File:** `src/main/java/frc/robot/commands/IntakeRollerCommand.java`
**Issue:** Lockout auto-reset set `intake.setRollerPower(forwardPower)` redundantly before `protection.commandedPower()` would set the same value.
**Fix:** Removed the redundant `intake.setRollerPower(forwardPower)` call in the lockout auto-reset block.

### 20. `SwerveModule.getPosition()` coupling compensation may accumulate error on belt-skip re-sync — **FIXED**
**File:** `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`
**Issue:** Coupling compensation used absolute steer position, so a belt-skip re-sync jump would cause an odometry spike.
**Fix:** Switched to delta-based coupling compensation. Added `couplingOffsetMotorRot` accumulator and `lastCouplingSteerRot` tracking. Only the delta in steer rotation is applied to coupling compensation each cycle, preventing jumps during re-sync events.

### 21. `RobotContainer.periodicDashboard()` calls `HubActivityTracker.isOurHubActive()` redundantly — **FIXED**
**File:** `src/main/java/frc/robot/RobotContainer.java`
**Issue:** `isOurHubActive()` called twice per loop — once for dashboard snapshot, once for SmartDashboard side effects.
**Fix:** Removed the redundant call. The `buildDashboardSnapshot()` call already includes the result, and with issue #7's fix, the method no longer has SmartDashboard side effects.

### 22. `SwerveSubsystem` belt-skip SmartDashboard indicator is never cleared — **FIXED**
**File:** `src/main/java/frc/robot/subsystems/SwerveSubsystem.java`
**Issue:** Belt-skip indicator was set to true but never reset to false after correction.
**Fix:** Changed to write the current check result each cycle: `SmartDashboard.putBoolean("Swerve/" + mod.getName() + "_BeltSkip", skipped)` — clears to false when no skip is detected.

### 23. `SwerveModule` CANcoder seed does not check status before seeding steer encoder — **FIXED**
**File:** `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`
**Issue:** `waitForUpdate()` result was used without checking OK status. CAN timeout could seed a 0.0 value.
**Fix:** Added `cancoderStatus.getStatus().isOK()` check. If the status is not OK, the seed is skipped with a warning printed to stderr, allowing the periodic re-sync to correct the position later.

### 24. `FeederSubsystem` lacks explicit motor inversion configuration — **FIXED**
**File:** `src/main/java/frc/robot/subsystems/FeederSubsystem.java`
**Issue:** No explicit `config.inverted()` call, unlike the adjacent HopperSubsystem.
**Fix:** Added `config.inverted(false)` with a comment explaining that positive power feeds toward the shooter and what to change if the motor runs backward after a swap.

### 25. Stale comment in `Constants.java` misidentifies CAN IDs — **FIXED**
**File:** `src/main/java/frc/robot/Constants.java`
**Issue:** Comment incorrectly stated "IDs 10 and 11 are shooter / Steer is on 19".
**Fix:** Updated comment to "Back-right IDs are 5/6/11. IDs 16-17 are shooter, 19 is hopper."

### 26. Driver B button (emergency stop) only stops for one scheduler cycle — **FIXED**
**File:** `src/main/java/frc/robot/RobotContainer.java`
**Issue:** `onTrue(Commands.runOnce(...))` only stopped the robot for 20ms before the default drive command resumed.
**Fix:** Changed to `whileTrue(Commands.run(() -> swerve.stop(), swerve))` — the robot stays stopped as long as the B button is held.

### 27. `buildIntakeGamePieceCommand()` does not wait for arm to reach deploy position — **NO FIX NEEDED**
**File:** `src/main/java/frc/robot/RobotContainer.java:893-896`
**Issue:** Rollers start spinning before the arm reaches the deployed position.
**Resolution:** Not a bug. The arm PID moves the arm while rollers spin. The overlap is intentional — it reduces cycle time in autonomous, and the rollers spinning early doesn't cause any mechanical issue.

### 28. `buildIntakeTiltToggleCommand()` holds intake subsystem for 3 seconds when not homed — **FIXED**
**File:** `src/main/java/frc/robot/RobotContainer.java`
**Issue:** When `isHomed()` is false, the `startEnd` command still held the intake subsystem for up to 3 seconds, blocking manual tilt control.
**Fix:** Changed to return an instant command (no sustained requirement) when not homed, so the intake subsystem is not blocked.

### 29. Auto command cancel in `teleopInit()` references unwrapped command — **FIXED**
**File:** `src/main/java/frc/robot/Robot.java`
**Issue:** `autonomousCommand` pointed to the original unwrapped command while the `.withTimeout()` wrapper was what actually got scheduled.
**Fix:** Now stores the wrapped command: `autonomousCommand = autonomousCommand.withTimeout(...)` so that `teleopInit()` cancels the correct object.

### 30. Dashboard subscribes to CANcoder calibration data that is never continuously published — **FIXED**
**File:** `src/dashboard/java/frc/dashboard/DashboardNtClient.java`, `src/main/java/frc/robot/commands/CalibrateCANcodersCommand.java`
**Issue:** Dashboard subscribed to calibration topics, but they were published with try-with-resources (immediately closed).
**Fix:** Resolved by fix #1 — persistent publishers in `CalibrateCANcodersCommand` now keep the topics alive. Calibration values will appear on the dashboard after the calibration command runs.

### 31. `TAG_HEIGHT_M` uses outer tag dimension but `tagPixelHeight()` measures inner corner span — **DEFERRED**
**File:** `src/main/java/frc/robot/Constants.java:456-457`, `src/main/java/frc/robot/vision/RioVisionThread.java:241-248`
**Issue:** The constant name is misleading — `FOCAL_LENGTH_PIXELS` is a combined calibration factor, not the camera's optical parameter.
**Resolution:** Deferred. The self-calibration procedure compensates for this automatically. Adding a clarifying comment is low priority and the code produces correct results when calibrated as documented.

---

## RECOMMENDATIONS (non-bug, non-tuning)

1. ~~**Add a "No Vision" indicator to the driver dashboard**~~ — Consider for future improvement.

2. ~~**Disable or hide the "Level 1 Climb" button**~~ — **Done.** Button is permanently disabled with tooltip.

3. **Consider adding alliance-neutral auto names** (e.g., "Depot Side" / "Outpost Side") since PathPlanner auto-flips for red alliance. — Accepted as-is per team preference.

4. ~~**Add an internal timeout to the ALIGN state**~~ — **Done.** 5-second convergence timeout added (fix #6).

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
