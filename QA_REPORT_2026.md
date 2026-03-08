# QA Report — FRC Team 3318 (2026 REBUILT)

**Date:** 2026-03-08
**Reviewer:** Claude QA (Opus 4.6)
**Codebase:** 3318FRCCode — Java 17, WPILib 2026, GradleRIO 2026.2.1
**Game:** 2026 FRC REBUILT (FUEL collection, HUB scoring, climbing)

---

## Executive Summary

The codebase is **well-structured and competition-ready for a first event**, with strong safety practices, comprehensive CAN bus management, and a working vision fallback. However, there are **several issues that could affect match performance** — ranging from a missing climber subsystem (game points left on the table) to uncalibrated physics constants and limited autonomous routines. No critical safety bugs were found.

**Severity Legend:**
- **CRITICAL** — Could cause robot disable, damage, or match loss
- **HIGH** — Will noticeably hurt match performance
- **MEDIUM** — Should be fixed before competition but not a blocker
- **LOW** — Improvement opportunity / code quality

---

## CRITICAL Issues

### C1. No Climber Implementation — 15 Auto Points + Endgame Points Lost
**Files:** `Constants.java`, `RobotContainer.java` (all climber code commented out)

The climber subsystem is fully disabled (CAN IDs 20/21 commented out, all bindings removed). In REBUILT, Level 1 climb is worth **15 points in auto** (max 2 robots) and additional points in endgame. This is a significant scoring opportunity being left on the table.

**Impact:** Loss of 15+ points per match from climbing alone.
**Action:** If climber hardware is installed, uncomment and wire up the subsystem before competition.

### C2. Only 2 Autonomous Routines — Both Blue Alliance Only
**Files:** `src/main/deploy/pathplanner/autos/`, `RobotContainer.java:374-375`

Only `BlueDepot.auto` and `BlueOutpost.auto` are defined. There are **no Red-side autonomous routines**. While PathPlanner can flip Blue paths for Red alliance, this depends on `DriverStation.getAlliance()` returning correctly — which can fail in practice mode or if FMS data is delayed.

Additionally, only 2 strategies (depot/outpost) is thin for a competitive event. Teams typically need 3-5 autos to adapt to alliance partners and opponents.

**Impact:** If alliance detection fails on Red, the robot drives the wrong path. Limited strategic flexibility.
**Action:** Test Red-side path flipping thoroughly. Consider adding more auto variants (e.g., multi-fuel pickup, preload-only-shoot, do-nothing-but-climb).

### C3. Shooter Physics Constants Are Unverified — Shots May Miss
**Files:** `Constants.java:298-302`

Three critical constants are marked `TUNE ME` and appear to be theoretical/estimated rather than measured:
- `SHOT_ANGLE_DEG = 55.0` — Launch angle must be measured with protractor or slow-mo video
- `HUB_SCORING_HEIGHT_M = 1.124` — Must match the actual 2026 field HUB opening height
- `SHOOTER_EXIT_HEIGHT_M = 20 inches` — Must be measured from floor to shooter wheel center

The `calculateTargetRPS()` function uses projectile motion physics with these constants. If any are wrong by even a few degrees/inches, shots will consistently miss at certain distances.

**Impact:** Vision-guided shots may calculate wrong RPS, causing FUEL to miss the HUB.
**Action:** Measure all three values on the actual robot and field. Validate with test shots at 3+ distances.

---

## HIGH Severity Issues

### H1. Vision Focal Length Is Uncalibrated — Distance Estimation Unreliable
**File:** `Constants.java:442`

`FOCAL_LENGTH_PIXELS = 300.0` is marked `CALIBRATE ME`. This value directly affects distance estimation from AprilTag pixel size. An incorrect focal length means `calculateTargetRPS()` receives wrong distance inputs, compounding with issue C3.

**Impact:** Distance-based shot speed will be systematically off until calibrated.
**Action:** Calibrate by placing robot at a known distance from a tag, measuring tag pixel height, and computing `f = px * d / TAG_HEIGHT_M`.

### H2. Intake Tilt PID Is Very Conservative — May Not Track Setpoints
**File:** `Constants.java:350`

`TILT_kP = 0.05` with `kI = 0` and `kD = 0` is extremely conservative. With a 10:1 gearbox and NEO motor, this may result in:
- Slow tilt response (intake takes too long to deploy/stow)
- Steady-state error (arm doesn't fully reach target angle)
- Arm drooping under gravity when partially deployed

**Impact:** Intake deployment may be sluggish during fast cycling.
**Action:** Tune on-robot. Start by increasing kP to 0.1-0.2 and add kD if oscillation occurs.

### H3. Steer PID Values Are Starting Estimates — Not Tuned on Robot
**File:** `Constants.java:249-253`

`STEER_kP_NON_PRO = 70.0` and `STEER_kD_NON_PRO = 0.2` are described as "near-tuned starting target." If these are too high, modules will oscillate (visible as wheel jitter). If too low, modules will be sluggish to rotate, causing path-following errors.

**Impact:** Swerve steering oscillation or sluggishness during auto paths and teleop.
**Action:** Run single-module validation mode and tune steer response before driving.

### H4. HUB AprilTag IDs May Not Match 2026 Field
**File:** `Constants.java:479-480`

The tag IDs for Red and Blue HUBs are:
- Red: `{2, 3, 4, 5, 8, 9, 10, 11}`
- Blue: `{18, 19, 20, 21, 24, 25, 26, 27}`

These need to be verified against the official 2026 REBUILT field AprilTag layout. If even one ID is wrong, the robot may ignore valid HUB tags or aim at non-HUB tags.

**Impact:** Vision-guided shooting fails entirely if tag IDs don't match the field.
**Action:** Cross-reference with the official 2026 AprilTag field layout document before first event.

### H5. No Game Piece Detection Sensor — Blind Feeding
**Files:** All subsystems

There is no sensor (beam break, proximity, etc.) to detect whether a FUEL is loaded in the hopper/feeder path. The robot feeds based on timers only (`FEED_TIME_SEC = 0.75s`). This means:
- Dry-firing when no game piece is present (wastes cycle time)
- No way to count stored FUEL (up to 8 can be preloaded in REBUILT)
- Operator must visually confirm game piece presence

**Impact:** Wasted shots and cycle time from dry-firing.
**Action:** Consider adding a beam-break or IR sensor in the feeder path.

---

## MEDIUM Severity Issues

### M1. AlignAndShootCommand Does Not Prevent Shooting at Inactive HUB
**File:** `AlignAndShootCommand.java:114-119`

The command logs a WARNING when the HUB is inactive but **does not abort the shot**. In REBUILT, shooting into an inactive HUB scores 0 points and wastes FUEL.

```java
if (!HubActivityTracker.isOurHubActive()) {
    System.out.println("[AlignAndShoot] WARNING: Alliance HUB is currently INACTIVE!");
    // ... continues with shot anyway
}
```

**Impact:** FUEL wasted by shooting during inactive HUB shifts.
**Action:** Consider either aborting the shot or adding operator confirmation when HUB is inactive. At minimum, surface this warning prominently on the dashboard.

### M2. Shot Geometry Check Missing Yaw Validation — May Fire While Misaligned
**File:** `AlignAndShootCommand.java:324-330`

`isShotGeometryFeasible()` only validates the **pitch** angle (vertical). It does **not** check whether the yaw (horizontal alignment) is within a reasonable bound. This means the robot could evaluate geometry as "feasible" and proceed toward feeding even when the robot is pointed 45+ degrees away from the target. The PID controller does check yaw tolerance before transitioning from ALIGN to CLEAR, but the geometry check itself doesn't gate on yaw.

```java
private boolean isShotGeometryFeasible(VisionResult result) {
    double pitchDeg = result.pitchDeg();
    // ... only checks pitch, not yaw
    return isShotPitchFeasible(pitchDeg);
}
```

**Impact:** Edge case where geometry is "feasible" but robot is severely misaligned. The PID gate catches this in ALIGN, but if vision is intermittent, geometry could pass while yaw is large.
**Action:** Add a yaw magnitude check to `isShotGeometryFeasible()` (e.g., `Math.abs(yawDeg) < 45.0`).

### M3. Intake Homing Scheduled Before Robot Enables — May Conflict
**File:** `RobotContainer.java:148`

`CommandScheduler.getInstance().schedule(buildIntakeHomeCommand())` is called in the constructor, but the CommandScheduler won't execute commands while disabled. The home command will execute when the robot first enables, potentially conflicting with autonomous commands if auto enables first.

**Impact:** If the robot enables in auto, the intake home command and auto path commands may fight for the intake subsystem.
**Action:** Verify that auto routines include their own `HomeIntake` named command (they do via `registerPathPlannerCommands`), but the startup-scheduled one may still interfere. Consider making startup homing conditional on auto vs teleop.

### M4. CANcoder Offsets May Need Recalibration
**File:** `Constants.java:199-207`

There are commented-out older offset values that differ from the active ones:
- FL: was `-0.35889`, now `-0.368408` (delta: 0.0095 rot = ~3.4°)
- BL: was `-0.03271`, now `-0.017578` (delta: 0.015 rot = ~5.4°)

These deltas suggest the offsets have been recalibrated at least once. If wheels have been removed or modules serviced since the last calibration, offsets may be stale.

**Impact:** Misaligned swerve modules cause crab-walking and poor auto path tracking.
**Action:** Recalibrate all 4 offsets using `CalibrateCANcoders` auto before every event.

### M5. No Simulation Support Enabled
**File:** `build.gradle` — `includeDesktopSupport = false`

Desktop simulation is disabled. This prevents using WPILib's simulation GUI for testing code changes without a robot. For a team that may need to iterate quickly at competition, this is a missed opportunity.

**Impact:** Cannot test code changes without deploying to the robot.
**Action:** Enable `includeDesktopSupport = true` and create basic simulation support.

### M6. Hardcoded Current Limits Not Centralized
**Files:** `SwerveModule.java`, `ShooterSubsystem.java`, `IntakeSubsystem.java`

Motor current limits are hardcoded in subsystem files rather than centralized in Constants:
- Swerve drive: 60A stator, 40A supply (SwerveModule.java)
- Swerve steer: 40A stator (SwerveModule.java)
- Shooter: 80A stator, 60A supply (ShooterSubsystem.java)

Only intake limits are partially in Constants. This makes it harder to tune current limits as a team.

**Impact:** Maintenance burden; easy to miss when adjusting limits.
**Action:** Move all current limits to `Constants` classes.

### M7. Brownout Threshold Hardcoded
**File:** `Robot.java:61`

`batteryVoltage < 7.0` is hardcoded. The FRC brownout threshold is 6.3V, but the alert threshold (7.0V) is a team preference that should be in Constants.

**Impact:** Minor maintenance issue.
**Action:** Move to `Constants.Robot.BROWNOUT_ALERT_VOLTAGE`.

### M8. PathPlanner Translation/Rotation PID Are Guesses
**File:** `RobotContainer.java:253-257`

Both translation and rotation PID for PathPlanner are set to `kP = 5.0, kI = 0, kD = 0`. These are starting estimates that need robot validation. If too aggressive, auto paths will overshoot waypoints. If too conservative, the robot won't track paths tightly.

**Impact:** Auto path following may be inaccurate.
**Action:** Test auto paths and tune PID values. Start by running a simple straight-line path and measuring tracking error.

### M9. `secondsUntilNextShiftChange()` Is Never Called
**File:** `HubActivityTracker.java:142-153`

The `secondsUntilNextShiftChange()` method exists but is never used by any command or dashboard publishing. This information would be valuable for operators deciding whether to shoot now or wait for the next active window.

**Impact:** Operators lack timing information for HUB shift changes.
**Action:** Publish this value to SmartDashboard and/or use it in scoring decision logic.

---

## LOW Severity Issues

### L1. No CI/CD Pipeline
No `.github/` workflows exist. All testing is manual. For a student team iterating on code, a basic build/test check on push would catch compile errors early.

### L2. Dashboard Phase 4 (Competition Hardening) Not Complete
Per `docs/CUSTOM_DASHBOARD_PLAN.md`, Phase 4 (competition hardening) is listed as "upcoming." This includes connection resilience and error recovery that would be important at events with spotty network.

### L3. Vision Thread Does Not Detect Camera Disconnect Gracefully
**File:** `RioVisionThread.java:130-143`

When `grabFrame()` fails, the thread logs an error (throttled to 1/sec) and continues retrying. However, the main loop only sees stale `VisionResult` data. There's no mechanism to notify the operator that vision is down beyond the camera heartbeat timeout check.

### L4. `IntakeRollerCommand` Locks Out After 3 Stalls But Doesn't Auto-Reset
**File:** `IntakeRollerCommand.java:106`

After 3 stall detections, the command ends with `isLockedOut()`. The operator must release and re-press the trigger to reset. This is by design but could frustrate operators during fast cycling if they don't realize the lockout occurred.

### L5. Constants Comment Says "MK4 L2" but Code Uses MK4i
**File:** `Constants.java:10`

Line 10 says "MK4 L2 + Falcon 500" but the steer gear ratio (21.43) and motor inversion pattern are specifically for MK4**i**. The comment should say "MK4i L2" to avoid confusion.

### L6. Shooter `periodic()` Always Publishes `AtSpeed` Against `TARGET_RPS`
**File:** `ShooterSubsystem.java:101-102`

The dashboard boolean `Shooter/AtSpeed` always checks against the default `TARGET_RPS = 60`, not the current distance-calculated target. During an AlignAndShoot sequence where the target may be 45 or 75 RPS, this dashboard indicator is misleading.

### L7. No Autonomous Timeout Safeguard
**File:** `Robot.java:78-87`

The autonomous command returned by `getAutonomousCommand()` has no global timeout wrapper. If a PathPlanner auto hangs (e.g., path tracking gets stuck), it could run into teleop. The `teleopInit()` does cancel it, but there's a risk window.

### L8. `buildIntakeGamePieceCommand` Leaves Arm Down After Intake
**File:** `RobotContainer.java:834-849`

The auto intake command stops rollers but doesn't stow the arm (comment says "leave arm down, ready to stow"). In auto, a deployed intake arm could collide with field elements or other robots.

### L9. Vision `estimateDistanceM` Fallback Chain Could Return NaN
**File:** `AlignAndShootCommand.java:256-261`

If the primary distance estimate returns non-finite AND `estimateDistanceFromPitch` also returns NaN (e.g., camera looking straight ahead at a tag), `calculateTargetRPS` receives NaN and falls back to `TARGET_RPS`. This is handled correctly but silently — no log message indicates the distance fallback was used.

### L10. Operator Right Stick Controls Both Intake Tilt AND Was Climber
**File:** `RobotContainer.java:527-548`

With the climber disabled, the operator right stick Y now controls intake tilt (was previously climber). This mapping change means operators trained on the old mapping might be confused. Document the current control layout for the drive team.

---

## Test Results

### Unit Tests: UNABLE TO RUN
Build failed due to `GradleRIO:2026.2.1` not being downloadable in this environment (403 Forbidden from Gradle plugin portal). This is an environment limitation, not a code issue. The project includes 7 unit test files:

1. `DashboardNtClientSequenceTest.java` — NT client command sequencing
2. `DashboardMainArgsTest.java` — Dashboard argument parsing
3. `IntakeRollerProtectionTest.java` — Intake stall detection logic
4. `AlignAndShootCommandTest.java` — Vision-guided shooting
5. `RobotDashboardServiceTest.java` — Dashboard service wiring
6. `ReadyToScoreEvaluatorTest.java` — Shoot readiness evaluation
7. `VisionSupportTest.java` — Vision processing

**Action:** Run `./gradlew test` on a machine with WPILib 2026 installed to verify all tests pass before competition.

---

## Summary by Severity

| Severity | Count | Key Themes |
|----------|-------|------------|
| CRITICAL | 3 | Missing climber, limited autos, unverified physics |
| HIGH | 5 | Uncalibrated vision, conservative PID, unverified tag IDs, no piece detection |
| MEDIUM | 9 | Inactive HUB shooting, missing yaw gate, hardcoded values, no simulation, PID tuning needed |
| LOW | 10 | Code quality, comments, edge cases, operator UX |

---

## Strengths

1. **Robust CAN management** — Unique ID validation at startup, frame rate optimization, belt-skip detection, retry logic on config apply
2. **Safety-first design** — Intake homing gate prevents position commands before zeroing, current limits on all motors, soft limits enforced
3. **Clean architecture** — Command-based with proper subsystem requirements, well-separated concerns
4. **Excellent code documentation** — Extensive comments explaining "why" not just "what," student-friendly
5. **Comprehensive dashboard** — Custom cross-platform dashboard with 70+ telemetry fields, camera debug info, control event logging
6. **Smart stall detection** — Intake roller auto-reverses on jam with configurable retry limits
7. **HUB activity tracking** — Game-specific shift timing logic for 2026 REBUILT
8. **Vision fallback** — AprilTag detection runs directly on roboRIO when coprocessor is unavailable
9. **Thread-safe vision** — AtomicReference for lock-free communication between vision thread and main loop
10. **PathPlanner fallback** — Graceful degradation with constants-based config when GUI settings are missing

---

## Pre-Competition Checklist

- [ ] Calibrate all 4 CANcoder offsets (use `CalibrateCANcoders` auto)
- [ ] Measure and set `SHOT_ANGLE_DEG` with protractor
- [ ] Measure and set `SHOOTER_EXIT_HEIGHT_M` from floor
- [ ] Verify `HUB_SCORING_HEIGHT_M` against 2026 field spec
- [ ] Calibrate `FOCAL_LENGTH_PIXELS` with known-distance tag test
- [ ] Verify HUB AprilTag IDs against official 2026 field layout
- [ ] Tune intake tilt kP on robot (start at 0.1-0.2)
- [ ] Tune swerve steer kP on robot using validation mode
- [ ] Tune PathPlanner translation/rotation PID with test paths
- [ ] Test Red-side path flipping with both auto routines
- [ ] Run all 7 unit tests on a WPILib machine
- [ ] Decide on climber: install hardware or remove code entirely
- [ ] Run `tests/run_robot_update_tests.sh` regression script
- [ ] Test full shoot cycle at 3+ distances
- [ ] Verify HUB shift timing against FMS Game Data at first event
- [ ] Brief drive team on current control mapping (esp. operator changes)
