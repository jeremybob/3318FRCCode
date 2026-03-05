# Rio Camera Fallback Plan

## Overview

If the Raspberry Pi 5 coprocessor cannot be brought online, we can fall back to
a USB camera plugged directly into the roboRIO 2. This document describes the
recommended implementation strategy: **AprilTag detect-only processing in a
background thread** (no full pose solve on the rio).

The goal is to preserve vision-based targeting for `AlignAndShootCommand` while
putting as close to **zero load on the main robot loop** as possible.

---

## Why Not Just Run PhotonVision on the Rio?

PhotonVision *can* run on a roboRIO 2, but it performs full pose estimation
(solvePnP) on every frame, which is CPU-intensive. On the rio's dual-core
Cortex-A53, this risks starving the 20 ms robot loop (drivetrain PID, shooter
velocity control, PathPlanner, etc.) and causing loop overruns.

Instead, we use WPILib's lightweight `AprilTagDetector`, which only finds tags
and returns their pixel coordinates — **3-5x cheaper** than a full pose solve.
We extract yaw and distance from the pixel data using simple geometry.

---

## Architecture

```
┌─────────────────────────────────────┐
│         Background Thread           │
│                                     │
│  UsbCamera (320x240, ~15 fps)       │
│  CvSink ──► grab frame (Mat)        │
│          ──► AprilTagDetector.detect │
│          ──► filter for HUB tag IDs │
│          ──► compute yaw/distance   │
│          ──► write AtomicReference   │
│                                     │
│  Runs as fast as it can (~10-15 fps)│
│  Zero impact on main robot loop     │
└──────────────┬──────────────────────┘
               │ AtomicReference<VisionResult>
               ▼
┌─────────────────────────────────────┐
│       Main Robot Loop (20 ms)       │
│                                     │
│  AlignAndShootCommand:              │
│    read latest VisionResult         │
│    use yawDeg for PD turn control   │
│    use distance for shooter RPS     │
│    state machine unchanged          │
│                                     │
│  SwerveSubsystem:                   │
│    odometry only (encoders + gyro)  │
│    no vision pose correction        │
└─────────────────────────────────────┘
```

### Key Design Decisions

1. **Background thread** — vision processing never blocks the main loop. The
   main loop reads a single `AtomicReference`, which is effectively free.
2. **Detect-only** — `AprilTagDetector.detect()` finds tags and returns corner
   pixel coordinates. We skip `estimatePose()` / solvePnP entirely.
3. **Pre-filtered results** — the background thread filters for alliance HUB
   tags and picks the best (closest) one before publishing, so the main loop
   does zero filtering work.
4. **Odometry-only pose** — without the Pi doing pose estimation, we rely on
   wheel encoders + Pigeon 2 gyro for field position. This is accurate enough
   for a 2.5-minute match when properly calibrated.

---

## New Components

### `VisionResult` (new data class)

An immutable snapshot published by the background thread:

| Field              | Type     | Description                                |
|--------------------|----------|--------------------------------------------|
| `tagId`            | `int`    | Fiducial ID of best HUB tag, or -1 if none |
| `yawDeg`           | `double` | Horizontal angle to tag center             |
| `pitchDeg`         | `double` | Vertical angle to tag center               |
| `tagPixelHeight`   | `double` | Tag height in pixels (distance proxy)      |
| `timestampSec`     | `double` | FPGA timestamp when frame was captured     |

No PhotonVision types cross the thread boundary — just plain doubles.

### `RioVisionThread` (new class)

A daemon thread that:

1. Opens a `UsbCamera` via `CameraServer.startAutomaticCapture()` at 320x240
2. Gets a `CvSink` to grab grayscale frames
3. Creates a WPILib `AprilTagDetector` (36h11 tag family)
4. Loops:
   - `cvSink.grabFrame(mat)` — blocks until next frame
   - `detector.detect(mat)` — returns array of `AprilTagDetection`
   - Filters for alliance HUB tag IDs
   - Picks the largest (closest) detection
   - Computes yaw/pitch from pixel coordinates (see math below)
   - Publishes `VisionResult` to shared `AtomicReference`

**Thread lifecycle:** started in `RobotContainer` constructor, runs until robot
code exits. Set as a daemon thread so it doesn't prevent shutdown.

---

## Pixel-to-Angle Math

No camera calibration matrix or solvePnP needed. Just the camera's field of
view (from the spec sheet or a one-time measurement).

### Yaw (horizontal angle to target)

```
tagCenterX = average of 4 corner X pixel coordinates
yawDeg = (tagCenterX - imageCenterX) / imageCenterX * (horizontalFOV / 2)
```

### Pitch (vertical angle to target)

```
tagCenterY = average of 4 corner Y pixel coordinates
pitchDeg = (imageCenterY - tagCenterY) / imageCenterY * (verticalFOV / 2)
```

Note: Y is inverted because pixel Y increases downward but pitch-up should be
positive.

### Distance from tag pixel height

Uses the pinhole camera model:

```
distanceM = (realTagHeightM * focalLengthPixels) / tagPixelHeight
```

where `tagPixelHeight` is the vertical span of the tag's corner coordinates.

**One-time calibration:** place the robot at a known distance (e.g., 2 m) from
a tag, measure the tag's pixel height, then solve:

```
focalLengthPixels = tagPixelHeight * knownDistanceM / realTagHeightM
```

Validate by checking pixel height at 1 m, 3 m, and 4 m.

---

## Changes to Existing Code

### `AlignAndShootCommand`

The state machine (SPIN_UP → ALIGN → CLEAR → FEED → DONE) is **unchanged**.
The PD turn controller is unchanged. Only the vision data source changes:

| Current (PhotonVision on Pi)                          | Fallback (Rio detect-only)                            |
|-------------------------------------------------------|-------------------------------------------------------|
| `PhotonCamera camera` field                           | `AtomicReference<VisionResult> latestResult` field    |
| `getLatestCameraResult()` returns `PhotonPipelineResult` | `latestResult.get()` returns `VisionResult` or null |
| `findBestHubTarget()` filters by tag ID at read time  | Pre-filtered by background thread                     |
| `hubTarget.getYaw()` (from PhotonVision)              | `result.yawDeg` (from pixel math)                     |
| `estimateDistanceM(hubTarget.getPitch())`             | `result.pitchDeg` same formula, or use `tagPixelHeight` |
| `isShotGeometryFeasible(target)`                      | Same check on `result.pitchDeg`                       |
| `target.getFiducialId()`                              | `result.tagId`                                        |

### `SwerveSubsystem`

- Remove `PhotonCamera` and `PhotonPoseEstimator` fields
- Remove `updateVisionPoseEstimator()` method
- Pose estimation uses wheel encoders + Pigeon 2 only
- `isCameraConnected()` can check if the vision thread is producing results

### `RobotContainer`

- Remove `PhotonCamera camera` field
- Create `RioVisionThread` in constructor, start it
- Pass the shared `AtomicReference<VisionResult>` to `AlignAndShootCommand`
- `SwerveSubsystem` constructor no longer takes a camera

### `Constants.java`

Replace PhotonVision-specific constants:

```java
// USB camera settings
public static final int CAMERA_DEVICE_ID = 0;
public static final int CAMERA_WIDTH = 320;
public static final int CAMERA_HEIGHT = 240;
public static final int CAMERA_FPS = 15;

// Camera intrinsics (measure once per camera)
public static final double HORIZONTAL_FOV_DEG = 70.0;  // from spec sheet
public static final double VERTICAL_FOV_DEG = 43.0;    // from aspect ratio
public static final double FOCAL_LENGTH_PIXELS = 300.0; // calibrate

// Standard FRC AprilTag size (6.5 inches)
public static final double TAG_HEIGHT_M = 0.1651;
```

Keep unchanged: `YAW_TOLERANCE_DEG`, `TURN_kP`, `TURN_kD`, `MAX_ROT_CMD`,
`TARGET_LOSS_TOLERANCE_SEC` (consider bumping to 0.5-0.75 s), HUB tag ID arrays.

### Dependencies

- **Remove** `vendordeps/photonlib.json` (PhotonVision library)
- **Add** nothing — `AprilTagDetector`, `CameraServer`, `CvSink` are all in
  WPILib core + the `wpilibj-apriltag` package that ships with WPILib 2026

---

## Tuning Guide

| Parameter             | How to Calibrate                                              |
|-----------------------|---------------------------------------------------------------|
| Camera FOV            | Point at known-width object at known distance, measure pixels |
| Focal length (pixels) | Tag pixel height at known distance: `f = px * dist / tagH`   |
| Distance accuracy     | Measure tag pixel height at 1 m, 2 m, 3 m, 4 m and validate |
| `TARGET_LOSS_TOLERANCE_SEC` | Increase from 0.35 s to 0.5-0.75 s for lower frame rate |
| `YAW_TOLERANCE_DEG`   | May need to widen from 2.0 to 3.0 deg for pixel-based yaw    |
| `TURN_kP` / `TURN_kD` | Re-tune if yaw responsiveness feels different                 |

---

## Performance Expectations

| Metric             | Expected Value                          |
|--------------------|-----------------------------------------|
| Detection FPS      | 10-15 fps at 320x240 on roboRIO 2       |
| Main loop impact   | ~0 ms (single AtomicReference read)     |
| Detection latency  | ~70-100 ms (one frame of pipeline lag)  |
| Yaw accuracy       | ~1-2 deg at 2-4 m shooting distance     |
| Distance accuracy  | ~10-15% (good enough for shooter curve) |

---

## Risks and Mitigations

| Risk                          | Mitigation                                                      |
|-------------------------------|-----------------------------------------------------------------|
| Odometry drift (no pose correction) | Pigeon 2 heading is stable. Reset odometry at auto start. Drift is typically < 6 in over a full match |
| Pixel distance less accurate  | `calculateTargetRPS()` clamps to 20-90 RPS. Speed curve is forgiving — 10% distance error is ~2-3 RPS |
| USB camera disconnects        | `cvSink.grabFrame()` returns error, thread retries next frame. Main loop sees stale result, loss tolerance handles it |
| OpenCV memory                 | Reuse a single `Mat` per iteration — never allocate in hot loop |
| Lower frame rate means slower alignment | Acceptable: even at 10 fps, ALIGN phase completes in < 1 s typically. 3 s timeout provides margin |

---

## Autonomous Implications

Without vision pose correction, autonomous accuracy depends entirely on:

1. **Accurate odometry calibration** — wheel diameter, gear ratio, CANcoder offsets
2. **Gyro reset at auto start** — `resetOdometry()` with known starting pose
3. **Predictable shooter speeds** — consider pre-computed RPS values for known
   auto shooting positions instead of calculating from vision distance

The `EightFuelClimbAuto` path would still work. The `AutoShoot` named command
uses `AlignAndShootCommand`, which still gets yaw alignment from the camera. The
main risk is that odometry-based position might place the robot slightly off from
the ideal shooting spot, but the alignment step corrects for angular error and the
shooter speed curve is forgiving of small distance errors.

---

## Future Upgrade: Adding Lazy Pose Solve

If detect-only works well but you want occasional pose correction, the
architecture supports a clean upgrade without restructuring:

1. In the vision thread, every ~5th frame, call `estimateTagCornersPose()`
   on the best detection (only when robot velocity is low)
2. Publish a separate `AtomicReference<Optional<Pose2d>>`
3. `SwerveSubsystem` consumes it with high standard deviations (low trust)

This gives periodic drift correction without the full cost of per-frame
pose estimation.

---

## Alternative Strategies Considered

### Strategy 2A: Pixel-Only (No AprilTag Detection)

Use raw color/blob detection instead of AprilTag decoding. Cheapest option but
loses tag IDs (can't distinguish alliance HUBs). Only viable if nothing else
works.

### Strategy 2C: Detect + Lazy Pose Solve

Run detection every frame but only solve pose every 5th frame when stationary.
Best accuracy but most complex. Recommended as a future upgrade to 2B+2D, not
as the initial implementation.

### Strategy 3: Pure Odometry Aiming

Skip vision entirely for targeting. Calculate heading to HUB from `getPose()`
and known HUB field coordinates. Use odometry distance for shooter speed. This
is essentially an enhanced version of the existing right-bumper fallback shoot.
Could be combined with 2B+2D as a further fallback.
