#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CONSTANTS_FILE="$ROOT_DIR/src/main/java/frc/robot/Constants.java"
CONTAINER_FILE="$ROOT_DIR/src/main/java/frc/robot/RobotContainer.java"
ALIGN_FILE="$ROOT_DIR/src/main/java/frc/robot/commands/AlignAndShootCommand.java"

assert_contains() {
  local file="$1"
  local pattern="$2"
  local description="$3"
  if grep -Fq "$pattern" "$file"; then
    echo "PASS: $description"
  else
    echo "FAIL: $description"
    echo "  Expected to find: $pattern"
    echo "  In file: $file"
    exit 1
  fi
}

echo "Running robot update checks..."

# 1) Drivetrain / Falcon constants
assert_contains "$CONSTANTS_FILE" "public static final double DRIVE_MOTOR_FREE_SPEED_RPS = 6380.0 / 60.0;" \
  "Swerve drive free speed constant uses Falcon free speed"
assert_contains "$CONSTANTS_FILE" "public static final double DRIVE_kV = 12.0 / DRIVE_MOTOR_FREE_SPEED_RPS;" \
  "Swerve kV derives from configured drive free speed"

# 2) Intake + Hopper gearing
assert_contains "$CONSTANTS_FILE" "public static final double TILT_POS_CONV_DEG = 360.0 / 10.0;" \
  "Intake tilt conversion uses 10:1 reduction"
assert_contains "$CONSTANTS_FILE" "public static final class Hopper {" \
  "Hopper constants section exists"
assert_contains "$CONSTANTS_FILE" "public static final double GEAR_RATIO = 8.0;" \
  "Hopper gear ratio set to 8:1"

# 3) Vision/geometry shooting checks
assert_contains "$ALIGN_FILE" "if (!isShotGeometryFeasible(result)) {" \
  "Align phase aborts when geometry is infeasible"
assert_contains "$ALIGN_FILE" "if (!hasShootableTarget()) {" \
  "Clear/feed phases verify shootability (with tolerance support)"
assert_contains "$CONSTANTS_FILE" "public static final double TARGET_LOSS_TOLERANCE_SEC = 0.35;" \
  "Vision loss tolerance constant exists"
assert_contains "$ALIGN_FILE" "return hasRecentValidTarget();" \
  "Command tolerates brief vision loss using recent valid target memory"
assert_contains "$CONSTANTS_FILE" "public static final double MIN_SHOT_PITCH_DEG" \
  "Vision minimum pitch limit exists"
assert_contains "$CONSTANTS_FILE" "public static final double MAX_SHOT_PITCH_DEG" \
  "Vision maximum pitch limit exists"

# 4) Override button + fallback speed
assert_contains "$CONSTANTS_FILE" "public static final double FALLBACK_RPS = 52.0;" \
  "Fallback shot speed constant exists"
assert_contains "$CONTAINER_FILE" "operatorController.rightBumper().onTrue(" \
  "Operator right bumper is bound"
assert_contains "$CONTAINER_FILE" "Constants.Shooter.FALLBACK_RPS" \
  "Operator right bumper shoots using fallback speed"
assert_contains "$CONTAINER_FILE" "operatorController.rightTrigger().onTrue(" \
  "Operator right trigger is bound"
assert_contains "$CONTAINER_FILE" "return new AlignAndShootCommand(swerve, shooter, feeder, hopper, intake, camera);" \
  "Align-and-shoot builder uses the shared camera and subsystems"

# 5) Legacy naming cleanup with compatibility
assert_contains "$CONTAINER_FILE" "NamedCommands.registerCommand(\"IntakeGamePiece\"" \
  "Preferred path event name uses game-piece naming"
assert_contains "$CONTAINER_FILE" "NamedCommands.registerCommand(\"IntakeFuel\"" \
  "Legacy IntakeFuel path event alias is retained"

# 6) Auto mode AutoShoot timeout is configurable
assert_contains "$CONSTANTS_FILE" "public static final double AUTO_SHOOT_TIMEOUT_SEC = 6.0;" \
  "AutoShoot timeout constant exists and is set to 6s"
assert_contains "$CONTAINER_FILE" ".withTimeout(Constants.Auto.AUTO_SHOOT_TIMEOUT_SEC)" \
  "AutoShoot named command uses configurable timeout"

# 7) Dashboard service integration
assert_contains "$CONTAINER_FILE" "new RobotDashboardService(new RobotDashboardService.Actions() {" \
  "RobotContainer constructs dashboard service"
assert_contains "$CONTAINER_FILE" "public void periodicDashboard() {" \
  "RobotContainer exposes dashboard periodic hook"
assert_contains "$ROOT_DIR/src/main/java/frc/robot/Robot.java" "robotContainer.periodicDashboard();" \
  "Robot periodic loop updates dashboard service"

echo "All robot update checks passed."
