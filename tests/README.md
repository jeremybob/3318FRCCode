# Robot Update Tests

This repository includes Gradle-based unit tests and also keeps a lightweight
source-level regression script for fast compatibility checks.

## Files

- `tests/run_robot_update_tests.sh`
  - Verifies drivetrain Falcon constants
  - Verifies hopper (8:1) and intake tilt (10:1) gearing constants
  - Verifies geometry gating and tolerant vision-loss behavior
  - Verifies operator override bindings and align-shoot command wiring
  - Verifies 2026 naming (`IntakeGamePiece`, `FourPieceClimbAuto`, `TwoPieceAuto`)
  - Verifies autonomous AutoShoot timeout configuration
  - Verifies dashboard service wiring in RobotContainer/Robot

## Run

```bash
bash tests/run_robot_update_tests.sh
```

For full Java compile/tests:

```bash
./gradlew test dashboardClasses
```
