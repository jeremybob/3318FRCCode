# Robot Update Tests

This repository snapshot does not include a Java build wrapper (`gradlew`) or Maven files,
so these checks are implemented as source-level tests.

## Files

- `tests/run_robot_update_tests.sh`
  - Verifies drivetrain Falcon constants
  - Verifies hopper (8:1) and intake tilt (10:1) gearing constants
  - Verifies geometry gating and tolerant vision-loss behavior
  - Verifies fallback-speed override button binding
  - Verifies autonomous AutoShoot timeout is configurable (currently 6s)

## Run

```bash
bash tests/run_robot_update_tests.sh
```
