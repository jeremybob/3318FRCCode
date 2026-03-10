# Driver / Operator Controls (2026)

Last updated: 2026-03-10
Source of truth: `src/main/java/frc/robot/RobotContainer.java`

Controller ports (`Constants.OI`):

- Driver controller: port `0`
- Operator controller: port `1`

## Driver controls (Port 0)

- Left stick Y/X: translation (field-relative by default)
- Right stick X: rotation
- Right Trigger (hold): precision mode (reduced drive speed)
- Left Bumper (hold): robot-relative drive mode
- Y (press): zero gyro heading
- B (hold): emergency stop drive output while held
- X (hold): X-lock (wheels angled inward to resist pushing)

## Operator controls (Port 1)

- Right Stick Y: manual intake tilt (open-loop power)
- Left Stick Y: manual hopper jog
- A (press): AlignOnly command (vision yaw alignment test, no shot)
- Right Trigger (>= 0.20, hold): AlignAndShoot command
- Right Bumper (hold): fallback shoot routine (no vision alignment)
- Left Trigger (hold): intake roller forward with stall protection
- Left Bumper (hold): intake roller reverse/eject
- X (press): intake home command
- Y (press): intake tilt toggle (stow/deploy PID target, manual stick override allowed)

## Disabled controls

- Climber controls are disabled in this build (no active operator climber bindings).

## Notes

- Teleop start schedules intake homing if needed.
- Intake tilt manual input deadband hysteresis: engage when `|rightY| > 0.12`, return to idle when `|rightY| < 0.08`.
