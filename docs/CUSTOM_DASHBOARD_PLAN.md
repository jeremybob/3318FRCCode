# 3318 Custom Dashboard Plan (Production Phases)

Last updated: 2026-02-21

## Objectives

- Cross-platform operation (macOS + Windows) using a single codebase.
- High-signal driver/operator UX with safety-gated writable actions.
- Typed NetworkTables contract with command acknowledgements.
- Legacy game naming cleanup (`Note`/`Fuel` -> `Piece`/`GamePiece`) without breaking old autos.

## Phase 1: Robot Telemetry Contract (implemented)

Robot publishes typed topics under `Dashboard/`:

- `robot/*`: mode, enabled, alliance, match time.
- `robot/timestamp_sec` heartbeat for stale-telemetry detection on dashboard.
- `drive/*`: pose and heading.
- `shooter/*`: left/right RPS, at-speed.
- `intake/*`: homed, limit switch, tilt, roller current.
- `feeder/*`, `hopper/*`: current.
- `climber/*`: armed, position, current.
- `align/*`: command state, vision status, yaw/pitch, abort reason.
- `shot/*`: final `ready` boolean + `ready_reason`.
- `meta/contract_version`.

## Phase 2: Safe Command Bridge (implemented)

Dashboard commands are sequence-based topics under `Dashboard/cmd/*`:

- `zero_heading_seq`
- `stop_drive_seq`
- `intake_home_seq`
- `align_shoot_seq`
- `fallback_shoot_seq`
- `level1_climb_seq`

Safety gates:

- `zero_heading`: disabled or teleop only.
- `stop_drive`: always allowed.
- `intake_home`: disabled only.
- `align_shoot`: enabled teleop only.
- `fallback_shoot`: enabled teleop only.
- `level1_climb`: enabled teleop and climber arm gate required.

Robot publishes ack topics:

- `ack/last_command`
- `ack/last_status` (`OK` or `REJECTED`)
- `ack/last_seq`
- `ack/message`
- `ack/timestamp_sec`

## Phase 3: Cross-platform Dashboard App (implemented MVP)

Location: `src/dashboard/java/frc/dashboard`

- `DashboardMain`: launch entrypoint.
- `DashboardNtClient`: NT subscriptions + command publishing.
- `DashboardFrame`: Driver / Operator / Pit tabs.
- `DashboardData`: typed data model.

Run locally:

```bash
./gradlew runDashboard
```

Optional connection arguments:

```bash
./gradlew runDashboard --args="--team 3318"
./gradlew runDashboard --args="--host 10.33.18.2"
```

## Phase 4: Competition Hardening (next)

1. Add trend plots (battery, loop timing, shooter spin-up latency).
2. Add autonomous visual chooser on dashboard (with disabled-mode apply).
3. Add event replay mode from logged CSV/NT snapshots.
4. Add UI integration tests for command button -> ack behavior.

Implemented in this pass:

- Competition-focused UI refresh for readability and reduced clutter.
- Driver shot checklist + "next action" guidance from readiness reason.
- Command pre-check lockouts/tooltips to prevent invalid requests before ack rejection.
- Telemetry freshness indicator using `robot/timestamp_sec`.
- Event feed for connection/ack/readiness state transitions.
