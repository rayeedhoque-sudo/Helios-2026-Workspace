# Log And Telemetry Ingest

This file defines what to collect once robot code exists and how an agent should use it.

## Logs To Collect

| Source | Save Location | Why |
|---|---|---|
| Driver Station `.dslog` / `.dsevents` | `inbox/driver-station-logs/` | Brownouts, comms loss, voltage, robot enable timeline. |
| WPILib DataLog `.wpilog` | `inbox/robot-logs/` | Custom robot telemetry. |
| CTRE hoot logs | `inbox/robot-logs/` | CAN and device-level diagnostics. |
| AdvantageKit logs, if used | `inbox/robot-logs/` | Replayable robot behavior. |
| NetworkTables snapshot | `inbox/robot-logs/` | Limelight and Shuffleboard values. |

## Minimum Signals To Add To Robot Code

When building the robot project, expose these signals through DataLog, NetworkTables, or a subsystem logging layer:

| Subsystem | Required Signals |
|---|---|
| Power | battery voltage, brownout flag, loop time |
| Swerve | drive supply/stator current, steer current, module positions, CANcoder absolute angles |
| Shooter | flywheel setpoint, velocity, voltage request, supply/stator current per motor |
| Hood | setpoint angle, measured angle, SPARK current, clamp state |
| Intake | roller command/current, slider command/current, stall detector state, timeout reason |
| Hopper | motor commands/current, beam-break states, feed state |
| Kicker | command/current, jam detector state |
| Vision | Limelight target valid, tag ID, camera pose, pose ambiguity, MegaTag2 accepted/rejected |

## Agent Ingest Procedure

1. Identify logs by session date and match them to any notes in `findings/`.
2. Extract voltage dips, brownout events, CAN faults, and high-current intervals.
3. Correlate events with mechanism commands.
4. Summarize facts into `findings/YYYY-MM-DD-log-analysis.md`.
5. Update the main data sheet only for durable facts, not one-off observations.

## Brownout Review Thresholds

Flag a finding when:

- Battery voltage drops below 7.5 V during normal operation.
- Driver Station logs show brownout events.
- Radio/comms drop during shooter spin-up or drive acceleration.
- A mechanism current limit is hit for more than 1 second.
- A motor temperature rises quickly during repeated mechanism use.
