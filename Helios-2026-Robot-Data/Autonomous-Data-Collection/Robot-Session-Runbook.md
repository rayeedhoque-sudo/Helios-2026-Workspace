# Robot-Connected Data Collection Runbook

Use this during a session with the laptop connected to the robot by USB, Ethernet, or robot Wi-Fi. The goal is to capture facts that desk-side CAD/code mining cannot verify.

## 1. Session Setup

- Confirm the robot is disabled and safely supported.
- Put a fresh battery on the robot and note battery label plus measured internal resistance.
- Connect to the robot network.
- Create a new session folder under `inbox/measurements/` named `YYYY-MM-DD-session-notes`.
- Run `scripts/Collect-FRC-Data.ps1` from PowerShell to copy local logs and known config files into `inbox/`.

## 2. CAN And Vendor Tool Readback

Capture these from Phoenix Tuner X:

| Needed Fact | Where To Save |
|---|---|
| Full CAN device list with names, IDs, firmware, bus, faults | `inbox/vendor-exports/phoenix-device-list-*` |
| CANcoder offsets after zeroing | `inbox/vendor-exports/phoenix-cancoder-offsets-*` |
| Pigeon 2 mount orientation and ID | `inbox/vendor-exports/phoenix-pigeon-*` |
| Swerve project/generator output | `inbox/vendor-exports/tuner-x-swerve-*` |

Capture these from REV Hardware Client 2:

| Needed Fact | Where To Save |
|---|---|
| SPARK MAX IDs, firmware, motor type, idle mode, current limit, inversion | `inbox/vendor-exports/rev-spark-configs-*` |
| The missing 3rd hopper motor ID and role | `inbox/vendor-exports/rev-hopper-*` |
| Hood and slider current limit/readback config | `inbox/vendor-exports/rev-hood-slider-*` |

Capture these from Limelight:

| Needed Fact | Where To Save |
|---|---|
| Camera name, model, IP, pipeline, AprilTag settings | `inbox/vendor-exports/limelight-config-*` |
| Camera pose/mount offset if configured | `inbox/vendor-exports/limelight-pose-*` |
| Screenshot of web UI settings page | `inbox/vendor-exports/limelight-screenshot-*` |

## 3. Physical Verification

Fill [Measurement-Intake-Worksheet.md](Measurement-Intake-Worksheet.md). Highest priority:

- 3rd hopper NEO 2.0 CAN ID and whether it follows another motor.
- Whether the 2nd Pigeon 2 exists on the robot.
- Breaker size and wire gauge for every motor controller.
- Beam-break installation and active-high vs active-low behavior.
- Limelight lens height, pitch, forward offset, and lateral offset.
- Shooter release height and wheel diameter.
- Kicker, hopper, and intake final reductions.
- Real robot weight.

## 4. Mechanism Bring-Up Logs

When code exists, run each mechanism one at a time while disabled/enabled as appropriate and collect logs:

| Mechanism | Log Signals |
|---|---|
| Drive | battery voltage, drive supply/stator current, wheel speed, odometry faults |
| Steer | CANcoder absolute position, steer current, closed-loop error |
| Shooter flywheel | supply/stator current per Kraken, velocity, voltage, setpoint |
| Hood | SPARK current, absolute angle, setpoint, limit clamp |
| Intake slider | SPARK current, commanded direction, stall detector, timeout reason |
| Hopper/kicker | currents, sensor states, feed state, jam condition |

Save logs to `inbox/robot-logs/`.

## 5. End Of Session

- Copy Driver Station logs into `inbox/driver-station-logs/`.
- Add a short note in `findings/YYYY-MM-DD-session-summary.md` with what changed.
- Ask Codex or Claude to run the ingest instructions and update the main data documents.
