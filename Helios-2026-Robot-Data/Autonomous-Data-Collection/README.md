# Autonomous Data Collection

This folder is the intake point for new Helios 2026 robot facts that Codex or Claude can turn into safer robot code.

Use this when the laptop is near the robot, when you have screenshots from vendor tools, or when you have photos/logs to hand back to an agent. Keep raw data in `inbox/`; agents should summarize findings into `findings/` before updating the main data sheet.

## Start Here

1. Run [Robot-Session-Runbook.md](Robot-Session-Runbook.md) during a robot-connected session.
2. Put photos, screenshots, logs, and exports in the matching `inbox/` subfolder.
3. Fill [Measurement-Intake-Worksheet.md](Measurement-Intake-Worksheet.md) with any confirmed values.
4. Give the folder path to Codex or Claude and ask it to run [Agent-Ingest-Instructions.md](Agent-Ingest-Instructions.md).

## Folder Map

| Path | Purpose |
|---|---|
| `inbox/photos/` | Robot, wiring, labels, mechanism, and Limelight mount photos. |
| `inbox/vendor-exports/` | Phoenix Tuner X, REV Hardware Client, PathPlanner, and Limelight exports or screenshots. |
| `inbox/driver-station-logs/` | Driver Station logs and log viewer exports. |
| `inbox/robot-logs/` | roboRIO/WPILib logs, AdvantageKit logs, CTRE hoot logs, NetworkTables captures. |
| `inbox/measurements/` | Completed worksheets, field measurements, battery data, hand notes. |
| `findings/` | Agent-written summaries of what was learned and what remains unknown. |
| `scripts/` | Local helper scripts for copying available data into `inbox/`. |

## Evidence Tags

- `[verified-on-robot]`: confirmed by physical inspection, vendor tool readback, or live robot test.
- `[verified-in-code]`: read directly from source, vendordep, generated config, or build output.
- `[vendor-export]`: exported or screenshotted from Phoenix Tuner X, REV Hardware Client, Limelight, Driver Station, or PathPlanner.
- `[photo-assumed]`: inferred from a photo. Use only as a candidate until confirmed.
- `[CAD]`: from Onshape/CAD. Treat like an assumption before deploy.
- `[unknown]`: not confirmed yet; leave code as `// TODO` with units.

## Safety Rule

Do not convert photo-only or CAD-only facts into deployed motor directions, CAN IDs, current limits, sensor polarity, or encoder offsets without confirmation from code, vendor tool readback, or a robot test.
