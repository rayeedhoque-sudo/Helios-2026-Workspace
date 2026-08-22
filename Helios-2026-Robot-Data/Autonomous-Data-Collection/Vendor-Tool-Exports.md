# Vendor Tool Export Checklist

Use screenshots when tools cannot export data directly. Screenshots still count as evidence if the page clearly shows device ID, device name, firmware, and settings.

## Phoenix Tuner X

Save to `inbox/vendor-exports/`.

Capture:

- Device list showing every CTRE device on the bus.
- Firmware versions and sticky faults.
- CAN bus utilization if available.
- Pigeon 2 ID and mount orientation.
- CANcoder IDs and absolute offsets after zeroing.
- Tuner X swerve project or generated constants.

Important unknowns this resolves:

- Whether CAN IDs 1-23 match the data sheet.
- Real CANcoder offsets.
- Whether the 2nd Pigeon exists.
- CTRE device firmware/fault state.

## REV Hardware Client 2

Save to `inbox/vendor-exports/`.

Capture for each SPARK MAX:

- CAN ID.
- Firmware version.
- Motor type.
- Idle mode.
- Inversion.
- Current limit.
- Follower setting.
- Faults/sticky faults.

Important unknowns this resolves:

- 3rd hopper NEO 2.0 CAN ID.
- Whether hopper motors should be followers or independent.
- Existing persisted SPARK settings that code must not accidentally wipe.
- Hood/slider current limit and brake mode status.

## Limelight

Save to `inbox/vendor-exports/`.

Capture:

- Camera hostname/name.
- Model and firmware.
- Pipeline settings.
- AprilTag family and filtering.
- Robot-space camera pose if configured.
- NetworkTables name.

Important unknowns this resolves:

- Limelight mount pose.
- Whether the code should trust MegaTag2.
- Which pipeline the robot uses for AprilTags.

## Driver Station

Save to `inbox/driver-station-logs/`.

Capture:

- `.dslog` and `.dsevents` files after every session.
- Screenshots or exports from DS Log File Viewer showing brownouts, battery voltage, comms loss, and CAN faults.

Important unknowns this resolves:

- Whether brownouts happen under specific mechanisms.
- Whether CAN faults correlate with mechanism activity.
- Battery voltage under load.

## PathPlanner

Save to `inbox/vendor-exports/`.

Capture:

- Robot config JSON.
- Autos and paths.
- Robot mass and MOI settings.
- Module locations, wheel radius, gearing, max speed, and current limit assumptions.

Important unknowns this resolves:

- Mismatches between PathPlanner, Tuner X, and code constants.
- Whether autos can move with nonzero constraints.
