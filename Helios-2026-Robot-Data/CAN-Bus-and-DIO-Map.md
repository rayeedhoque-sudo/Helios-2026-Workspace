# Helios 2026 — CAN Bus & DIO Quick Map (Team 9704)

Single CAN bus = default roboRIO bus (`rio`, CAN 2.0). Motor models in **bold** are CAD-confirmed; others from code. Full detail in [Hardware-Data-Sheet.md](Hardware-Data-Sheet.md).

## CAN devices (V2 — updated 2026-09-29 from CAD; launcher/intake-roller IDs pending)

| CAN ID | Device | Controller | Motor | Mechanism |
|---|---|---|---|---|
| 1  | Back Left Steer | TalonFX | **Kraken X44** | Swerve steer BL |
| 2  | Back Left Drive | TalonFX | **Kraken X60** | Swerve drive BL |
| 3  | Back Left CANcoder | CANcoder | — | Swerve azimuth BL |
| 4  | Front Right Steer | TalonFX | **Kraken X44** | Swerve steer FR |
| 5  | Front Right Drive | TalonFX | **Kraken X60** | Swerve drive FR |
| 6  | Front Right CANcoder | CANcoder | — | Swerve azimuth FR |
| 7  | Back Right Steer | TalonFX | **Kraken X44** | Swerve steer BR |
| 8  | Back Right Drive | TalonFX | **Kraken X60** | Swerve drive BR |
| 9  | Back Right CANcoder | CANcoder | — | Swerve azimuth BR |
| 10 | Front Left Steer | TalonFX | **Kraken X44** | Swerve steer FL |
| 11 | Front Left Drive | TalonFX | **Kraken X60** | Swerve drive FL |
| 12 | Front Left CANcoder | CANcoder | — | Swerve azimuth FL |
| TBD | Launcher flywheel L1 (belted 1:1 to L2) | TalonFX | **Kraken X60** | Flywheel (V2) |
| TBD | Launcher flywheel L2 | TalonFX | **Kraken X60** | Flywheel (V2), 18T→26T |
| TBD | Launcher flywheel R (mirrored) | TalonFX | **Kraken X60** | Flywheel (V2) |
| TBD | Launcher feeder (replaces CIM kicker) | TalonFX | **Kraken X60** | 3-roller feed, 2.5:1 |
| TBD | Launcher hood (replaces NEO 550) | TalonFX | **Kraken X44** | Hood sector, ~42.5:1 |
| **16** | Intake front rollers | TalonFX | **Kraken X60** (was X44 on 18) | Intake rollers, 2:1. Team-stated 2026-09-30: the V1 flywheel-D motor (CAN 16) now drives the intake rollers. **Robot code still treats 16 as flywheel D** |
| 20? | Hopper A | SPARK MAX | **NEO 2.0** | Hopper belt floor (V2), confirm ID |
| 21? | Hopper B | SPARK MAX | **NEO 2.0** | Hopper belt floor (V2), confirm ID |
| 22 | Intake slider / jackshaft | SPARK MAX | **NEO 2.0** (CAD label V1.1) | Intake slider, MAXPlanetary 20:1 |
| 23 | Pigeon 2.0 | Pigeon2 | — | Heading (IMU) |

## V2 notes (2026-09-29)

- The team has **wired** the V2 motors but **has not assigned CAN IDs to new/replaced motors yet**. Don't guess them.
- **Gone from V1:** 4th flywheel, Victor SPX + CIM kicker (V1 CAN 17), hood SPARK MAX + NEO 550 (V1 CAN 19), intake X44 (V1 CAN 18). The V2 CAD still shows 4 SPARK MAX; one is probably the leftover hood controller. Confirm whether it's still on the bus.
- Pigeon 2.0 #2: the V2 CAD has one in the drivetrain and one in the launcher; code uses one (CAN 23). Confirm.

## DIO

| Port | Device | Status |
|---|---|---|
| DIO 8 | Indexer beam-break A | Constant declared (`LightSensor.INDEXER_SENSOR_ID_A`), **but `Sensors.java` is a stub — never constructed; reads always `true`.** |
| DIO 9 | Indexer beam-break B | Same — `LightSensor.INDEXER_SENSOR_ID_B`, stubbed. |

## Other electronics (non-CAN / network)

| Device | ID | Note |
|---|---|---|
| Limelight 4 | NT name `limelight-knight` | AprilTag vision |

> **No climber this season.** The old `ClimbSubsystemConstants` (both CAN IDs `0`, all-zero) is dead — do not use.
