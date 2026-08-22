# Helios 2026 — CAN Bus & DIO Quick Map (Team 9704)

Single CAN bus = default roboRIO bus (`rio`, CAN 2.0). Motor models in **bold** are CAD-confirmed; others from code. Full detail in [Hardware-Data-Sheet.md](Hardware-Data-Sheet.md).

## CAN devices (IDs 1–23, no gaps)

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
| 13 | Shooter flywheel A (master) | TalonFX | **Kraken X60** | Shooter flywheel |
| 14 | Shooter flywheel B (follower, Aligned) | TalonFX | **Kraken X60** | Shooter flywheel |
| 15 | Shooter flywheel C (follower, Aligned — was Opposed; skipped in team test 2026-07-16) | TalonFX | **Kraken X60** | Shooter flywheel |
| 16 | Shooter flywheel D (follower, Opposed — spin test 2026-07-16: wrong way when Aligned) | TalonFX | **Kraken X60** | Shooter flywheel |
| 17 | Kicker | **Victor SPX** (fw 22.1) | **CIM** (motor swapped from the CAD Kraken, 2026-07-16) | Kicker into shooter — ID verified LIVE 2026-07-16 (Phoenix diagnostics scan: only Victor on the bus, at 17) |
| 18 | Intake roller | TalonFX | **Kraken X44** | Intake rollers |
| 19 | Shooter hood / angle | SPARK MAX | **NEO 550** | Hood pivot (abs encoder) |
| 20 | Hopper A | SPARK MAX | **NEO 2.0** | Hopper indexing |
| 21 | Hopper B | SPARK MAX | **NEO 2.0** | Hopper indexing |
| 22 | Intake slider / pivot | SPARK MAX | **NEO V1.1** | Intake slider |
| 23 | Pigeon 2.0 | Pigeon2 | — | Heading (IMU) |

## Not yet assigned a CAN ID (exists in CAD, missing in code)

| Device | Controller | Motor | Note |
|---|---|---|---|
| Hopper C (3rd hopper motor) | SPARK MAX | **NEO 2.0** | CAD shows 3 hopper NEOs + 5 SPARK MAX; code wires only 2. Needs a CAN ID (24+). |
| Pigeon 2.0 #2 | Pigeon2 | — | CAD shows two Pigeons; code uses one. Confirm if redundant/backup, else drop. |

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
