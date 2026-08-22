# Measurement Intake Worksheet

Fill only values that were actually measured, read from a tool, or confirmed by a person at the robot. Leave blanks empty.

## Session

| Field | Value |
|---|---|
| Date | |
| People present | |
| Robot connection type | USB / Ethernet / Wi-Fi |
| Battery label | |
| Battery internal resistance | |
| Notes | |

## CAN And Devices

| Device | Expected ID | Observed ID | Source | Notes |
|---|---:|---:|---|---|
| Pigeon 2 primary | 23 | | Phoenix Tuner X | |
| Pigeon 2 secondary | unknown | | Phoenix Tuner X / physical | |
| 3rd hopper SPARK MAX | unknown | | REV Hardware Client | |
| Limelight | NT `limelight-knight` | | Limelight UI | |

## Electrical

| Mechanism | Controller ID | PDH/PDP Channel | Breaker A | Wire Gauge | Source |
|---|---:|---:|---:|---|---|
| FL drive | 11 | | | | |
| FR drive | 5 | | | | |
| BL drive | 2 | | | | |
| BR drive | 8 | | | | |
| Shooter A | 13 | | | | |
| Shooter B | 14 | | | | |
| Shooter C | 15 | | | | |
| Shooter D | 16 | | | | |
| Kicker | 17 | | | | |
| Intake roller | 18 | | | | |
| Hood | 19 | | | | |
| Hopper A | 20 | | | | |
| Hopper B | 21 | | | | |
| Intake slider | 22 | | | | |
| Hopper C | unknown | | | | |

## Geometry

| Measurement | Value | Units | Source |
|---|---:|---|---|
| Robot weight | | lb or kg | scale |
| Frame length | | in | tape |
| Frame width | | in | tape |
| Wheelbase | | in | tape |
| Track width | | in | tape |
| Bumper thickness | | in | tape |
| Shooter release height | | in | tape |
| Flywheel wheel diameter | | in | calipers |
| Ball compression gap | | in | calipers |
| Limelight lens height | | in | tape |
| Limelight forward offset from robot center | | in | tape |
| Limelight lateral offset from robot center | | in | tape |
| Limelight pitch | | deg | angle gauge |
| Limelight roll | | deg | angle gauge |

## Sensors

| Sensor | Port/ID | Installed | Active State | Source |
|---|---|---|---|---|
| Indexer beam-break A | DIO 8 expected | yes / no | blocked=true / blocked=false | |
| Indexer beam-break B | DIO 9 expected | yes / no | blocked=true / blocked=false | |
| Hood absolute encoder | SPARK MAX data port expected | yes / no | angle increases up / down | |

## Mechanism Ratios

| Mechanism | Observed Teeth / Cartridge Stack | Derived Ratio | Direction |
|---|---|---|---|
| Hood | | | motor rotations : hood degrees |
| Intake slider | | | motor rotations : slider inches |
| Intake roller | | | motor rotations : roller rotations |
| Hopper | | | motor rotations : roller rotations |
| Kicker | | | motor rotations : wheel rotations |

## Bring-Up Results

| Mechanism | Test Performed | Result | Follow-Up |
|---|---|---|---|
| Swerve drive | | | |
| Shooter | | | |
| Hood | | | |
| Intake | | | |
| Hopper | | | |
| Kicker | | | |
