*Identify FRC hardware from photos/spec sheets, map it to the right vendor library, and know which facts photos can never provide.*

## 1. Visual identification cues (photo triage)

| Device | Cues in a photo | Easy confusion |
|---|---|---|
| Kraken X60 | Black cylindrical brushless motor, "KRAKEN" branding (WestCoast Products), CAN pigtail (yellow/green) + power leads exit the motor directly — controller is integrated (TalonFX) | Falcon 500 (also integrated TalonFX) |
| Kraken X44 | Same family, visibly smaller/compact 44 mm-class body | Kraken X60 |
| Falcon 500 | Silver/gray body, teal-green end accents, VEX/CTRE branding, integrated TalonFX | Kraken X60 |
| NEO (v1.1) | Black ~60 mm-class cylinder, 3 thick phase wires + thin white JST encoder pigtail; NO integrated controller | Kraken (Kraken has CAN wires, NEO has phase wires) |
| NEO 550 | Much smaller (550-size) black/orange motor, same phase + JST wiring | NEO; 775pro (775 is brushed, only 2 power wires) |
| NEO Vortex | NEO-family motor with a dock interface; a SPARK Flex mounts directly onto it | NEO |
| SPARK MAX | Small black/orange rectangular controller, USB-C, data port, CAN pigtail | SPARK Flex (Flex is larger, docks onto Vortex) |
| SPARK Flex | Larger REV controller, docks to NEO Vortex (can also drive other motors via adapter) | SPARK MAX |
| CANcoder | Small black CTRE puck mounted over a rotating shaft magnet; CAN wires only | Through-bore encoder (that one has a hex bore + ribbon/DIO cable) |
| Pigeon 2 | Flat small rectangular CTRE IMU board/case, CAN wires, mounted flat near robot center | CANcoder |
| roboRIO 1 vs 2 | Both gray/blue NI bricks; roboRIO 2 boots from a microSD card (visible card slot) — model name is printed on the device, read it | — |
| PDH (REV) | Long black/orange REV panel: 20 high-current (40 A max) + 3 low-current (15 A) + 1 switchable low-current channel, CAN | PDP |
| PDP (CTRE) | Squarer black/red CTRE panel: 16 channels (8x 40 A + 8x 30 A pairs) | PDH |
| Limelight 2/3/3G/4 | Green-LED camera in a printed enclosure; model name is printed on the case/back — have the user read the label rather than guessing from shape | Generic USB cam (no LED array, no enclosure branding) |
| Beam-break sensor | Small emitter+receiver pair (often barrel style), thin 3-wire leads to roboRIO DIO | Limit switch (mechanical lever, 2–3 wires) |
| REV Through Bore Encoder | Black puck with 1/2" hex bore through the center; outputs absolute (duty-cycle) and/or quadrature | CANcoder (no bore, CAN wires) |

Rules of thumb: CAN pigtail straight out of the motor body = integrated TalonFX (Phoenix 6). Three phase wires + JST = REV brushless, needs SPARK MAX/Flex. Two power wires only = brushed motor. If a label/sticker is visible but unreadable, ask the user to read it — never guess model from silhouette alone.

PDH/PDP channel counts and SPARK MAX/Flex roles per https://docs.wpilib.org/en/stable/docs/controls-overviews/control-system-hardware.html ; roboRIO 2 microSD boot per https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/roborio2-imaging.html . Other visual cues (colors, accents, sizes) are physical-hardware knowledge, not doc-verifiable — treat as hints, confirm via label.

For whole-robot, electrical-board, or wiring-harness photos (mapping what's connected to what, PDH/PDP channels, CAN topology), use [wiring-and-robot-photos.md](wiring-and-robot-photos.md).

## 1.5 Reading labels and markings (OCR)

A photo can *read* a printed value, but it can also *misread* it. Every value read off a label is `[assumed]`, never `[verified]` — OCR produces a **candidate** the user (or the code/CAD) confirms; it does not upgrade provenance. Reading a label is a first pass, not a replacement for asking.

| Marking | Where it appears | What it gives | How to treat it |
|---|---|---|---|
| Ratio sticker — `4:1`, `9:1`, `L2`, `5.50:1` | gearbox housing, swerve module | candidate reduction | `[assumed]` — confirm against CAD (`[CAD]`) or code before use |
| CAN ID label (heat-shrink / tape numbers) | motor leads, controller | candidate CAN ID | `[assumed]` — **NEVER** write code from a photo-read CAN ID |
| Motor model — `KRAKEN X60`, `NEO 550` | motor body | model family | narrows the guess; still confirm the exact model |
| Wire gauge — `12 AWG`, `10 AWG` | wire insulation print | breaker-sizing sanity (see [power-and-safety-checklist.md](power-and-safety-checklist.md)) | `[assumed]` |
| Breaker rating — `40A` | Snap-Action breaker top | channel breaker size | `[assumed]` |
| Encoder / sensor part number | sensor body | device family | confirm wiring/port with the user |

If a label is partially legible or ambiguous, **ask the user to read it** rather than guessing from a blurry crop. A photo-read ratio feeds the spec's "Gear ratios / kinematics" as `[assumed]`; if an Onshape `[CAD]` value also exists, two independent sources agreeing is stronger evidence — but still confirm before deploy.

## 1.6 COTS gearboxes & swerve modules (recognition)

The **reduction of a COTS gearbox is not in a photo** — the photo shows the housing exists, never its ratio. Recognize the *family*, read the config-code sticker if visible, then look the ratio up in [cots-ratio-reference.md](cots-ratio-reference.md). Do not infer a ratio from size or stage count.

| Visual cue | COTS family | Action | Easy confusion |
|---|---|---|---|
| Stacked round cartridges, REV black/orange, hex output | REV **MAXPlanetary** | read cartridge stack → net ratio = product (`cots-ratio-reference.md`) | VersaPlanetary (VEX, ring-clamped stages) |
| Ring-clamped round stages, VEX | VEXpro **VersaPlanetary** | read stage stack → net ratio = product | MAXPlanetary |
| Square swerve module, drive motor beside steer, billet frame | SDS **MK4 / MK4i / MK4c / MK4n** | read `L1/L2/L3/L4` code → ratio table; **MK4i** = steer motor on top/inverted | MK4 vs MK4i = steer-motor position |
| Compact module, NEO/Vortex on top, REV branding | REV **MAXSwerve** | read drive pinion `12T/13T/14T` → ratio | SDS modules (MAXSwerve is more compact, single-vendor) |
| Swerve module, WCP branding, X/X2/XS family | WCP **Swerve X** | hand-enter from WCP ratio table (not embedded) | SDS MK4 family |
| Inline planetary on a Falcon/CIM, VEX | VersaPlanetary on a motor | as above | a bare gearbox vs an integrated motor+gearbox |

Connector / wiring cues that help disambiguate hardware: Anderson **SB50** (large battery/main) vs **PowerPole PP15-45** (motor leads); **JST PH/XH** (REV encoder/sensor pigtails); CAN **Weidmuller/screw** terminals vs a CAN **pigtail** straight from a motor (integrated TalonFX). 

**Swerve note:** even when you recognize the module and its ratio, swerve gear ratios are normally taken from CTRE Tuner X / YAGSL, not hand-entered — see [cots-ratio-reference.md](cots-ratio-reference.md) and [onshape-integration.md](onshape-integration.md).

## 2. Decisive mapping: device → controller → library → use

| Motor / device | Controller | Vendor lib (Helios versions) | Java class | Typical use |
|---|---|---|---|---|
| Kraken X60 / X44 / Falcon 500 | Integrated TalonFX | Phoenix 6 (26.1.0) | `com.ctre.phoenix6.hardware.TalonFX` | Swerve drive/steer, flywheels, high-power mechanisms |
| NEO / NEO 550 | SPARK MAX | REVLib (2026.0.3) | `com.revrobotics.spark.SparkMax` (`MotorType.kBrushless`) | Hoods, hoppers, sliders, low/mid-power |
| NEO Vortex | SPARK Flex | REVLib (2026.0.3) | `com.revrobotics.spark.SparkFlex` | Mid/high-power mechanisms |
| Brushed (CIM, 775pro) | SPARK MAX brushed mode (`MotorType.kBrushed`) or legacy Talon SRX (Phoenix 5 — NOT in this project) | REVLib | `SparkMax` | Legacy mechanisms |
| CANcoder | — (CAN sensor) | Phoenix 6 | `com.ctre.phoenix6.hardware.CANcoder` | Swerve azimuth absolute angle |
| Pigeon 2 | — (CAN IMU) | Phoenix 6 | `com.ctre.phoenix6.hardware.Pigeon2` | Robot heading (created inside CTRE `SwerveDrivetrain`) |
| Beam break / limit switch | — (DIO) | WPILib | `edu.wpi.first.wpilibj.DigitalInput` | Game-piece detection |
| REV Through Bore (absolute) | SPARK MAX data port or roboRIO DIO | REVLib (`spark.getAbsoluteEncoder()` → `SparkAbsoluteEncoder`) or WPILib `edu.wpi.first.wpilibj.DutyCycleEncoder` (https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html) | — | Arm/hood absolute position |
| Limelight | — (Ethernet/NT) | none (`LimelightHelpers.java` vendored in project, name-based API) | — | AprilTag targeting |

Constructor forms grounded in project source (`src\main\java\frc\robot\subsystems\misc\ShooterSubsystem.java`):

```java
TalonFX motor = new TalonFX(13);                       // CAN ID 13, default (roboRIO) bus
SparkMax hood = new SparkMax(19, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
SparkAbsoluteEncoder enc = hood.getAbsoluteEncoder();  // absolute encoder on SparkMax data port
// Phoenix 6 v26 Follower takes MotorAlignmentValue, NOT a boolean (API drift!):
shooterB.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Aligned));
shooterC.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed));
```

REVLib config pattern (per https://docs.revrobotics.com/revlib/spark/configuring-a-spark):

```java
SparkMaxConfig config = new SparkMaxConfig();
config.smartCurrentLimit(20);                          // Amps — NEO 550 burns out fast without this
spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

## 3. Helios ground truth (what this robot actually has)

From `C:\Users\Rayeed Hoque\FRC\2026-Helios-Code\src\main\java\frc\robot\Constants\SubsystemConstants.java` and `TunerConstants.java`:

| Mechanism | Hardware | CAN/DIO IDs |
|---|---|---|
| Swerve (4 modules) | 8x TalonFX (drive+steer), 4x CANcoder, FusedCANcoder feedback (falls back to RemoteCANcoder without Pro license — TunerConstants comment) | steer/drive/enc: FL 10/11/12, FR 4/5/6, BL 1/2/3, BR 7/8/9 |
| Gyro | Pigeon 2 | 23 |
| Shooter flywheel | 4x TalonFX Kraken (A leader; B Aligned; C, D Opposed followers), VelocityVoltage + Slot0 | 13, 14, 15, 16 |
| Shooter hood | SparkMax + NEO 550, absolute encoder, 187.5:1 | 19 |
| Intake | TalonFX roller; SparkMax brushless slider | 18; 22 |
| Hopper | 2x SparkMax brushless; TalonFX kicker | 20, 21; 17 |
| Indexer sensors | DIO light/beam sensors (`Sensors.java` is a stub: never constructs `DigitalInput`, returns hardcoded `true`) | DIO 8, 9 (constants only) |
| Vision | Limelight named `"limelight-knight"` (model NOT determinable from code — ask) | — |

Current-limit status (brownout-relevant): swerve IS limited via `TunerConstants.java` — steer 40 A stator limit (`CurrentLimitsConfigs`), drive slip current 120 A (`withSlipCurrent`). Every mechanism motor (TalonFX 13–18, all SparkMaxes) has NO current limit configured anywhere in the source. Also note: `IntakeSubsystem.isIntakeSliderStall()` reads `motor.get()` (commanded duty cycle), not actual current — it cannot detect a real stall. Real current on a SparkMax is `getOutputCurrent()` (VERIFY-BY-COMPILE: not exercised in this project's source; REV's latest javadoc shows a newer signals API where it returns `Signal<Double>` and `get()` is gone — that javadoc does NOT match this project's REVLib 2026.0.3, where `get()` compiles, so expect plain `double` but confirm by compiling).

## 4. What photos can NEVER tell you — MUST come from the user

Never invent these. If any are missing for the mechanism being coded, stop and ask — **or** source them properly: a printed label read off a photo (§1.5) is a candidate `[assumed]` value, and an Onshape CAD model gives a `[CAD]` value (gear ratios, dimensions, mass properties — see [onshape-integration.md](onshape-integration.md)). Both are sanctioned sources to be confirmed against code/hardware, not inventions. The list below is still never *guessed*:

- **CAN IDs** (and which CAN bus — roboRIO vs CANivore; Helios uses the default bus, `new CANBus("", ...)` in TunerConstants)
- **Gear ratios** (motor rotations per mechanism rotation/inch) — a photo shows a gearbox exists, never its reduction
- **Which direction is positive** / which motors are inverted / leader-vs-follower assignment and alignment
- **Soft limits / range of motion** (min/max angle, travel in inches) and hard-stop locations
- **Absolute encoder offsets/zero position** (e.g. Helios `SHOOTER_ANGLE_OFFSET = 63`)
- **Wiring**: which DIO port each sensor is on, which PDH/PDP channel and breaker size each motor has
- **Current limits desired** and whether brownouts have been observed (Helios: yes — known pain)
- **Wheel/pulley/sprocket diameters** for converting rotations to linear units
- **Camera name + mounting pose** (Limelight name strings are software config, not visible)
- Whether a mechanism is position-, velocity-, or duty-cycle-controlled by design intent

## 5. Spec questions to ask, per mechanism type

| Mechanism | Ask for |
|---|---|
| Flywheel/shooter | Motor count + CAN IDs, leader + follower alignment, gear ratio (motor:wheel), wheel diameter, target surface speed or RPM, tolerance |
| Arm / pivot / hood | CAN ID, gear ratio, absolute encoder? (type + where plugged in), zero offset, min/max angle, which direction is positive, gravity-loaded? |
| Elevator / slider | CAN ID(s), gear ratio + drum/pulley diameter (rotations→inches), travel limits, limit switches or stall-detect homing?, brake mode needed |
| Intake roller | CAN ID, intake vs outtake direction sign, duty cycle vs velocity control, current limit (rollers stall on game pieces — limit is near-mandatory) |
| Swerve | Prefer regenerating via Tuner X; otherwise module IDs, encoder offsets, gear ratios, wheel radius, track dimensions — never hand-derive these |
| Climber | CAN IDs, ratchet/brake?, travel limits, manual-speed expectations |
| Sensors | DIO port numbers, NC vs NO logic (does `get()` return true when blocked or clear?) |
| Any motor | Breaker size on PDH/PDP channel and desired current limit (Amps) — team has brownout history, so always ask |
