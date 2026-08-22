# Helios 2026 — "Everything We Need to Know" Checklist

Master list of every fact still to confirm and every decision still to make before/while writing the new code. Pulled from the data sheet (§7–§11), the CAD dig, and the old-code review.

**Legend:** ⛔ = safety blocker (resolve before the motor runs under power) · 📷 = a robot photo / quick look at the robot answers it · 🎯 = a strategy/tuning decision the team makes (not a "lookup") · ✅ = already known (listed for boundary, no action).

---

## Tier 0 — Safety (do NOT run a motor without these)

- [ ] ⛔ **Current limit on every motor** — **3 tiers (conservative-start / regular / cap) in Hardware-Data-Sheet §7.** Team preference: **start at the conservative numbers**, raise toward regular only if a mechanism is underpowered, never pass the cap. Old code limits nothing except swerve steer (40 A). Conservative starting values (`St`=stator/`Sup`=supply, A):
  - [ ] 4× drive Kraken X60 (2,5,8,11) — St 40 / Sup 30  *(reg 120/60 · cap 120/70)*
  - [ ] 4× flywheel Kraken X60 (13–16) — St 40 / Sup 25  *(reg 80/30 · cap 120/40)* — #1 brownout risk
  - [ ] kicker Kraken X60 (17) — St 30 / Sup 20  *(reg 40/20 · cap 80/40)*
  - [ ] intake roller Kraken X44 (18) — St 30 / Sup 20  *(reg 40/25 · cap 60/40)*
  - [ ] hood **NEO 550** (19) — `smartCurrentLimit` **20**  *(cap 30 — fragile)* ← highest burnout risk; today no config
  - [ ] 3× hopper NEO 2.0 (20, 21, ?) — `smartCurrentLimit` **20**  *(reg 40 · cap 60)*
  - [ ] intake slider NEO V1.1 (22) — `smartCurrentLimit` **20**  *(reg 30–40 · cap 60)*
- [ ] ⛔ **Neutral (brake/coast) mode per mechanism** — it's a *design choice*, not a lookup (decided, see §7): **Brake** = drive, steer, hood, slider (must hold); **Coast** = flywheels (regen!), kicker, rollers. Default is Coast, so brake motors must be set explicitly. API: TalonFX `NeutralModeValue.Brake`, SPARK MAX `IdleMode.kBrake`.
- [ ] ⛔ **Bound every open-loop move with a timeout / limit.** Old `stowSliderCommand` runs the slider forever (no timeout) → stall. Every "run until X" needs a timeout + measured-current stall check (`getOutputCurrent()` / `getStatorCurrent()`, debounced — never `.get()`).
- [ ] ⛔ **Hood travel limits enforced in software** (MIN 3.224° / MAX 44.5° today — confirm real range) so the NEO 550 can't pin against a hard stop.

---

## Tier 1 — Hardware facts to confirm on the robot

### CAN bus / device count
- [ ] 📷 Confirm every CAN ID 1–23 matches the physical bus (see [CAN-Bus-and-DIO-Map.md](CAN-Bus-and-DIO-Map.md)).
- [ ] 📷 **3rd hopper motor** — CAD shows 3 NEO 2.0 + 5 SPARK MAX, code wires only 2. Does it exist? What CAN ID (assign 24+)? Is it a CAN follower of A/B or independent?
- [ ] 📷 **2nd Pigeon 2.0** — CAD shows two; code uses one (CAN 23). Real/redundant? Its CAN ID, or drop it.
- [ ] ✅ Limelight 4, NT name `limelight-knight` (confirmed).
- [ ] CAN bus choice: stays on the default roboRIO bus (CAN 2.0, 100 Hz odometry) or move swerve to a CANivore/CAN FD? Affects bus name + odometry rate.

### Motor models & polarity
- [ ] 📷 Confirm CAD-resolved motor models on the real robot: drive **Kraken X60**, steer **Kraken X44**, flywheel/kicker **Kraken X60**, intake roller **Kraken X44**, hood **NEO 550**, hopper **NEO 2.0**, slider **NEO V1.1**.
- [ ] 📷 **Every motor inversion / direction** — CAD gives geometry, never electrical direction. Verify each mechanism spins the right way (drive, steer, all 4 flywheels + follower alignments, kicker, intake roller, slider, 3 hopper, hood).

### Sensors
- [ ] 📷 **Indexer beam-breaks on DIO 8/9** — actually wire/construct `DigitalInput(8)`/`DigitalInput(9)` (old `Sensors.java` is a stub returning `true`). Confirm active-high vs active-low.
- [ ] 📷 **Hood absolute encoder zero** — confirm the `SHOOTER_ANGLE_OFFSET = 63` and re-zero on the robot; verify which direction increases angle.
- [ ] Decide fuel-detection logic (which sensor gates indexing vs. kicking).

---

## Tier 2 — Gear ratios & drivetrain geometry to finalize

- [x] ✅ **Hood ratio RESOLVED: 187.5:1** = UltraPlanetary 2× 5:1 (25:1) × 24T→180T sector (7.5:1). Team confirmed the 5:1 cartridges; CAD BOM's "3:1" label was wrong.
- [ ] 📷 **Intake roller full chain** — 12T→24T (2:1) motor stage known; confirm downstream routing + final roller surface speed.
- [ ] 📷 **Intake slider — travel type & distance** — MAXPlanetary 5:1 + a 10T/20T (10 DP) stage seen in CAD, but **rack-vs-geared and inches-per-motor-rotation can't be read from CAD**. Measure on the robot (code's slider constants are all 0).
- [ ] 📷 **Kicker** ratio — not derivable from BOM.
- [ ] 📷 **Hopper** — 10T→60T (6:1) + 18T→36T (2:1) known; confirm which feeds the 2" rollers and the effective roller diameter.
- [ ] ✅ Flywheel **1.5:1 overdrive** (36T→24T), 2" radius — confirmed (code + CAD).
- [ ] ✅ Swerve: drive 6.48:1, steer 12.1:1, couple 5.4, wheel radius 2" — confirmed.
- [ ] **CANcoder magnet offsets** — re-zero all 4 on the robot (three config sources disagree; never trust the stored numbers).
- [ ] **Pigeon mount-pose** — `pigeonConfigs = null` today; set mount orientation.
- [ ] Confirm/tune `kSpeedAt12Volts = 4.93 m/s`, `kSlipCurrent = 120 A`, `kCoupleRatio = 5.4` (Tuner says "tune to your robot").
- [ ] Confirm wheelbase 26.75" × track 16.75" (derived from module positions) and reconcile the three disagreeing swerve sources (`TunerConstants` vs `tuner-project.json` vs PathPlanner `settings.json` — different stator limits, inverts, offsets).
- [ ] Derive the hood's `0.0317428` deg-conversion factor correctly (it's unexplained in old code).

---

## Tier 3 — Tuning values to fill (every placeholder `0` in the old code)

### PID / feedforward (tune on robot)
- [ ] Swerve **drive** (old: kP 0.01, kV 0.1165; kS/kD = 0) and **steer** (kP 15; kI/kD/kS/kV = 0)
- [ ] Shooter **flywheel** velocity (old: kP 0.4, kD 0.01, kV 0.12625; kS/kA/kI = 0)
- [ ] Shooter **hood** angle (old: kP 0.275; kI/kD = 0; no gravity/FF on an angled hood)
- [ ] Intake **slider** position (old: all 0) — needs the slider ratio first
- [ ] 🎯 If hopper/kicker/intake-roller move to closed loop, their gains too

### Setpoints / constants
- [ ] 🎯 Shooter: `SHOOTER_HIGH_SPEED` (old 25 — units? confirm), `SHOOTER_LOW_SPEED` (0), `DISTANCE_SHORT`/`DISTANCE_FAR` (0), tolerances (speed 0.3, angle 0.5)
- [ ] 🎯 Intake slider macros: `INTAKE/OUTTAKE/STOW_SLIDER_INCHES` (all 0)
- [ ] 🎯 Confirm duty-cycle speeds: intake/outtake 0.25, hopper 0.8, indexer 0.75, slider ±0.5 / −0.25
- [ ] Hood angle limits (MIN 3.224° / MAX 44.5°) — confirm against real hardstops

### Vision
- [x] 🎯 **Distance → (flywheel speed, hood angle) model** — CODE DONE 2026-07-17: per-class `InterpolatingDoubleTreeMap` tables (SCORE flattens 44.5→31.5° over 3.0–6.2 m banking off the hub's back net; FEED 40→33° over 4–9 m), seeded from drag ballistics against the official GE-26300 hub drawings. Derivation + generator + tuning order: `Helios-2026/docs/shot-model/`. Still needs ON-ROBOT tuning (`SHOT_SPEED_SCALE` first).
- [ ] **Real AprilTag fiducial IDs** — old code duplicates hub IDs for the trench (RED 7/7, BLUE 8/8 — copy-paste bug). Get the actual 2026 field tag IDs.
- [ ] Limelight pipeline setup; decide on MegaTag2 `addVisionMeasurement` (currently commented out in the drivetrain) + vision std-devs.
- [ ] Confirm/tune LL X/Y/Z multipliers (old all 1).

### PathPlanner / autos
- [ ] **`PP_MAX_VELOCITY/_ACCELERATION/_ANGULAR_VELOCITY/_ANGULAR_ACCELERATION`** — all 0 today (any path built from these won't move).
- [ ] 🎯 Build autonomous routines — `getAutonomousCommand()` returns `Commands.none()` today.
- [ ] Bumper offsets (0/0 assumed centered) — confirm.

---

## Tier 4 — Physical / mass

- [ ] **Weigh the robot** — every CAD mass is density-defaulted/unreliable (top-level reads an absurd 680 kg). PathPlanner's hand-entered **63.5 kg / MOI 7.68 kg·m²** is the current best guess; replace with a real weight.
- [ ] Confirm frame size (0.61 × 0.978 m) and bumper dimensions.
- [ ] 🎯 If using physics sim, per-mechanism MOI (flywheel, hood, slider) — CAD values are low-confidence (assign materials in CAD or measure spin-up).

---

## Tier 5 — Behavior / strategy decisions (team chooses)

- [ ] 🎯 Controller layout / button mapping for the new code (old: single Xbox on port 0).
- [ ] 🎯 Drive feel: translation scaling (old 50%), rotation rate (old 0.75 rot/s), deadbands (10%), field- vs robot-centric default.
- [ ] 🎯 Shooter sequencing: when does the kicker fire (old: only on `desiredVelReached`, not on fuel sensor) — define the real ready-to-shoot condition.
- [ ] 🎯 Intake/hopper state machine + fuel handling for the new design.
- [ ] 🎯 Whether to restore the commented-out auto-align (`rotateToAngle`) and how it picks the target tag.

---

## Already locked (no action — reference only)

- ✅ Software stack: WPILib/GradleRIO **2026.2.1**, Phoenix6 **26.1.0**, REVLib **2026.0.3**, PathPlanner **2026.1.2**, Java 17, team **9704**.
- ✅ Full CAN map (IDs 1–23) and motor models (CAD-confirmed).
- ✅ Motor inventory: 9× Kraken X60, 5× Kraken X44, 3× NEO 2.0, 1× NEO V1.1, 1× NEO 550; 4 CANcoders, 1–2 Pigeon 2, 1 Limelight 4.
- ✅ Swerve ratios/geometry, flywheel 1.5:1, hood sector 7.5:1.
- ✅ **No climber this season.**

---
*Anything marked 📷 is a candidate for the upcoming robot photos. Bring this list to the robot.*
