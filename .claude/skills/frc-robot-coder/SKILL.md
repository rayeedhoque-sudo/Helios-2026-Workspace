---
name: frc-robot-coder
description: Use when working on FRC (FIRST Robotics Competition) robot code or a WPILib/GradleRIO Java project — when the user shares robot photos or hardware specs (motors, motor controllers, CAN IDs, gear ratios), mentions subsystems like swerve, shooter, intake, or hopper, names hardware or libraries like TalonFX, Kraken, Falcon, SPARK MAX, NEO, roboRIO, Limelight, CTRE Phoenix, REVLib, or PathPlanner, reports robot problems like brownouts, power issues, stalling, or motors misbehaving, or shares an Onshape CAD link / asks to read gear ratios, wheel dimensions, BOM, or mass properties (moments of inertia) from CAD (Onshape).
---

# FRC Robot Coder

Turn robot photos + hardware specs into working, safe WPILib robot code. Home team: FRC 9704 "Helios" — project at `C:\Users\Rayeed Hoque\FRC\2026-Helios-Code` (Java 17, WPILib 2026, Phoenix 6 26.1.0, REVLib 2026.0.3, PathPlanner). Read [team-conventions.md](team-conventions.md) before touching that repo; the rest of this skill applies to any FRC Java project.

## Workflow

1. **Gather** —
   - **1a. Photos** — Read photos directly (the Read tool renders images). Identify hardware per [hardware-identification.md](hardware-identification.md); read labels/markings (OCR, §1.5) for candidate ratios, CAN IDs, model numbers, and wire gauges — all `[assumed]`. For whole-robot / wiring / electrical-board shots use [wiring-and-robot-photos.md](wiring-and-robot-photos.md). Photos yield *candidate* labels only — never authoritative CAN IDs, gear ratios, wiring, or sensor polarity.
   - **1b. CAD** — If the user has an Onshape document, extract gear ratios / dimensions / BOM / mass properties per [onshape-integration.md](onshape-integration.md) (run `scripts/onshape_fetch.py`). CAD-sourced values are `[CAD]` and cross-checked before deploy. Skip swerve (use Tuner X). Mind the annual API quota — cache, prefer version reads.
2. **Interview** — fill the gaps one question at a time, per the interview script in [robot-spec-template.md](robot-spec-template.md). If the user can't answer right now, make the conservative engineering choice, mark it `[assumed]`, and leave a TODO (rule 5) — don't block on an unanswered question.
3. **Spec before code** — create/update `docs/robot-spec.md` in the robot project from the template. Every value is marked `[verified]`, `[CAD]`, or `[assumed]` (CAD-sourced values are `[CAD]` — see [onshape-integration.md](onshape-integration.md)). Code built on `[assumed]` electrical facts must say so out loud.
4. **Code** — follow [team-conventions.md](team-conventions.md). Vendor API shapes come ONLY from [phoenix6-patterns.md](phoenix6-patterns.md) and [revlib-patterns.md](revlib-patterns.md) — see hard rule 4. Safety defaults from [power-and-safety-checklist.md](power-and-safety-checklist.md).
5. **Verify** — `gradlew build` after every change, per [build-and-verify.md](build-and-verify.md); simulation or unit tests when behavior matters, not just compilation.
6. **Attribute** — FIRST officially permits AI assistance with credit (FIRST policy, Nov 2023). Note it in commits/docs: *"Developed by Team 9704 with Claude Code."*

## Hard rules

1. **NEVER configure a motor without current limits.** A stalled NEO 550 at the SPARK MAX 80 A default destroys itself in ~2 s (REV locked-rotor data); unlimited Krakens brown out the robot. Limits per mechanism: see the pattern files.
2. **NEVER use `motor.get()` or `getAppliedOutput()` for stall/jam detection** — those are *commanded* output. Measured signals: `getOutputCurrent()` (REV), `getStatorCurrent()` (CTRE), debounced.
3. **NEVER invent CAN IDs, ports, gear ratios, polarities, or directions.** Interview the user or read existing code. Gear ratios and dimensions MAY also come from Onshape CAD (marked `[CAD]`, cross-checked before deploy — see [onshape-integration.md](onshape-integration.md)) or from a photo-read label (marked `[assumed]`); a CAD value is a sanctioned source, not an invention, but is never `[verified]` until matched against code or confirmed by the user. Inventing a value bricks a mechanism or worse.
4. **NEVER emit vendor APIs from memory.** WPILib 2026 / Phoenix 6 v26 / REVLib 2026 broke training-data APIs (`Follower` now takes `MotorAlignmentValue`; `CANSparkMax`/`burnFlash()` no longer exist). The pattern files are compile-verified against this project's jars — when going beyond them, prove the API with a build before presenting it.
5. **Placeholder values get `// TODO` + units** and are listed back to the user when work ends.
6. **A passing build is the floor, not the proof.** Sim ≠ hardware; end every task by listing what still needs on-robot verification.

## Limits

- `gradlew deploy` needs the robot's network — from this machine: build + simulate only.
- PID gains and current-limit values in the references are starting points to tune on the robot, not final answers.
- Onshape API calls draw on a small annual quota (~2,500/yr); the helper caches, and the screenshot fallback in [onshape-integration.md](onshape-integration.md) covers quota-exhausted / no-keys situations.
