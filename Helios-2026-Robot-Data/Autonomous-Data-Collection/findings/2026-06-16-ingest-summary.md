# 2026-06-16 Ingest Summary — Robot Photos (orientation / model spot-check)

Source: 5 photos in `C:\FRC\Helios-2026-Robot-Data\Photos\` (note: not in the expected
`Autonomous-Data-Collection\inbox\photos\` — consider moving them there to match the folder
convention). Reviewed by Claude Code for the drivetrain/swerve calibration prep. All photo-derived
facts are **`[photo-assumed]` = candidate only**, per the folder Safety Rule: do NOT convert these
into deployed motor directions, CAN IDs, current limits, sensor polarity, or encoder offsets without
confirmation from code, vendor-tool readback, or a robot test.

## New Evidence

| File | Source Type | Summary |
|---|---|---|
| `Photos\IMG_2883.jpg` | `[photo-assumed]` | Clear **"KRAKEN X44"** label on a motor belt-driving a mecanum/compliant roller (purple spacers) — consistent with the **intake roller** assembly. A treaded wheel with a "TALON FX" label sits below. |
| `Photos\IMG_2882.jpg` | `[photo-assumed]` | Whole-robot frame on its side: wiring harness + battery, **wheels at the corners carrying CTRE TalonFX labels** — swerve drive corners are TalonFX (Kraken). Module corners visible but not square-on. |
| `Photos\IMG_2880.jpg` | `[photo-assumed]` | A **NEO 2.0** (REV-21-1653) on a SPARK MAX mounted to a gearbox plate with a **spur-gear stage**, on a bench (ruler present). Hopper drive module. |
| `Photos\IMG_2881.jpg` | `[photo-assumed]` | A **NEO 2.0** mounted in-frame behind a triangulated (lightened) structural plate, with wiring. Hopper area. |
| `Photos\IMG_2876.jpg` | `[photo-assumed]` | Interior shooter/hopper tower with timing belts and stacked wheels; a **yellow ball game piece ("fuel")** resting in the mechanism. |

## Confirmed Facts (candidate-level, corroborating existing data)

| Fact | Value | Evidence Tag | Source |
|---|---|---|---|
| Intake roller motor | Kraken X44 (visual label) | `[photo-assumed]` | IMG_2883 |
| Swerve drive at corners | CTRE TalonFX (Kraken) | `[photo-assumed]` | IMG_2882 |
| Hopper motors | NEO 2.0 on SPARK MAX, with spur-gear reduction | `[photo-assumed]` | IMG_2880, IMG_2881 |
| Intake roller wheels | mecanum / compliant | `[photo-assumed]` | IMG_2883 |
| Game piece | yellow ball | `[photo-assumed]` | IMG_2876 |

## Conflicts

| Existing Value | New Value | Likely Resolution | Source |
|---|---|---|---|
| (none) | — | All photo evidence **corroborates** the existing Hardware-Data-Sheet / CAD; no conflicts found. | — |

## Code Impact

| Area | Required Change | Risk |
|---|---|---|
| Swerve `TunerConstants` (offsets/inversions) | **None from photos.** CANcoder zero offsets are an electrical reading not visible in a photo; module bevel/inversion not square-on enough to call. Still require Tuner X readback + low-speed spin test on the robot. | Low — no change made |
| Data-sheet confidence tags (optional) | The "Kraken X44" model (was `[inferred]`) could be upgraded to also carry `[photo-assumed]` for the intake roller; same for TalonFX drive / NEO 2.0 hopper. Not done here to avoid scope creep. | Low |

## Still Unknown (photos cannot resolve)

| Question | Best Next Data Source |
|---|---|
| Per-module swerve bevel direction → drive/steer inversions | Square-on per-module photo (wheel forward, bevel visible) for a *candidate* list; **confirm by spin test** on blocks |
| 4× CANcoder zero offsets | Phoenix Tuner X readback with wheels physically squared |
| Which corner = which CAN ID (FL/FR/BL/BR mapping) | Tuner X / spin test |
| 3rd hopper NEO 2.0 CAN ID + role | Tuner X device list on the powered robot |
| Pigeon 2 mount orientation (yaw) | Physical mounting inspection / Tuner X mount calibration |

---
*Logged by Team 9704 with Claude Code. Photo facts are candidate-only until confirmed on the robot.*
