# Helios 2026 — Robot Data (FRC Team 9704)

Everything currently known about the **Helios 2026** robot, gathered as reference for the **from-scratch code rewrite**. Nothing here is robot code — it's the hardware/config knowledge base the new code should be built from.

## ▶ START HERE (coding agent)

If you're the agent writing the new robot code, read in this order:
1. **[Hardware-Data-Sheet.md](Hardware-Data-Sheet.md)** — the authoritative reference. CAN map, motor models, gear ratios, current limits + brake/coast (§7), geometry, AprilTags (§3), and the open-items list (§11).
2. **[Rewrite-Checklist.md](Rewrite-Checklist.md)** — what must be a coded safe-default now (Tier 0) vs. tuned later vs. measured on the robot.
3. **[CAN-Bus-and-DIO-Map.md](CAN-Bus-and-DIO-Map.md)** — the device map to wire constants from.

Rules for the new code (also in the project `CLAUDE.md`):
- **Every motor gets a current limit + neutral mode from line one** — use the §7 *conservative-start* column; never the vendor default. (Old code had none → brownout/burnout.)
- Values tagged `[placeholder]`, `[CAD]`, or `[inferred]` are **not yet real** — write them as `// TODO` constants with units, don't trust them as final.
- **No climber this season.** The hopper has **3** motors (old code wired 2).
- Don't invent CAN IDs / ratios / offsets — if it's not here, it's in §11 as "needs the robot": leave a TODO and surface it.

## What's in this folder

| File | Contents |
|---|---|
| [Hardware-Data-Sheet.md](Hardware-Data-Sheet.md) | **The master document.** Full CAN map, per-subsystem hardware, gear ratios, current limits, drivetrain geometry, PathPlanner config, placeholders, gotchas, CAD-derived data (§10), and the open-questions list (§11). |
| [Rewrite-Checklist.md](Rewrite-Checklist.md) | **Action list** — everything still to confirm/decide before & during the rewrite, in priority tiers (safety → hardware → ratios → tuning → strategy). Bring it to the robot. |
| [Measurement-Worksheet.md](Measurement-Worksheet.md) | **Fill-in-the-blank sheet** of concrete values you can measure at the robot or read from labels/the game manual (slider stroke, shooter/vision geometry, CAN IDs, breaker sizes, AprilTag IDs). |
| [CAN-Bus-and-DIO-Map.md](CAN-Bus-and-DIO-Map.md) | Quick-reference: CAN IDs 1–23 + DIO ports, one table. Print this for the pit. |
| [CAD/Onshape-Reference.md](CAD/Onshape-Reference.md) | The Onshape document URLs, what each assembly contains, and how to re-pull data (quota-safe). |
| [CAD/cad-bom-digest.md](CAD/cad-bom-digest.md) | Parsed CAD BOM: motors, gearboxes/tooth counts, wheels, shaft spans, masses — the human-readable digest of the raw pulls. |
| [CAD/raw/](CAD/raw/) | Raw JSON from the Onshape pulls (shooter, intake, hopper, top-level mass + BOM). Primary source; full part lists. |

## How this data was gathered

1. **Old code** — mined from `2026-Helios-Code-main.zip` (the unfinished 2026 codebase). Extracted copy lives at `C:\FRC\extracted\2026-Helios-Code-main\`. Every code value was re-read and cross-checked against source.
2. **CAD** — pulled from the team's Onshape documents (shooter / intake / hopper sub-assemblies + top-level robot) via the `frc-robot-coder` skill's `onshape_fetch.py`. See [CAD/Onshape-Reference.md](CAD/Onshape-Reference.md).
3. **Robot photos** — *pending* (the team will provide pictures of the actual robot). These will resolve the remaining open items in §11 of the data sheet.

## Confidence legend (used throughout)

- `[verified-in-code]` — literally present in a source/config file of the old codebase.
- `[CAD]` — from the Onshape model. Reflects the design, **not** the wired robot — treat like an assumption until confirmed.
- `[inferred]` — derived/strongly implied, not literally stated.
- `[placeholder]` — a `0` / stub / unset value in the old code; **not a real number.**

## Key facts at a glance

- **Drivetrain:** swerve, 4 modules (WCP X3, 10T pinion). Drive Kraken X60 @ **6.48:1**, steer Kraken X44 @ **12.1:1**, wheel radius **2"**, max speed **4.93 m/s**. Wheelbase **26.75"** × track **16.75"**.
- **Motors (19 total):** 9× Kraken X60, 5× Kraken X44, 3× NEO 2.0, 1× NEO V1.1, 1× NEO 550. Plus 4 CANcoders, 2× Pigeon 2.0 (1 used), 1× Limelight 4.
- **Subsystems:** swerve · shooter (4 flywheel Kraken + NEO 550 hood) · hopper (3× NEO 2.0 — code wires only 2) · intake (Kraken X44 roller + NEO V1.1 slider) · kicker (Kraken X60).
- **No climber this season.**
- **Mass:** PathPlanner has 63.5 kg / MOI 7.68 kg·m² (hand-entered). CAD masses are unreliable (unassigned materials) — weigh the robot.
- **⚠ Biggest carryover risk:** the old code has **no current limits on any mechanism motor** (only swerve steer @ 40 A). The rewrite must add limits everywhere — see §7 of the data sheet.

## Open questions (full list in Hardware-Data-Sheet.md §11)

Hood total ratio (code 187.5:1 vs ~18:1 readable — missing sector stage), 3rd hopper motor CAN ID/role, intake slider spur stage, kicker ratio, 2nd Pigeon, all current limits, brake/coast modes, CANcoder offsets, sensor polarity, motor inversions. **Robot photos will close most of these.**

---
*Compiled by Team 9704 with Claude Code. Sources: old codebase + Onshape CAD. Not verified against the physical robot.*
