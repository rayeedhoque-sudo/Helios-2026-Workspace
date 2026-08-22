# Helios 2026 — CAD BOM Digest

Human-readable digest of the Onshape pulls in [raw/](raw/). Tooth counts, wheel diameters, and shaft lengths read directly from the BOMs; derived ratios noted. All `[CAD]` — reflects the design, not the wired robot.

## Motors & controllers (top-level robot BOM)

| Item | Qty (CAD) | Part # | Used by |
|---|---|---|---|
| Kraken X60 Brushless Motor | 5 | WCP-0940 | shooter: 4 flywheel + 1 kicker (swerve drive Krakens not in this assembly) |
| Kraken X44 Brushless Motor | 1 | WCP-0941 | intake roller (swerve steer X44s not in this assembly) |
| NEO 2.0 Brushless Motor | 3 | REV-21-1653 | hopper (code wires only 2) |
| NEO V1.1 Brushless Motor | 1 | REV-21-1650 | intake slider |
| NEO 550 Brushless Motor | 1 | REV-21-1651 | shooter hood |
| SPARK MAX Controller | 5 | REV-11-2158 | 3 hopper + 1 slider + 1 hood (code constructs only 4) |
| Pigeon 2.0 | 2 | — | code uses 1 (CAN 23) |
| Limelight 4 Camera | 1 | — | vision `limelight-knight` |

> Full robot incl. swerve (from Tuner X), per CAD BOM: **9× Kraken X60, 5× Kraken X44, 3× NEO 2.0, 1× NEO V1.1, 1× NEO 550** = 19 motors.
>
> **⚠ Physical robot differs from this CAD BOM (team-confirmed 2026-06-16):** intake **front rollers ARE a Kraken X44** (CAN 18, confirmed present); the intake **slider is a NEO 2.0**, not the V1.1 the CAD lists; and the intake has a **3rd motor — a NEO 2.0 jackshaft** (swapped from a Kraken; new CAN ID needed). Hopper = **2** NEO 2.0 (CAD's 3-position gearbox has 1 empty slot). So physically: **9 Kraken X60, 5 Kraken X44, 4 NEO 2.0, 1 NEO 550 = 19 motors** (14 TalonFX + 5 SPARK MAX); **no NEO V1.1.** See Hardware-Data-Sheet §2 / §4.4 / §10.1.

## Gearboxes / reductions (from tooth counts)

| Mechanism | Reduction | Source parts | Confidence |
|---|---|---|---|
| Flywheel | **1.5:1 overdrive** | 36T HTD5 custom pulley → 24T pulley (WCP-0992/0564) | high — matches code |
| Hopper "gearbox goon" | **6:1 at every motor position** + 2:1 belt → **~12:1 to rollers** | 3× 10T NEO pinion (REV-21-4014) → 2× 60T spur (WCP-0121); the two 60T mesh 1:1 as an **idler**, output spur is coaxial with the 18T pulley → 36T belt (WCP-0563→0990). **Geometry-verified** via occurrence transforms (mesh centers 1.80″, spur↔spur 3.00″). Position changes **direction only** (idler side vs output side → opposite inversion), not ratio. | **high** (Onshape geometry-verified 2026-06-16) |
| Intake roller | **2:1** motor stage | 12T pulley (WCP-1017) → 24T pulley (WCP-0564) | medium |
| Intake slider | **MAXPlanetary 5:1** + spur | cartridge REV-21-2103; spur among 10T/20T/24T/44T | partial |
| Shooter hood | UltraPlanetary ~9:1 (2× "3:1" REV-41-1601) + (2:1 spur 20T WCP-0734→40T WCP-0177) + **24T→180T sector = 7.5:1** | sector **verified** (gear feature `numTeeth=180`, 18" PD, 10 DP); code says 187.5:1 ⇒ UltraPlanetary cartridge ratio (3:1 vs 5:1) still to confirm |
| Kicker | 11T SplineXS pinions (WCP-1009) + 18T pulley + belt | partial |

## Gear / pulley inventory (qty, teeth)

**Shooter:** spurs 20T×2, 40T×2, 13T (UltraPlanetary), 11T SplineXS×4 · **custom hood gears: 180T sector + 24T pinion (10 DP, 18"/2.4" PD) = 7.5:1** · pulleys 24T×11 (15mm+9mm), 36T custom×4, 18T SplineXS · belts 95T, 50T×2, 45T, 165T×2, 115T
**Intake:** spurs 10T×2, 20T×2, 44T, 24T×2 · pulleys 12T (SplineXS), 24T×7 · belts 50T×3, 45T×2
**Hopper:** gears 10T NEO pinion×3, 60T spur×2 · pulleys 18T, 36T, 39T custom · belts 80T×7, 70T×7, 75T×7+1, 95T

## Wheels

| Mechanism | Wheels |
|---|---|
| Shooter flywheel | 4" brass flywheel ×2, 4" urethane (45A) ×4, 2" urethane (45A) ×10 → **2" effective radius** |
| Intake | 4" mecanum (217-3645) ×3, 4" squish ×5, 3" compliant (60A) ×3 |
| Hopper | 2" mecanum (WCP-0353/0354) ×7, 2" Thrifty squish (45A) ×10 |

## Mechanism spans (longest hex shafts)

| Mechanism | Spans |
|---|---|
| Shooter | bottom shaft 18.91", top shafts 7.655" ×2, kicker shaft 16.66" |
| Intake | full-width roller shafts ~30–31" (1/2" ThunderHex 31" / 30.125") |
| Hopper | full-width floor shafts ~21.1" ×5 (1/2" ThunderHex) |

> Intake (~30") and hopper (~21") spans exceed the 24" (0.61 m) frame width — intake likely extends past the frame, or shafts are uncut stock. Confirm with a photo.

## Mass properties — ⚠ UNRELIABLE (unassigned materials → default density)

| Assembly | Mass | MOI (z) | Missing materials | Verdict |
|---|---|---|---|---|
| Shooter | 16.39 kg | 0.671 kg·m² | 2 | rough |
| Intake | 6.77 kg | 0.815 kg·m² | 15 | very low |
| Hopper | 9.39 kg | 1.243 kg·m² | 2 | rough |
| Top-level robot | 680 kg | 131 kg·m² | 26 | **garbage — discard** |

> Do not use any CAD mass. Weigh the robot. PathPlanner's hand-entered **63.5 kg / 7.68 kg·m²** is the current best full-robot figure.
