*Documented gear ratios for common FRC COTS gearboxes and swerve modules. Use this to resolve a config code or part name (from a photo label or an Onshape BOM) into a candidate ratio. Every value here is a **candidate** — it enters the spec as `[CAD]` (if read from Onshape) or `[assumed]` (if read from a photo), and is confirmed against the actual robot or code before deploy.*

> **Why this file exists:** COTS gearboxes are usually inserted into CAD as a single rigid solid with the reduction "baked in" — there are no modeled gear teeth, so the ratio is **not** in the geometry and cannot be computed from the model. It lives only in the manufacturer's spec, keyed by a config code (e.g. `L2`, `12T`) or the part name. `onshape_fetch.py` attaches a low-confidence candidate from `scripts/cots_ratios.json`; **this markdown is the source of truth** and the script's JSON is a derived copy (regenerate the JSON from this file when you change it — identical keys).

## Hard rules

- **NEVER** invent a ratio for a config you don't find here. If a module/config isn't listed (see *Needs confirmation* below), read the vendor's ratio table and confirm with the user — do not guess by interpolating.
- **Swerve ratios normally come from CTRE Tuner X or YAGSL**, not from CAD. Use this table to *cross-check* a Tuner-X value or to handle a non-CTRE stack — not to spend Onshape API quota re-deriving swerve. (See [onshape-integration.md](onshape-integration.md).)
- A config code resolves to a ratio only together with its **direction**: drive ratios are *motor : wheel*; steer/azimuth ratios are *motor : module rotation*. State which when you record it.

## Swerve modules

Drive ratio = motor rotations per wheel rotation. Steer ratio = motor rotations per module (azimuth) rotation.

| Module | Steer (azimuth) | Drive configs (motor : wheel) | Notes |
|---|---|---|---|
| **SDS MK4** | 12.8 : 1 | L1 = 8.14, L2 = 6.75, L3 = 6.12, L4 = 5.14 | Falcon/Kraken pinion |
| **SDS MK4i** | 150/7 ≈ 21.43 : 1 | L1 = 8.14, L2 = 6.75, L3 = 6.12 | inverted; no L4. "+" variants use a 16T pinion — see below |
| **REV MAXSwerve** | 9424/203 ≈ 46.42 : 1 | 12T = 5.50 (Low), 13T = 5.08 (Med), 14T = 4.71 (High) | ratio set by drive-pinion tooth count |

Source: vendor product pages + YAGSL standard conversion factors (community-canonical). Confirm the *actual* config code on the robot before trusting a value.

## Planetary gearboxes (compute the NET ratio)

A planetary stack's net ratio is the **product of its stacked cartridge ratios** — the teeth are not individually modeled, so read the cartridge config code, don't measure geometry.

| Gearbox | Cartridge stages available | Net ratio |
|---|---|---|
| **REV MAXPlanetary** | 3:1, 4:1, 5:1, 9:1 (9:1 first-stage only) | product of the cartridges, e.g. 9:1 + 4:1 = **36:1** |
| **VEXpro VersaPlanetary** | 3:1, 4:1, 5:1, 7:1, 10:1 | product of the stages, e.g. 5:1 + 4:1 = **20:1** |

Worked example: a MAXPlanetary with a 9:1 then a 5:1 cartridge → 9 × 5 = **45:1** (motor : output). Always confirm the cartridge stack with the user — the config code or a photo of the stamped cartridges is the only reliable source.

## Needs confirmation (NOT embedded — read the datasheet)

These are config-heavy or I am not confident of the exact documented drive ratios, so they are intentionally left out of the lookup table. Read the vendor ratio table and confirm with the user; never reuse a similar module's numbers.

- **SDS MK4n** — steer 18.75 : 1; drive ratios: read the SDS MK4n table.
- **SDS MK4c** — steer 12.8 : 1; drive ratios: read the SDS MK4c table.
- **SDS MK4i "+" (L1+/L2+/L3+)** — 16T drive pinion (~14% faster than the 14T base). Read the SDS table; do **not** reuse the base L1/L2/L3 numbers.
- **WCP Swerve X / X2 / XS** — many gear-set × pinion configurations; hand-enter from the WCP product ratio table.
- **ThriftyBot Swerve** — see YAGSL standard conversion factors / ThriftyBot docs.

## How Claude uses this

1. Get a config code or part name from a photo label ([hardware-identification.md](hardware-identification.md) §1.6) or an Onshape BOM (`onshape_fetch.py --mode bom`).
2. Match the family + config here (or via the script's low-confidence candidate).
3. Record the ratio in `docs/robot-spec.md` under "Gear ratios / kinematics" with its source and direction, marked `[CAD]`/`[assumed]`, and **ask the user to confirm** before it is treated as anything stronger.

*Sync note: `scripts/cots_ratios.json` is a derived copy of this file for the helper script. When you edit a value here, update the JSON to match (identical keys).*
