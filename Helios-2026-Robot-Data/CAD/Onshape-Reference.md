# Helios 2026 — Onshape CAD Reference

The team's CAD documents and how the data in this folder was pulled from them. Raw pulls are in [raw/](raw/); the human-readable digest is [cad-bom-digest.md](cad-bom-digest.md).

## Documents (from `2026-Helios-Code/docs/onshape-config.json`)

| Subsystem | Nickname | URL | Pulled |
|---|---|---|---|
| Shooter | "mahinski on the shootski" | `https://cad.onshape.com/documents/33c1707818bb825081d64496/v/d463951fbe3acea693177426/e/eaa2bd86ab52340572b184e0` | ✅ all modes |
| Intake | "slideuzzy intuzzy" | `https://cad.onshape.com/documents/13ee370048fd8ffd56bbee96/v/2f243b30b721e9c736969a31/e/a55e4e29195b7ca807f24ae1` | ✅ all modes |
| Hopper | "Hoppity Hopper" (has "gearbox goon") | `https://cad.onshape.com/documents/e5843f80827566d656536e4a/v/307372e77bc2ce584ace8b4a/e/2bdc84552249bbf56972dfe1` | ✅ all modes |
| **Launcher V3 (V2 shooter)** | "Unvaulting may 4 2019" (Launcher V3 folder) | `https://cad.onshape.com/documents/33c1707818bb825081d64496/w/6da0fa1b2da5c93748f3fd3d/e/b3a34082366b447d854f9347` | ✅ 2026-09-29 all modes + indented BOM + assembly def + Part Studio `10ad8c0e…` features |
| **Hopper V2** | "Assembly 1" | `https://cad.onshape.com/documents/706596de2507bbf61861d0a7/w/8a09c46db37b18c30360443b/e/9bfd675f4546e27aaa2716da` | ✅ 2026-09-29 all modes + indented BOM + assembly def |
| Top-level robot v2 | "Helios V2" (re-pulled 2026-09-29: flat + multi-level BOM) | `https://cad.onshape.com/documents/303789bd62fb0d9d609c14b6/w/a87ec714c13eeda7135bab77/e/07728cb84454d25bab1e0df4` | ✅ massprops + bom |
| Top-level robot v1 | "helios_v1" | `https://cad.onshape.com/documents/303789bd62fb0d9d609c14b6/w/a87ec714c13eeda7135bab77/e/eb069a40237674fe1af7e361` | not pulled |
| Variable Studio | — | `https://cad.onshape.com/documents/303789bd62fb0d9d609c14b6/w/a87ec714c13eeda7135bab77/e/044435da11aeedef217d4b11` | empty (team uses gear relations + COTS, not named variables) |

Notes from the config: *"Swerve ratios come from CTRE Tuner X, not CAD."* · *"No climber this season."* · Sub-assemblies are pinned `/v/` versions (immutable, cacheable, quota-friendly).

## V2 raw pulls ([raw/v2/](raw/v2/), 2026-09-29, ~20 API calls)

| File | Contents |
|---|---|
| `launcher-v3.json`, `hopper-v2.json` | helper `--mode all` (BOM, massprops, relations — no gear relations modeled) |
| `toplevel-v2-bom.json`, `toplevel-v2-bom-multilevel.json` | flat + multi-level indented BOM of "Helios V2" (shows which subassembly owns each motor) |
| `assembly-defs.json` | launcher + hopper assembly definitions (occurrence transforms + mates) — used to pair motors↔pulleys↔shafts |
| `launcher-gear-features.json` | Part Studio `10ad8c0e…` features (filtered to gear/pulley generators): belt/pulley generator tooth counts + the **170T hood sector** spur-gear feature |
| `hopper-pinion-partstudio-features.json` | Part Studio `0ce2eac6…` (imported STEP only — no tooth count readable) |

Method notes: workspace (`/w/`) reads aren't cache-stable — prefer pinning a `/v/` version for re-pulls. Belt/gear pairings were done by matching occurrence (y,z) shaft positions and checking belt length = 2C + π(D1+D2)/2 against the BOM belt tooth counts.

## What each raw pull contains ([raw/](raw/)) — V1 (2026-06-14)

| File | Source | Highlights |
|---|---|---|
| `shooter-assembly.json` | shooter doc, `--mode all` | 5× Kraken X60 + NEO 550 hood; flywheel 36T→24T (1.5×); real "24/180" hood gear relation (24T→180T sector, 7.5:1); 113 BOM lines |
| `intake-assembly.json` | intake doc, `--mode all` | Kraken X44 roller + NEO V1.1 slider; MAXPlanetary 5:1; 12T→24T (2×); 86 BOM lines |
| `hopper-assembly.json` | hopper doc, `--mode all` | 3× NEO 2.0; "gearbox goon" 10T→60T (6×) + 18T→36T (2×); 71 BOM lines |
| `toplevel-robot-massprops.json` | helios_v2, `--mode massprops` | 680 kg ⚠ (default density + missing swerve — unreliable, do not use) |
| `toplevel-robot-bom.json` | helios_v2, `--mode bom` | Full motor inventory: 5 Kraken X60, 1 Kraken X44, 3 NEO 2.0, 1 NEO V1.1, 1 NEO 550, 5 SPARK MAX, 2 Pigeon 2.0, 1 Limelight 4; 317 BOM lines |

## Re-pulling (quota-safe)

Credentials: either the env vars `ONSHAPE_ACCESS_KEY` / `ONSHAPE_SECRET_KEY`, **or** the local key file **`C:\FRC\.onshape.json`** (gitignored — `{"accessKey":...,"secretKey":...}`) passed via `--credentials-file`. Onshape free/EDU quota ≈ 2,500 calls/year; the helper caches, so re-runs of unchanged `/v/` versions cost **0 calls**. The script is at `C:\FRC\.claude\skills\frc-robot-coder\scripts\onshape_fetch.py`.

```bash
python "C:/FRC/.claude/skills/frc-robot-coder/scripts/onshape_fetch.py" \
  --url "<document-url>" --mode all \
  --credentials-file "C:/FRC/.onshape.json" \
  --cache-dir "C:/FRC/Helios-2026-Robot-Data/CAD/.onshape-cache"
```

> ⚠️ **`C:\FRC\.onshape.json` holds a live Onshape API key.** It travels with the `C:\FRC` folder, so anyone with the folder can read the team's CAD. Use a **read-only-scoped** key (create at <https://cad.onshape.com/user/developer/apiKeys>), keep it gitignored, and **regenerate it** at the developer portal if the folder is ever shared publicly. Re-pulling is usually unnecessary — every CAD value and raw pull is already saved in [raw/](raw/).

Modes: `variables`, `config`, `bom`, `massprops`, `relations`, `all`. Swerve is intentionally **not** pulled (use Tuner X).

## CAD caveats (carried into the data sheet)

- **Masses are unreliable** — every assembly has unassigned materials, so Onshape used a default density. Weigh the robot instead.
- **The "24/180" shooter gear relation is REAL** — it's the hood's 24T pinion → 180T sector gear (7.5:1), verified from the hood Part Studio gear feature (`numTeeth=180`, 18" PD, 10 DP). (An earlier note called it a stale artifact; that was wrong — it's a spur sector gear, not a missing pulley.)
- **CAD ≠ wired robot.** CAD gives geometry/ratios, never CAN IDs, current limits, inversions, or sensor polarity. Confirm those on the robot.
