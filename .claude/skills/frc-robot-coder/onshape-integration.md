*Pull build data and gear ratios from an Onshape CAD document via the REST API, using the helper `scripts/onshape_fetch.py`. CAD-sourced values enter the spec marked `[CAD]` and are cross-checked against code/hardware before deploy. This file is generic (no team specifics) so the skill stays plugin-shareable; per-robot document URLs and secrets live in the robot project, never here.*

Evidence tags used below: **`api-verified`** = confirmed against a real Onshape response; **`api-hypothesized`** = from research, not yet confirmed (the gear-relation `parameterId` is the one open case — see §11).

## 1. When to use Onshape

Use CAD extraction when the user says "see the build / gear ratios from CAD", shares a `cad.onshape.com` link, or a **custom mechanism** (arm, elevator, shooter, custom gearbox) needs a ratio / dimension / moment-of-inertia that code and photos can't supply.

**Skip swerve.** Swerve gear ratios come from CTRE Tuner X or YAGSL — do **not** spend API quota re-deriving them from CAD. Reserve CAD extraction for custom, non-swerve mechanisms where no generator exists.

## 2. Setup (one-time — needs the user)

1. **Create a read-only API key** at <https://cad.onshape.com/user/developer/apiKeys> (a free Education/FRC account works — `api-verified`). Grant only "Read" scopes. The **secret key is shown once** — copy it now; if lost, regenerate.
2. **Set the keys as environment variables** (never put them in a file that could be committed):
   - PowerShell, persistent (new shells): `setx ONSHAPE_ACCESS_KEY "your-access-key"` then `setx ONSHAPE_SECRET_KEY "your-secret-key"`
   - PowerShell, this session only: `$env:ONSHAPE_ACCESS_KEY = "..."; $env:ONSHAPE_SECRET_KEY = "..."`
   - Git Bash: `export ONSHAPE_ACCESS_KEY="..."` / `export ONSHAPE_SECRET_KEY="..."`
   - *Alternative to env vars:* a gitignored `.onshape.json` next to the script or passed via `--credentials-file`, shaped `{"accessKey": "...", "secretKey": "..."}`. Env vars take precedence; never commit this file.
3. **Add gitignore lines** to the robot project's `.gitignore` (the WPILib `.gitignore` does **not** ignore `*.json`, so this is required to avoid leaking your config):
   ```
   # Onshape integration — local, do not commit
   docs/onshape-config.json
   docs/.onshape-cache/
   .env
   .onshape.json
   *.onshape-key
   ```
4. **Create `docs/onshape-config.json`** in the robot project mapping each subsystem to its Onshape document (Claude reads this to pick the right URL; the script takes `--url`):
   ```json
   {
     "documents": {
       "shooter":  { "url": "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>", "type": "assembly" },
       "armPivot": { "url": "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>", "type": "assembly" }
     },
     "preferVersion": false,
     "notes": "Swerve ratios come from Tuner X, not CAD."
   }
   ```

## 3. Auth & base URL (`api-verified`)

HTTP Basic auth — access key as username, secret key as password, over HTTPS. Base URL `https://cad.onshape.com/api/v10`; header `Accept: application/json;charset=UTF-8;qs=0.09`. The helper handles all of this; you never assemble it by hand. (An HMAC `On`-scheme exists for higher security but is unnecessary for a private read-only tool.)

## 4. URL / id model (`api-verified`)

A document URL is `https://cad.onshape.com/documents/{did}/[w|v|m]/{wvmid}/e/{eid}` — 24-hex ids. The middle selector matters: **`w`** = workspace (live, mutable), **`v`** = version (immutable, **cacheable** — prefer for stable reads), **`m`** = microversion. The helper parses this for you and preserves the selector; pass `--prefer-version` to resolve a `w/` link to its latest `v/` for cache-friendly reads.

## 5. Gear-ratio cascade (most → least reliable; stop at the first high-confidence hit)

1. **Named Variables** — `#gearRatio`, `#reduction`, `#wheelDiameter` (`api-verified` endpoint). Gold standard: machine-readable, unambiguous, **high** confidence. Teams following Onshape4FRC conventions store reductions here.
2. **Configuration inputs** — when the ratio is a config parameter of a library part. **medium** confidence.
3. **BOM / part-name** — the common COTS case (~70–80%): the ratio is in a part name like `MK4i L2` or `MAXPlanetary 9:1`. Brittle, **low** confidence; resolve via [cots-ratio-reference.md](cots-ratio-reference.md) and confirm with the user.
4. **Gear relations** — modeled `BTMMateRelation` ratios. Advanced; the ratio `parameterId` is `api-hypothesized`, so capped at **medium** confidence until confirmed (§11).

The helper runs this cascade in `--mode all` and short-circuits once a high-confidence Variable answers — saving API quota.

**Known limits (read tooth counts by hand):** the helper does NOT auto-pair *fastened* meshing gears — e.g. a 10T pinion into a 60T gear = 6:1 — because the ratio is in the geometry/tooth counts, not a relation or variable. The helper surfaces the part names in the BOM (`--mode bom`); pair the meshes yourself. Also: a modeled gear *relation* can be a stale artifact (a real shooter encoded `24/180` with no matching 180t pulley) — always cross-check a `[CAD]` ratio against the physical pulley/sprocket inventory. And a **view-only shared document cannot have materials assigned**, so its mass/inertia can't be improved without an editable copy.

## 6. Invoking the helper

Run from the **robot project root** so the cache lands in the project. Interpreter is `python` (full path `C:\Python314\python.exe`).

PowerShell:
```powershell
python "C:\Users\Rayeed Hoque\.claude\skills\frc-robot-coder\scripts\onshape_fetch.py" `
  --url "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>" `
  --mode all --cache-dir "docs/.onshape-cache"
```
Git Bash:
```bash
python "/c/Users/Rayeed Hoque/.claude/skills/frc-robot-coder/scripts/onshape_fetch.py" \
  --url "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>" \
  --mode all --cache-dir "docs/.onshape-cache"
```
Useful flags: `--mode {variables,config,bom,massprops,relations,all}`; `--configuration "<encoded>"` for a non-default config; `--prefer-version` (cacheable reads); `--rotation-axis {x,y,z}` and `--axis-offset-m <m>` for mass-properties MOI; `--no-cache` to force a fresh read. `stdout` is always one JSON object; notes go to stderr.

Note: `--mode config` lists the configuration *inputs* only. To read values for a non-default configuration, pass an already-encoded `--configuration "<encoded>"` string to `variables`/`bom`/`massprops` — auto-generating that encoding (the `configurationencodings` round-trip) is not built in yet, so default-configuration reads are the supported path.

## 7. Interpreting the JSON

```
gearRatios[]   {value, units, source(variable|config|bom-name|relation|cots-table-candidate), sourceName, confidence, raw, context}  — sorted high→low confidence
dimensions[]   {name, value, units, source, confidence, raw}            — wheel/sprocket/drum sizes (SI)
bom[]          {name, partNumber, description, quantity, itemSource, candidateRatio}
massProperties {mass_kg, centroid_m, inertiaTensor_kgm2, momentOfInertia_kgm2, axis, appliedParallelAxis, offset_m, hasMass, massMissingCount, confidence}
relationParameterIdsSeen[]   — supports the §11 confirmation task
apiCallsUsed, quotaExhausted, warnings[]
```
Pick the **highest-confidence** `gearRatios` entry; always read `warnings[]` (quota, missing material). `apiCallsUsed` lets you watch quota; `0` means everything came from cache.

## 8. Writing results into the spec

Every CAD-derived value enters `docs/robot-spec.md` as **`[CAD]`** (see [robot-spec-template.md](robot-spec-template.md)), citing the document + element + derivation method and confidence:
```
- Shooter wheel ratio: 1.5 wheel rot : 1 motor rot [CAD] (Onshape doc <did>, elem <eid>, variable "gearRatio", confidence high) — confirm against code/hardware before deploy.
- Wheel diameter: 4 in / 0.1016 m [CAD] (variable "wheelDiameter").
- MOI about rotation axis: 0.0123 kg·m^2 [CAD] (massproperties, axis z; hasMass=true) — seeds FlywheelSim/SingleJointedArmSim; verify vs measured spin-up.
```
`[CAD]` is treated like `[assumed]` at the pre-deploy gate (it reflects the *model*, not the wired robot). **NEVER** silently promote `[CAD]` to `[verified]` — only matching code or user confirmation does that.

## 9. Quota discipline

Onshape allows ~**2,500 API calls/year** (Free/EDU per user; Educator per classroom). Only 2xx/3xx count. The helper already: caches to disk (`docs/.onshape-cache/`), prefers `v/` immutable reads with `--prefer-version`, short-circuits the ratio cascade, backs off on **429** (honoring `Retry-After`), and **stops on 402** (`quotaExhausted: true`, exit code 3). A full custom-mechanism pass costs a handful of calls; a re-run with no model change is a cache hit (`apiCallsUsed: 0`). Do not poll in loops; do not extract swerve.

## 10. Screenshot fallback

When there are no keys yet, the quota is exhausted (402), or a value isn't API-reachable: have the user open the Onshape **Variable table**, **BOM**, or **Mass Properties** panel and paste a **screenshot**. Read it via the photo path ([hardware-identification.md](hardware-identification.md) §1.5). A screenshot-sourced value is **`[assumed]`** (no API provenance) — *not* `[CAD]`. Same downgrade rule as any photo.

## 11. One-time gear-relation `parameterId` confirmation (`api-hypothesized` → `api-verified`)

The literal `parameterId` of the gear-relation ratio is not in public docs, so the extractor finds it defensively by parameter type (`BTMParameterQuantity-147`) rather than by id. To confirm it once and raise confidence:

1. In a throwaway Onshape document, create two mates and a **gear relation** with a **non-default** ratio (e.g. 3.0) — defaults are omitted from the encoded output, so a non-default value is readable.
2. Run `onshape_fetch.py --mode relations --url <that doc>`.
3. Read `relationParameterIdsSeen` and which entry's `value`/`raw` carries your `3.0`.
4. Record the confirmed ids in the table below and change this section's tag to `api-verified`.

**Confirmed 2026-06-13** against Team 9704's shooter CAD (via `relationParameterIdsSeen`) — the ids below are now `api-verified`. The extractor still locates them defensively by parameter type, and this procedure is retained in case Onshape changes them.

| Field | parameterId | Status |
|---|---|---|
| gear-relation ratio (quantity) | `relationRatio` | `api-verified` |
| rack/screw lead (quantity) | `relationLength` | `api-verified` |
| relation type (enum) | `relationType` | `api-verified` |
| reverse direction (boolean) | `reverseDirection` | `api-verified` |
| coupled mates (query) | `matesQuery` | `api-verified` |

Alternatives if you'd rather not build a scratch relation: the Glassworks explorer (<https://cad.onshape.com/glassworks/explorer/>, search `BTMMateRelation`) or an authenticated `GET https://cad.onshape.com/api/openapi`.

## 12. Security

- Secrets come from env vars (or a gitignored `.onshape.json` / `--credentials-file`) — **never** committed, **never** in `onshape-config.json`, **never** printed. The helper never logs the secret or the `Authorization` header.
- Create keys read-only-scoped. If a secret is ever exposed, regenerate it at the developer portal.
- Confirm `git status` shows `docs/onshape-config.json`, `docs/.onshape-cache/`, and any key file as untracked after setup.
