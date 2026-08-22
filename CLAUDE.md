# Project
This is the robot code for FRC Team 9704 robot "Helios" — the software that controls the robot.
This is a **from-scratch rewrite**; the old 2026 code is at `C:\FRC\extracted\2026-Helios-Code-main\` and is **reference only** (it has no current limits and several bugs — do not copy it).

## Always Do First
- Always invoke the `frc-robot-coder` skill when working on robot code, every session, no exceptions.

## Always Do
- If unsure about a measurment or current value, or unsure if a module or part exists, interview the user before preceding, no matter what, no exceptions.
- Always have a adversarial view to the user, if you find that there could be potential hardware or safety risks involved in a new addition/fix/feature from the user, present the risks to the user and ask for permissions before proceeding with the change/implementation.

## Project Root (scope here)
- **The robot project lives in `C:\FRC\Helios-2026\`.** Create ALL new robot code there, and run every `gradlew` build/deploy/test command from that directory (`cd C:\FRC\Helios-2026`). Do not create robot code or run gradle at the `C:\FRC` root.
- This file (`C:\FRC\CLAUDE.md`) auto-loads as an ancestor even when the working dir is `C:\FRC\Helios-2026`, so full project context is always available.
- *(WPILib VS Code note for humans: open the `Helios-2026` folder — not `C:\FRC` — so the extension's Build/Deploy buttons scope to the project. CLAUDE.md/extension scoping can't be set from this file; it follows the opened folder.)*

## Robot Hardware Data — READ BEFORE CODING
All known hardware/config data lives in **`C:\FRC\Helios-2026-Robot-Data\`**. Start at its `README.md` (§ "START HERE"), then `Hardware-Data-Sheet.md` — the authoritative reference: full CAN map, motor models, gear ratios, current limits + brake/coast (§7), drivetrain geometry, AprilTags (§3), and the still-open items (§11). Re-read it before writing each subsystem instead of re-deriving from the old tree.

Key facts the new code MUST honor:
- **Every motor gets a current limit + neutral (brake/coast) mode from line one** — use the *conservative-start* column in Hardware-Data-Sheet §7; never a vendor default. (The old code limited nothing → brownout/burnout history.)
- **No climber this season.** The **hopper gearbox is a 3-motor-position design with only 2 NEO 2.0 installed** (CAN 20, 21) — confirmed 2026-06-16; the 3rd position is empty. Gearbox ratio **6:1 per position → ~12:1 to rollers** (position-independent; only direction differs — set the 2 motors' relative inversion).
- **Intake = 2 motors (corrected 2026-07-21 — team confirmed in conversation: the slider motor IS the jackshaft, not a separate 3rd motor; there was never a missing CAN ID):** (1) **front rollers = Kraken X44 / TalonFX, CAN 18** (photo-confirmed — NOT swapped); (2) **slider/jackshaft = NEO 2.0 / SPARK MAX, CAN 22** (swapped from a Kraken post-CAD; CAD separately mislabeled it NEO V1.1 — it's a NEO 2.0). Keep CAN 18 a TalonFX; the NEO 2.0 is a SparkMax (smartCurrentLimit + idle mode + `getOutputCurrent()` stall detection). *(An earlier note described this as 3 motors with a jackshaft still needing a CAN ID — that was wrong; do not re-introduce it.)*
- 18 motors total: 9× Kraken X60, 5× Kraken X44, 3× NEO 2.0 (2 hopper + intake slider/jackshaft), 1× NEO 550 (no NEO V1.1); controllers = 14 TalonFX + 4 SPARK MAX; + 4 CANcoders, Pigeon 2 (CAN 23), Limelight 4.
- Values tagged `[placeholder]`, `[CAD]`, or `[inferred]` in the data sheet are **not final** — write them as `// TODO` constants with units.
- Items the data sheet (§11) marks "needs the robot" (motor inversions, CANcoder offsets, breaker sizes, PID/shooting-model tuning, robot mass) are genuinely unknown — leave a TODO and follow the "interview the user" rule below.

## Networking / Deploy
- **Team number: 9704.** Operating IPs: radio/gateway `10.97.4.1`, roboRIO `10.97.4.2`, Limelight `limelight-knight.local` (~`10.97.4.11`). USB-B tether to the roboRIO = `172.22.11.2`. Deploying code needs the robot's network (tether or robot Wi-Fi).
- Radio (VH-109) reprogramming steps, installed deploy tooling, bundled installers (`C:\FRC\tools\`), and remaining tool installs: invoke the project's **`radio-deploy` skill**.


## Code Style
- All robot code is in Java (WPILib / GradleRIO).
- Use the WPILib VS Code extension for all FRC robot code.
- Write a separate class for each subsystem (hopper, intake, etc.).
- Use simple comments that explain what each function does.

## Post-Write
- After changing or writing any code, review the changed class for current/voltage issues
  that could be dangerous or inefficient to run on the robot (brownouts, stalls, etc.).
- You may add to this CLAUDE.md, but existing content must be preserved across compaction.

## No-New-Issues Rule (regressions have burned us repeatedly)
Past sessions kept introducing new bugs while fixing old ones. Every code change must:
- Fix ONLY the named issue — smallest possible diff, no drive-by refactors, no new behavior
  paths bundled into a fix. New functionality is its own change, separately requested.
- Reuse the in-project pattern for the job (e.g. the 0.25 s open-loop ramp, the Debouncer
  stall pattern) instead of inventing a new mechanism.
- Never touch tuned on-robot values (current limits, hood voltages/anchors, follower
  alignments, keybinds) unless the change is ABOUT them and the team directed it.
- End with a desk check before claiming done: `gradlew build` + all tests green, plus a
  small pinning test whenever pure logic/math changed (shot model, gates, state machines).
- If a change can only be verified on the robot (signs, inversions, physical behavior),
  leave it gated/TODO and say so — never ship an unverifiable behavior change as a "fix".

## Commit Policy (every code change gets committed)
- **Every code change is committed in the same session it is made.** One commit per logical
  change (a fix, a feature, one tuning pass) — committed immediately after its desk check
  (build + tests green) passes, before starting the next change. Not per file-edit:
  mid-edit states don't compile; the verified logical unit is what gets committed.
- Never batch unrelated changes into one commit, and **never end a session with uncommitted
  code** — the 2026-07-18 scare (on-robot fixes living only in an uncommitted working tree,
  nearly lost to a revert) is why this rule exists.
- On-robot tuning sessions included: whenever changed values made the robot behave, commit
  right then — those numbers are exactly what almost got lost.
- Message style: short imperative summary line; body says what + why; end with
  "Developed by Team 9704 with Claude Code." (FIRST AI-attribution policy).

## Open Work
- **Read `C:\FRC\Helios-2026\docs\handoff-2026-07-21.md` before resuming robot-code work** —
  the prioritized open-issues handoff from the 2026-07-21 full-code review. All 5 numbered
  issues in that doc are now resolved (issue 1's "jackshaft CAN ID" was never actually
  missing — see the intake correction above); remaining open work lives in that doc's
  device-side checklist and on-robot verify list, not the numbered issues.
