# Helios 2026 — V2 (FRC Team 9704)

Robot code for **Helios**, re-architected clean-slate from the V1 tree at `C:\FRC\Helios-2026\`.
Java 17 · WPILib 2026.2.1 · Phoenix 6 26.3.0 · Phoenix 5 (VictorSPX kicker) · REVLib 2026.0.3 · PathPlanner 2026.

## What V2 is (and isn't)

V2 is a **structural re-architecture** of V1, not a behavioral rewrite. Every tuned,
on-robot-verified value and every control behavior is preserved **byte-for-byte** from V1
source (verified by a line-level diff — see "Verification" below). What changed is *how the
code is organized*, not *what values it uses or how it acts on the robot*.

**Why behavior was preserved:** the tuned numbers (current limits, hood encoder anchors,
follower alignments, CANcoder offsets, shot model, keybinds, inversions) and the safety
interlocks were tuned/verified on the physical robot. Changing them can only be validated on
the robot — which can't be done from a dev machine — so per the project's No-New-Issues rule
they were carried over unchanged. If you want specific control-logic reworked, direct the
change explicitly and it will be done as a deliberate, separately-verified change.

## What changed from V1

- **Standard package layout** (V1's nonstandard layout is gone):
  - `frc.robot` — `Main`, `Robot`, `RobotContainer`, `Telemetry`, `PowerTelemetry`
  - `frc.robot.constants` — `Constants` (was the capitalized `frc.robot.Constants.SubsystemConstants`), `FieldConstants`, `TunerConstants`
  - `frc.robot.subsystems` — `CommandSwerveDrivetrain` (was in `frc.robot.commands`), `ShooterSubsystem`, `IntakeSubsystem`, `HopperSubsystem`, `MatchStatus` (the `misc` sub-package is gone)
  - `frc.robot.util` — `LimelightHelpers`, `LimelightThermalManager` (the `utility` sub-package is gone)
- **Dead code removed:** the all-zero `ClimbSubsystemConstants` (no climber this season).
- **Generated / vendored code preserved verbatim:** `TunerConstants` + `CommandSwerveDrivetrain`
  are the Tuner X-generated swerve (never hand-authored — regenerate via Tuner X on the robot,
  then re-apply the shooter-side-front transform noted in `TunerConstants`); `LimelightHelpers`
  is vendored source; vendordeps and the PathPlanner deploy resources are copied as-is.

## Verification (what "identical" means here)

- `gradlew build` green; **all 15 unit tests pass** (`ShotModelTest`, `LimelightThermalTest`,
  `MatchStatusPracticeMatchTest`, `PathPlannerFilesTest`) — same suite as V1.
- **Line-level diff V1→V2:** every subsystem, `RobotContainer`, the drivetrain, telemetry,
  and `Constants` differ from V1 **only** in `package`/`import` lines (and the dropped Climb
  block). No numeric, PID, current-limit, inversion, keybind, or logic line changed.
- A passing build + sim is the floor, **not** proof of on-robot correctness — see below.

## Adopted from top-team research

A survey of 254/1678/6328/2910 open-source code mostly **validated** V1's architecture (it
already uses command-factory subsystems, supplier-based cross-subsystem coordination instead
of static flags, built-in WPILib telemetry, and PathPlanner). The one net-new pattern adopted:

- **Motor-config factory (`frc.robot.util.MotorConfigs`, 254-style).** Every mechanism
  TalonFX/SparkMax is now configured through helpers that *require* a current limit + neutral/
  idle mode as arguments, so a motor cannot be set up without them — turning the "every motor
  gets a limit + neutral mode from line one" rule into a call-site guarantee, directly targeting
  this team's brownout/burnout history. It only **centralizes** the config calls the subsystems
  already made: every limit, neutral mode, ramp, reset mode, follower-disable, and inversion is
  **byte-identical to V1** (the slider's tiered 15/20/1000 limit uses a tiered overload; the
  hopper's `disableFollowerMode` + `inverted(false)` are applied via an `extra` customizer).
  Swerve motors are still configured by Tuner X, not this factory. Build + all 15 tests green.

## Deliberately NOT adopted (mid-season risk > reward)

- **Superstructure coordinator (1678/254).** Would centralize the shot sequence — but that
  sequence (aim→freeze→feed, kicker at-speed gate, hood interlocks) is exactly the tuned,
  on-robot-verified behavior. Rewriting it re-opens tuned control. Deferred (a directed change).
- **AdvantageKit / IO-layer / log-replay (6328), custom swerve (2910), 971's AOS stack** —
  rejected: heavy dependencies / large regression surface / re-derives tuned swerve, for
  capabilities this robot doesn't currently need.

## Match-efficiency research pass (Limelight + shooter)

A second research pass (Limelight docs + top-team shooter code) drove two safe additions —
both leave active behavior unchanged (the throttle acts only in an idle window; the debounce
ships default-off):

- **Limelight idle throttle** (`LimelightThermalManager`): the vendor-recommended `SetThrottle`
  (NT `throttle_set`) sheds board/CPU heat + power while the camera isn't needed — applied
  *only* while disabled and already-enabled-once (between/after matches); full rate during a
  match and in the pre-match window, so the MegaTag1 boot-seed + in-match MegaTag2 fusion never
  lose frames. Tune `LL_THROTTLE_DISABLED` (100–200) on the robot.
- **Exposure: nothing to change.** There is no runtime exposure API (a Limelight platform
  limit), and the team's fixed **270** is correct for AprilTags — auto-exposure would add motion
  blur and destabilize MegaTag2. Varying-venue lighting = re-tune exposure per venue during
  field setup (device-side), not code.
- **Hailo overheating is physical, not software:** seat its thermal pad, mount to metal, add a
  fan/shroud. Robot code can only warn — it can't cool the fixed ~3.75 W chip.
- **Shooter at-speed kicker debounce** (`AT_SPEED_DEBOUNCE_SEC`): a rising-edge debounce so a
  single noisy velocity sample can't open the kicker early (6328 pattern). **Enabled at 0.06 s**
  (team request 2026-07-24) — the *rise* is delayed but the ball-strike *release* stays immediate
  (WPILib `kRising`), so the existing fuel pacing is unchanged. Set `0` to revert to the raw,
  V1-identical gate. Verify fire rate on the robot; raise toward 0.1–0.2 s for more filtering.
  The rest of the shooter (coast-when-off, 25 A supply cap, closed-loop + kV) already matches
  254/1678/6328 — deliberately unchanged.

## Build / deploy

From `C:\FRC\Helios-2026-V2`:
- `gradlew build` — compile + tests (works offline once deps are cached).
- `gradlew deploy` — needs the robot's network (tether `172.22.11.2` or robot Wi-Fi; team 9704).
- Or use the **driver companion app** (`../Helios-2026/driver-companion`) — its Deploy tab has a
  **V1 / V2 project selector**; pick "V2" to deploy this tree.

## On-robot verify list (carried from V1 — still unverified on hardware)

These were `TODO`-gated in V1 and remain so — a from-scratch re-architecture cannot resolve a
physical/calibration unknown. Test in this order (from the V1 handoff):

1. Shooter C follower direction (`MotorAlignmentValue.Opposed`) — confirm it no longer fights.
2. **Hood encoder anchors** (`HOOD_RAW_AT_FULL_UP/DOWN`) — hand-move to both stops, robot
   disabled, read "Hood Encoder Raw (rot)". *(This is the real "hood won't adjust" item — the
   code path is correct and adjustable; it needs the two-point anchor confirmed on the robot.
   Use Test mode DPAD-LEFT/RIGHT to jog the hood and park it at each stop.)*
3. Hopper belts feed direction (hold B; flip **both** inversions together if backward).
4. Slider extend sign (negative = extend assumed).
5. `TAG_LATERAL_OFFSET_SIGN` (0.0 = disabled) — verify vs botpose, then set ±1.0.
6. Heading-servo PID (5.0 P shared by aim/align) + search rate.
7. Steer supply 30 A watch: if modules hesitate mid-swing, step 30→35→40 (`TunerConstants`).
8. Ballistics: `SHOT_EFFICIENCY` first, drag slopes last.

Plus the device-side checklist from the V1 handoff (Limelight mount pose + pipeline 1 at
exposure 270 / black-level 30; battery load-test; kicker breaker size; weigh the robot).

Developed by Team 9704 with Claude Code.
