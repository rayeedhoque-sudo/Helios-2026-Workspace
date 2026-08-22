*Template: copy into the robot project as `docs/robot-spec.md`, then fill per subsystem by interview + code reading. This file is the single source of truth for hardware facts before writing any robot code.*

## Marker rules (mandatory on EVERY value)

| Marker | Meaning |
|---|---|
| `[verified]` | User explicitly confirmed it, OR read directly from project source (cite file path) |
| `[CAD]` | Pulled from Onshape CAD (helper or a CAD screenshot). MUST cite document + element + derivation (variable / config / BOM-name / gear-relation / mass-props). Reflects the *model*, not the as-built robot/code — **treated like `[assumed]` at the pre-deploy gate** |
| `[assumed]` | Claude inferred/guessed. MUST be confirmed with the user before deploy |

Never silently upgrade `[assumed]` or `[CAD]` to `[verified]` — only user confirmation or matching code does that. Provenance order: `[verified]` (code/user) > `[CAD]` (model) > `[assumed]` (guess/photo). A photo alone is `[assumed]` (you can misread a label); a value from the Onshape **API** is `[CAD]`; a value from a CAD **screenshot** is `[assumed]` (no API provenance); a CAN ID in code is `[verified]`. A `[CAD]` value disagreeing with a `[verified]` code value is a finding to surface, not silently reconcile.

---

## Robot overview

| Field | Value |
|---|---|
| Team / robot / season | TBD `[assumed]` |
| Framework | Command-based Java TBD `[assumed]` |
| Vendordeps (exact versions from `vendordeps/*.json`) | TBD `[assumed]` |
| Drive base | TBD (swerve/tank; module type; gyro + CAN ID) `[assumed]` |
| Known pain points (brownouts, CAN errors, etc.) | TBD `[assumed]` |

## Subsystem template (copy once per subsystem)

### <SubsystemName>

**Motors**
| Function | Motor (exact model) | Controller | CAN ID | Inverted? | Current limit (A) |
|---|---|---|---|---|---|
| TBD | TBD `[assumed]` | TBD `[assumed]` | TBD `[assumed]` | TBD `[assumed]` | TBD — "none configured" is a finding, not a blank `[assumed]` |

**Gear ratios / kinematics** (always state units and direction of ratio; record the source)
- TBD, e.g. "5:1 motor rotations : mechanism rotations" or "1.5 flywheel rot per motor rot" `[assumed]`
- When from CAD, cite source + confidence, e.g. "1.5 wheel rot : 1 motor rot `[CAD]` (Onshape doc <did>, elem <eid>, variable \"gearRatio\", confidence high)". See [onshape-integration.md](onshape-integration.md). For COTS gearboxes/swerve modules the ratio comes from the config code via [cots-ratio-reference.md](cots-ratio-reference.md), not the geometry. Swerve ratios come from Tuner X, not CAD.

**Mass properties (sim seed)** (optional — for physics simulation)
- MOI about rotation axis: TBD kg·m^2 `[CAD]` (Onshape massproperties, axis ?, parallel-axis ?; hasMass=?) — seeds `FlywheelSim` / `SingleJointedArmSim`; verify against measured spin-up. Unreliable if the model has unassigned materials (helper surfaces `massMissingCount`).

**Sensors**
| Sensor | Type | Port/ID (bus: CAN / DIO / analog / SPI) | Used for |
|---|---|---|---|
| TBD | TBD `[assumed]` | TBD `[assumed]` | TBD `[assumed]` |

**Mechanical limits & soft stops** (units mandatory)
- Hard stops: TBD `[assumed]`
- Soft limits in code: TBD (constant name + file if they exist) `[assumed]`
- What breaks if driven past the limit: TBD `[assumed]`

**Desired behaviors** (one line each: trigger → action → end condition)
- TBD `[assumed]`

**Driver-control mapping**
| Input (controller, port, button/axis) | Behavior |
|---|---|
| TBD `[assumed]` | TBD `[assumed]` |

**Known issues / code smells found while reading source**
- TBD

---

## EXAMPLE (filled): Intake — FRC 9704 "Helios", 2026 REBUILT

All `[verified]` values below were read from
`src/main/java/frc/robot/Constants/SubsystemConstants.java` and
`src/main/java/frc/robot/subsystems/misc/IntakeSubsystem.java`.

**Motors**
| Function | Motor (exact model) | Controller | CAN ID | Inverted? | Current limit (A) |
|---|---|---|---|---|---|
| Intake roller | Kraken X60 `[assumed]` (code only proves "a TalonFX-attached motor") | TalonFX | 18 `[verified]` (`INTAKE_MOTOR_ID`) | No invert configured `[verified]` | NONE configured `[verified]` — brownout risk, team has power problems |
| Intake slider (constant is named `INTAKE_PIVOT_MOTOR_ID` but the mechanism is a linear slider) | NEO 550 `[assumed]` (code only proves `MotorType.kBrushless`) | SparkMax | 22 `[verified]` | No invert configured `[verified]` | NONE configured in code `[verified]`; firmware default applies — default value not confirmed against REV docs, do not rely on it `[assumed]` |

**Gear ratios / kinematics**
- Roller ratio: TBD `[assumed]` — not in code, ask user.
- Slider: driven open-loop at fixed duty cycle (±0.5, retract -0.25 in `stowSliderCommand`) `[verified]`; travel distance constants `INTAKE_SLIDER_INCHES` / `OUTTAKE_SLIDER_INCHES` / `STOW_SLIDER_INCHES` all exist but = 0 and unused `[verified]`.

**Sensors**
| Sensor | Type | Port/ID | Used for |
|---|---|---|---|
| None wired into intake logic `[verified]` (an unused `Sensors` stub is instantiated — see Known issues) | — | — | Slider end-of-travel inferred via "stall detection" (see Known issues) |

**Mechanical limits & soft stops**
- Slider hard stops at both ends of travel; commands rely on hitting them `[verified]` (commands race stall-wait vs `waitSeconds(1)`).
- Soft limits: none in code `[verified]`. Slider PID constants exist (`INTAKE_SLIDER_kP/kI/kD`) but = 0 and unused `[verified]`.
- Roller speeds: `INTAKE_SPEED = 0.25`, `OUTTAKE_SPEED = 0.25` duty cycle `[verified]`.

**Desired behaviors** `[verified from code]`
- Intake: extend slider (duty -0.5) until "stall" or 1 s → stop slider → state `INTAKE_STATE` → `periodic()` runs roller at +0.25.
- Outtake: same extend sequence → roller at -0.25.
- Stow: retract slider (duty +0.5) until "stall" or 1 s → roller stopped via `stopMotor()` in `periodic()`.

**Driver-control mapping** `[verified]` (from `src/main/java/frc/robot/RobotContainer.java`)
| Input | Behavior |
|---|---|
| Xbox port 0, right bumper | whileTrue `intakeCommand()`, whileFalse `stowCommand()` |
| Xbox port 0, left bumper | whileTrue `outtakeCommand()`, whileFalse `stowCommand()` |

**Known issues / code smells** `[verified]`
- BUG: `isIntakeSliderStall()` returns `Math.abs(intakeSliderMotor.get()) < STALL_SPEED && intakeSliderMotor.get() > 0` (`STALL_SPEED = 0.05`) — `SparkMax.get()` returns the commanded setpoint (`m_setpoint`), NOT measured current; while commanded at ±0.5 it is always false, so every sequence silently falls through on the 1 s timeout. Real stall detection: `public double getOutputCurrent()` (amps) — confirmed via `javap` against the project's pinned `REVLib-java-2026.0.3.jar` (gradle cache). Caution: the live REV javadoc (codedocs.revrobotics.com) documents a newer Signal-based API (`Signal<Double> getOutputCurrent()`) that does NOT match 2026.0.3 — trust the pinned jar over the website.
- No current limits on either motor; slider stalls against hard stops by design every cycle → sustained stall current → brownout contributor. Fix API (confirmed in `REVLib-java-2026.0.3.jar`): `new SparkMaxConfig().smartCurrentLimit(amps)` then `motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)` — project has no existing `configure()` call to copy from.
- `stowSliderCommand()` has no 1 s timeout race; its `waitUntil(isIntakeSliderStall())` can never become true (see BUG), so the command would never end. Latent only — not bound in `RobotContainer`.
- `IntakeSubsystem` instantiates a `Sensors` object it never uses; `Sensors.java` is a stub (`getIndexSensorA()/B()` return hardcoded `true`; `DigitalInput` imported but never constructed; `LightSensor` DIO 8/9 constants unused).

---

## Interview script (ask ONE question at a time; record answer with `[verified]`)

Ask in this order; skip anything already `[verified]` from code. Prefer "read the code first, ask only the gaps."

1. "What exact motor is on <function>? (e.g. Kraken X60, Falcon 500, NEO, NEO 550 — the controller in code only tells me the family.)"
2. "What's the gear ratio between that motor and the mechanism, and in which direction? (e.g. 5 motor turns = 1 arm turn)"
3. "Are there sensors I can't see in code — limit switches, beam breaks, through-bore encoders? What port are they wired to?"
4. "Where are the mechanical hard stops, in real units (degrees/inches)? What happens if a motor drives past them?"
5. "What should this subsystem do, in your words? Trigger → action → when it stops."
6. "Which button/axis should control it, and on which controller (driver vs operator port)?"
7. "Has this robot browned out or rebooted mid-match? Which mechanisms were running when it happened?" (If yes: budget current limits per motor before writing new features.)
8. "Anything on the robot the code doesn't mention yet?" (catches phantom/placeholder subsystems — e.g. Helios has Climb constants that are all 0 with CAN ID 0.)

## Pre-deploy gate

Grep this file for `[assumed]` **and `[CAD]`** — every remaining hit must be confirmed by the user or read from code before deploy. CAD reflects the model, not the wired robot, so a `[CAD]` value still needs cross-checking. Also grep generated code for `VERIFY-BY-COMPILE:` and resolve each by building (`./gradlew build`; `.\gradlew build` in PowerShell).
