*REVLib 2026 SPARK MAX patterns (NEO / NEO 550) for Helios — current config API, current limits, encoders, closed loop, real stall detection.*

## Ground truth

- Pinned version: REVLib **2026.0.3** (`vendordeps/REVLib.json`). All signatures below were verified with `javap` against the resolved jar:
  `C:\Users\Rayeed Hoque\.gradle\caches\modules-2\files-2.1\com.revrobotics.frc\REVLib-java\2026.0.3\...\REVLib-java-2026.0.3.jar`
- Import root is `com.revrobotics.spark.*` (NOT legacy `com.revrobotics.CANSparkMax`). Confirmed in project source:
  `src\main\java\frc\robot\subsystems\misc\ShooterSubsystem.java` (lines 8-9, 39-40), `IntakeSubsystem.java` (lines 4, 22).
- Docs: https://docs.revrobotics.com/revlib/spark/configuring-a-spark , https://docs.revrobotics.com/revlib/spark/closed-loop , javadocs https://codedocs.revrobotics.com (codedocs may show a NEWER dev API than 2026.0.3 — e.g. `Signal<Double>` returns. The jar wins; on 2026.0.3 the getters below return plain `double`).
- 2026-06-12: every snippet in this file was compile-checked in-project (`gradlew compileJava` against the pinned jars) — only deprecation notes below came from that pass.

## SPARK MAX devices on Helios (and their config status)

| CAN | Device | Motor | Encoder | Config applied in code |
|-----|--------|-------|---------|------------------------|
| 19 | Shooter hood (`shooterAngle`, ShooterSubsystem.java:39) | NEO 550 (per `NEO550_ROTATIONS_PER_HOOD_DEGREE`, line 135) | REV absolute via `getAbsoluteEncoder()` | **NONE** — running 80 A default limit |
| 20 | Hopper A (`hopperMotorA`, HopperSubsystem.java:19) | brushless, model not in code | none used | **NONE** — 80 A default |
| 21 | Hopper B (`hopperMotorB`, HopperSubsystem.java:20) | brushless, model not in code | none used | **NONE** — 80 A default |
| 22 | Intake slider (`intakeSliderMotor`, IntakeSubsystem.java:22) | brushless, model not in code | none used | **NONE** — 80 A default |

NO SPARK MAX in the project ever calls `configure(...)` (grep confirms; the only `configure` hits are AutoBuilder and Jackson). The swerve IS limited — TunerConstants.java: steer 40 A stator (lines 60-67), drive bounded by 120 A slip current (line 55, applied via `withSlipCurrent`, line 115). The brownout/burnout exposure is these four mechanism SPARKs, plus the TalonFX mechanisms (see Phoenix file).

## 1. Config API — SparkMaxConfig + configure()

Legacy `restoreFactoryDefaults()` / `burnFlash()` / `setSmartCurrentLimit()` on the motor object DO NOT EXIST in 2026.0.3. Everything goes through a config object:

```java
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;      // top-level. The nested SparkBase.ResetMode/PersistMode configure()
import com.revrobotics.PersistMode;    // overload also compiles but is @Deprecated(forRemoval) in 2026.0.3 — use these

SparkMax hood = new SparkMax(19, MotorType.kBrushless);
SparkMaxConfig cfg = new SparkMaxConfig();
cfg.smartCurrentLimit(20)              // Amps — NEO 550: NEVER skip this (see section 2)
   .idleMode(IdleMode.kBrake)
   .inverted(false);
// configure() returns REVLibError — check it at init
hood.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

- `ResetMode.kResetSafeParameters` = factory-default everything not set in `cfg` (the `restoreFactoryDefaults()` replacement; use at construction). `kNoResetSafeParameters` = patch only the listed settings.
- **Hazard (hard rule 3):** on a device that has never been configured from code, `kResetSafeParameters` silently wipes settings persisted via the REV Hardware Client (e.g. inversion) that existing command signs may have been tuned against. Until the device's config has been read out (REV Hardware Client) and captured in code, patch with `kNoResetSafeParameters` and leave a TODO (this is how the Helios intake slider is handled — `IntakeSubsystem` constructor).
- `PersistMode.kPersistParameters` = save to controller memory (the `burnFlash()` replacement) so settings survive power cycle/brownout. REV: persisting "is time-intensive and blocks communication" — do it once in the subsystem constructor, never periodically (configuring-a-spark URL below).
- `configureAsync(...)` exists with the same arguments (verified in jar).
- Source: https://docs.revrobotics.com/revlib/spark/configuring-a-spark

## 2. Current limits — the brownout/burnout fix

| Parameter | Value | Source |
|-----------|-------|--------|
| SPARK MAX **default** smart current limit | 80 A | https://docs.revrobotics.com/brushless/neo/locked-rotor-testing |
| NEO stall current | 105 A | https://docs.revrobotics.com/brushless/neo/v1.1 |
| NEO 550 stall current | 100 A | https://docs.revrobotics.com/brushless/neo/550 |
| NEO sane starting limit | 40 A | common starting point; REV publishes test data, not one official number |
| NEO 550 limit | **20 A max** | REV locked-rotor data: 20 A survived the full 220 s test; 40 A failed ~27 s, 60 A ~5.5 s, 80 A ~2.0 s |

**WARNING: a stalled NEO 550 at the 80 A default destroys itself in ~2 seconds** (REV locked-rotor data above). REV: "It is highly recommended to adjust the Smart Current Limit when driving the NEO 550." Helios's hood (CAN 19) is exactly this case today. Small mechanisms that can hard-stop (hood, slider) can go lower (10-15 A) — they then stall safely against the hard stop instead of browning out the robot.

```java
cfg.smartCurrentLimit(20);             // stall+free limit, Amps (int)
cfg.smartCurrentLimit(20, 30);         // stallLimit A, freeLimit A
cfg.smartCurrentLimit(20, 30, 1000);   // + limitRpm: below this RPM use stallLimit
cfg.secondaryCurrentLimit(40);         // Amps (double) — hard chop backstop, optional
```

## 3. Idle mode

`IdleMode.kBrake` (positional mechanisms: hood, slider, arms) / `IdleMode.kCoast` (flywheels, intake rollers). Enum has exactly these two values (jar).

## 4. Encoders

Native units: position = **rotations**, velocity = **RPM**; conversion factors multiply those (https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/encoderconfig).

```java
// Relative (built-in NEO encoder): motor.getEncoder() -> com.revrobotics.RelativeEncoder
cfg.encoder
   .positionConversionFactor(360.0 / GEAR_RATIO)        // motor rot -> mechanism degrees
   .velocityConversionFactor(360.0 / GEAR_RATIO / 60.0); // RPM -> deg/s
// RelativeEncoder (jar): double getPosition(), double getVelocity(), REVLibError setPosition(double)

// Absolute (REV Through Bore on data port): motor.getAbsoluteEncoder() -> SparkAbsoluteEncoder
// (com.revrobotics.spark.SparkAbsoluteEncoder — see ShooterSubsystem.java lines 8, 40)
cfg.absoluteEncoder
   .positionConversionFactor(360.0)        // 1 sensor rotation -> 360 deg
   .velocityConversionFactor(360.0 / 60.0) // RPM -> deg/s
   .inverted(false)
   .zeroOffset(0.25);  // rotations, range [0, 1) — applied to the RAW reading "as if the zero offset was
                       // set to 0, the position conversion factor was set to 1, and inverted was set to
                       // false" (codedocs javadoc; signature verified in jar). Verify reading on hardware.
// Also in jar: zeroCentered(boolean) — compiles on 2026.0.3 (checked 2026-06-12); semantics inferred from name (likely [-0.5, 0.5) reporting) — verify the actual reading on hardware
// SparkAbsoluteEncoder: getPosition(), getVelocity() return double (jar)
```

## 5. Closed-loop control (on-controller PID)

Helios currently runs the hood with a roboRIO-side `PIDController` + `setVoltage()` (ShooterSubsystem.java lines 50-54, 217-218) — that works; the on-controller loop below runs at 1 ms and frees the RIO loop. Docs: https://docs.revrobotics.com/revlib/spark/closed-loop

```java
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;       // top-level in 2026.0.3 (NOT ClosedLoopConfig.FeedbackSensor)
import com.revrobotics.spark.ClosedLoopSlot;       // kSlot0..kSlot3

cfg.closedLoop
   .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // codedocs javadoc: "default feedback sensor is assumed to be the primary encoder"
   .pid(0.275, 0.0, 0.0)                            // output [-1,1] per unit of error (post-conversion units)
   .outputRange(-1.0, 1.0);
motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

SparkClosedLoopController ctl = motor.getClosedLoopController();
ctl.setSetpoint(42.0, ControlType.kPosition);       // units = encoder native * conversion factor
```

- `setSetpoint(double, ControlType[, ClosedLoopSlot[, double arbFF[, ArbFFUnits]]])` is current; `setReference(...)` is `@Deprecated(forRemoval=true)` in the 2026.0.3 jar itself (javap -v) and merely delegates to `setSetpoint`.
- `ControlType` values (jar): `kDutyCycle, kVelocity, kVoltage, kPosition, kCurrent, kMAXMotionPositionControl, kMAXMotionVelocityControl`.
- `FeedbackSensor` values (jar): `kNoSensor, kPrimaryEncoder, kAnalogSensor, kAlternateOrExternalEncoder, kAbsoluteEncoder, kDetachedAbsoluteEncoder, kDetachedRelativeEncoder`.
- Also on ClosedLoopConfig (jar): `iZone(double)`, `positionWrappingEnabled/InputRange(...)` (for continuous angles), `allowedClosedLoopError(double, ClosedLoopSlot)` (slot arg required in jar; codedocs: "how much deviation from the setpoint is tolerated", units affected by conversion factor), `maxMotion` sub-config. `pidf(p,i,d,ff)` and `velocityFF(double)` compile but are `@Deprecated(forRemoval=true)` in 2026.0.3 (compile-checked 2026-06-12) — prefer `pid(...)` plus the `arbFF` argument of `setSetpoint`. `ctl.isAtSetpoint()` exists (jar) — VERIFY-ON-HARDWARE: no fetched doc states its tolerance comes from `allowedClosedLoopError`.

## 6. Reading REAL current — correct stall detection

`SparkBase` getters, all plain `double` in 2026.0.3 (jar): `getOutputCurrent()` /*Amps*/, `getAppliedOutput()` /*actual duty cycle*/, `getBusVoltage()` /*V*/, `getMotorTemperature()` /*deg C*/, `get()` /*last COMMANDED duty cycle*/.

**Known Helios bug** — `IntakeSubsystem.isIntakeSliderStall()` (IntakeSubsystem.java lines 62-64) tests `intakeSliderMotor.get()`, the commanded duty cycle from `set()` — not a measurement. At `set(-0.5)` (intake/outtake) the `get() > 0` clause is false; at `set(0.5)` (stow) the `Math.abs(get()) < STALL_SPEED` (0.05) clause is false — so it always returns false. intake/outtake/stow just run out their 1 s `waitSeconds` race; `stowSliderCommand()` (lines 129-141) has NO timeout and never finishes. Correct signal:

```java
/** True when the slider is pushing against its hard stop. Current in Amps. */
public boolean isIntakeSliderStall() {
    // STALL_CURRENT_AMPS does NOT exist yet — add it to IntakeSubsystemConstants.
    // The existing STALL_SPEED = 0.05 (SubsystemConstants.java:58) is a duty-cycle threshold; retire it with the bug.
    return intakeSliderMotor.getOutputCurrent() > IntakeSubsystemConstants.STALL_CURRENT_AMPS; // e.g. 15 A with a 20 A smart limit
}
```

Pick the threshold below the configured `smartCurrentLimit` but above free-running current; current also spikes briefly on startup/reversal, so debounce ~0.1-0.25 s before declaring stall: `edu.wpi.first.math.filter.Debouncer` — `new Debouncer(0.2, Debouncer.DebounceType.kRising)`, then `debouncer.calculate(rawStall)` (signatures verified in this project's wpimath-java-2026.2.1 jar).

Three rules that make stall detection actually work (learned fixing the Helios slider, 2026-06-12):
- **Debouncer lifecycle**: wpimath `Debouncer` has no `reset()` (jar-verified) — one left `true` at the end of a move declares an instant stall on the next move's startup spike. Create a **fresh `Debouncer` at the start of every move** (see `IntakeSubsystem.startSliderMove()`).
- **Timing**: debounce time + expected travel time must be comfortably under any `waitSeconds` timeout fallback, or the timer always wins and the detection is dead code.
- **`.get()` as a gate is fine**: `Math.abs(motor.get()) > 0.05 && getOutputCurrent() > THRESHOLD` uses commanded output only as an *are-we-commanding-motion* gate so a stopped motor can't read as stalled — the ban is on using commanded output as the stall *measurement*.

## 7. Followers

Follower mode is configuration, not a control request (unlike Phoenix 6's `setControl(new Follower(...))` used for the Krakens in ShooterSubsystem.java lines 91-93):

```java
SparkMaxConfig followerCfg = new SparkMaxConfig();
followerCfg.follow(LEADER_CAN_ID, true);   // overloads (jar): follow(int), follow(int, boolean invert),
                                           // follow(SparkBase), follow(SparkBase, boolean)
followerMotor.configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
// Apply current limits to the FOLLOWER's own config too — they are per-controller.
// Also on SparkBase (jar): isFollower(), pauseFollowerMode(), resumeFollowerMode(); cfg.disableFollowerMode()
```

## 8. Legacy-API trap table (training-data drift — do not emit the left column)

| Legacy (pre-2025) | Current (2026.0.3, in jar) |
|---|---|
| `CANSparkMax`, `com.revrobotics.CANSparkMax` | `SparkMax`, `com.revrobotics.spark.SparkMax` |
| `restoreFactoryDefaults()` | `configure(cfg, ResetMode.kResetSafeParameters, ...)` |
| `burnFlash()` | `PersistMode.kPersistParameters` argument |
| `motor.setSmartCurrentLimit(40)` | `cfg.smartCurrentLimit(40)` then `configure(...)` |
| `motor.setIdleMode(...)` | `cfg.idleMode(IdleMode.kBrake)` |
| `getPIDController()` / `SparkPIDController` | `getClosedLoopController()` / `SparkClosedLoopController` |
| `pid.setReference(...)` | `ctl.setSetpoint(...)` (setReference deprecated) |
| `encoder.setPositionConversionFactor(x)` | `cfg.encoder.positionConversionFactor(x)` |
| `motor.follow(leader)` | `cfg.follow(leaderId[, invert])` |
