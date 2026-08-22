*Brownout prevention and electrical-safety reference for Helios (FRC 9704) — the team's #1 reported pain; use when diagnosing reboots, stutters, dimming, or adding any new motor.*

## 1. What Helios has configured TODAY (verified against source 2026-06)

| Load | Motor / controller | CAN ID | Current limit in code | Source |
|---|---|---|---|---|
| Swerve drive ×4 | Kraken X60 / TalonFX | 11,5,2,8 | **YES** — 120 A stator via `kSlipCurrent`; supply = untuned **70 A factory default** (empty `driveInitialConfigs` is a full config → defaults written to device) | `TunerConstants.java:55,59,115` |
| Swerve steer ×4 | Kraken X60 / TalonFX | 10,4,1,7 | **YES** — 40 A stator in `steerInitialConfigs`; supply = 70 A default | `TunerConstants.java:60-67` |
| Shooter flywheel ×4 | Kraken X60 / TalonFX | 13,14,15,16 | **NONE in code** — only `Slot0Configs` applied (partial-group apply leaves device's persisted limits untouched; see note) | `ShooterSubsystem.java:90-93` |
| Shooter hood | NEO 550 / SparkMax | 19 | **NONE** — and `periodic()` drives it `setVoltage(clamp ±12V)` | `ShooterSubsystem.java:39,218`; NEO 550 per `NEO550_ROTATIONS_PER_HOOD_DEGREE` (`SubsystemConstants.java:111-114`) |
| Intake roller | TalonFX (motor model unverified) | 18 | **NONE in code** (no configurator call at all) | `IntakeSubsystem.java:21` |
| Intake slider | SparkMax (brushless; motor type unverified) | 22 | **NONE** | `IntakeSubsystem.java:22` |
| Hopper ×2 | SparkMax brushless (NEO assumed) | 20,21 | **NONE** | `HopperSubsystem.java:19-20` |
| Kicker | TalonFX (motor model unverified) | 17 | **NONE in code** | `HopperSubsystem.java:23` |

Phoenix 6 v26 TalonFX factory defaults: **120 A stator + 70 A supply, both ENABLED**, supply drops to 40 A after 1.0 s of limiting (verified: `CurrentLimitsConfigs` constructor in `wpiapi-java-26.1.0.jar`; https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html). So "NONE in code" ≠ unlimited — **but** the shooter/roller/kicker never write the current-limit group, so whatever was last persisted via Tuner X silently sticks; nothing in code guarantees the defaults. SparkMax rows: nothing in code, and REV does not document a safe default — treat as unprotected. The gap: no **tuned** limit on any mechanism, and 4 × 70 A default shooter supply = 280 A worst case.

## 2. Brownout mechanics — roboRIO staged protection
Source: https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-brownouts.html

| Stage | Battery-rail voltage | What happens | Recovery |
|---|---|---|---|
| 1 | < 6.8 V | 6 V PWM rail output starts dropping | automatic |
| 2 | < 6.3 V (roboRIO 1) / 6.75 V default, software-settable (roboRIO 2) | Brownout protection: PWM outputs disabled, CAN motor controllers sent explicit **disable**, 6V/5V/3.3V user rails off, relay outputs disabled | outputs re-enable only when voltage rises > 7.5 V |
| 3 | < 4.5 V | roboRIO may **black out** (full reboot; tens of seconds dead — boot time is rule of thumb) | normal boot sequence begins above 4.65 V |

Brownout = the roboRIO protecting itself. Motors cutting out in pulses = repeated stage-2 entry/exit across the 6.3→7.5 V hysteresis band.

## 3. Symptom → likely cause

| Symptom | Likely cause |
|---|---|
| Robot fully reboots mid-match | Stage 3 (<4.5 V): huge simultaneous draw, dying battery (high internal resistance), or loose battery terminal/SB50 |
| Robot disables for ~1 s repeatedly; DS log shows brownout | Stage 2 cycling: total draw sags battery below 6.3 V |
| Swerve stutters/pulses under hard acceleration | Drive supply limit is the untuned 70 A default; 4 drives + anything else exceeds budget → stage 2 |
| Lights dim / comms lag when shooter spins up | 4 Krakens with no tuned limits at near-stall during spin-up — the single biggest draw on this robot |
| Radio drops, comms return after ~60-80 s | Voltage dip rebooted the radio — check radio power path and brownout sources |
| Works on fresh battery, dies on second match with same battery | Battery internal resistance too high; retire it |
| Main breaker trips | Sustained >120 A — current limits missing or too high |
| Hood NEO 550 / slider (type unverified) gets hot or smokes | Stalled with no `smartCurrentLimit` (NEO 550 stall = 100 A — REV: https://docs.revrobotics.com/brushless/neo/550; stall-survival time not vendor-stated, assume seconds) |

## 4. Worked current budget for Helios

Physics: loaded voltage ≈ 12.5 V − I_total × R_system. Healthy battery + wiring R_system ≈ 0.02 Ω.
→ Stage-2 brownout (6.3 V) when total draw ≈ (12.5−6.3)/0.02 ≈ **310 A**. Stage-3 blackout ≈ 400 A.
Main breaker: 120 A continuous (tolerates short bursts well above that, then trips).

Kraken X60 stall current: **366 A** (trapezoidal) / **483 A** (FOC) per motor; WCP does not state a test voltage (https://docs.wcproducts.com — Kraken X60 motor performance page). With Phoenix 6 factory defaults active, each TalonFX is capped at 70 A supply — **if** the devices are at factory defaults, which nothing in code guarantees. Even then: 4 shooter Krakens commanded `VelocityVoltage` from rest ≈ **280 A** supply for up to 1 s (until the 40 A lower limit engages); add hard-accel drive and you blow past the ~310 A stage-2 line. **The math does not work without explicit, lower limits.**

Recommended per-motor limits (peak supply budget; tune stator down only if mechanism still works):

| Mechanism | Stator limit (A, torque cap) | Supply limit (A, battery cap) | Burst supply total |
|---|---|---|---|
| Drive ×4 | 120 (keep existing slip) | **60 each — ADD** | 240 |
| Steer ×4 | 40 (existing) | — | ~40 |
| Shooter ×4 | **80 each** | **30 each** | 120 |
| Intake roller | 60 | 30 | 30 |
| Kicker | 40 | 20 | 20 |
| Hopper ×2 (NEO) | smartCurrentLimit 40 each | — | 80 |
| Hood (NEO 550) | smartCurrentLimit 20 | — | 20 |
| Slider (SparkMax) | smartCurrentLimit 20 | — | 20 |

Roller/kicker rows assume Kraken-class TalonFX motors (model not determinable from code). NEO 550 = 20 A and NEO = 40 A smart limits are community-standard values from REV locked-rotor guidance, not vendor-published numbers.

Even with these limits, hard-accel (240 A) + shooter spin-up (120 A) overlaps past the ~310 A line → **stagger** (Section 5.4). Rule of thumb: keep any instantaneous total under ~250 A.

## 5. Fixes, in priority order

### 5.1 Current limits (biggest win — do first)
TalonFX (Phoenix 6 26.1.0 — every `with*` overload below incl. `Amps.of()` javap-verified in `wpiapi-java-26.1.0.jar`; in-repo pattern `TunerConstants.java:60-67`; guidance: https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html). Apply to **each** of the 4 shooter motors — followers enforce their own device-level configs:
```java
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import static edu.wpi.first.units.Units.Amps;

var limits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Amps.of(80))       // A, torque cap
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(Amps.of(30))       // A, battery-side cap
    .withSupplyCurrentLimitEnable(true);
// Breaker protection: SupplyCurrentLowerLimit / SupplyCurrentLowerTime reduce the
// supply cap after sustained limiting (defaults 40 A after 1.0 s; same configs class).
shooterA.getConfigurator().apply(limits);  // repeat for B, C, D
```
SparkMax (REVLib 2026.0.3 — `configure()`, `smartCurrentLimit(int)`, and both enum imports javap-verified in `REVLib-java-2026.0.3.jar`; usage doc: https://docs.revrobotics.com/revlib/spark/configuring-a-spark):
```java
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;    // TOP-LEVEL imports — the nested SparkBase.ResetMode/PersistMode
import com.revrobotics.PersistMode;  // configure() overload is @Deprecated(forRemoval) in 2026.0.3

SparkMaxConfig config = new SparkMaxConfig();
config.smartCurrentLimit(20); // A — NEO 550: 20 A; NEO: 40 A (community-standard values)
// CAUTION: kResetSafeParameters wipes settings persisted via the REV Hardware Client (e.g. inversion)
// on devices never configured from code — see revlib-patterns.md for the kNoResetSafeParameters fallback.
shooterAngle.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```
Also add a supply limit to the empty `driveInitialConfigs` in `TunerConstants.java:59`. Full patterns: see `phoenix6-patterns.md` / `revlib-patterns.md`.

### 5.2 Ramp rates (smooths the spin-up spike)
Phoenix 6 (confirmed: https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/ClosedLoopRampsConfigs.html and OpenLoopRampsConfigs.html):
```java
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
// Shooter uses VelocityVoltage (closed-loop voltage) -> Voltage*ClosedLoop* field
shooterA.getConfigurator().apply(new ClosedLoopRampsConfigs()
    .withVoltageClosedLoopRampPeriod(0.3)); // s to ramp 0 V -> 12 V; range 0-1 s
```
REVLib: `config.openLoopRampRate(double seconds)` / `config.closedLoopRampRate(double seconds)` — javap-verified on `SparkBaseConfig` in `REVLib-java-2026.0.3.jar`.

### 5.3 Voltage "compensation"
Phoenix 6: prefer voltage-based control (`VoltageOut`, `VelocityVoltage` — already used in `ShooterSubsystem.java:43`); requesting volts instead of duty cycle is inherently battery-sag-tolerant. REVLib: `config.voltageCompensation(11.0 /* V nominal */)` — javap-verified on `SparkBaseConfig` in `REVLib-java-2026.0.3.jar` (undo with `disableVoltageCompensation()`).

### 5.4 Stagger spin-up
Don't command shooter spin-up in the same instant as full-throttle drive or intake deploy. Sequence it: `Commands.sequence(spinUpShooter, Commands.waitSeconds(0.3), deployIntake)`. With flywheel inertia, 0.5-1.5 s of ramped spin-up costs nothing in cycle time.

### 5.5 Battery health
Measure internal resistance (Battery Beak or charger readout): fresh ≈ 0.011-0.015 Ω; **retire/practice-only above ~0.020 Ω** (rule of thumb, not a vendor spec). Load-test capacity yearly. Torque battery lugs; inspect SB50 for heat discoloration.

### 5.6 Wire gauge + breaker sizing sanity
Per FRC manual norms (verify current-season rules): 40 A breaker → ≥12 AWG; 30 A → ≥14 AWG; 20 A → ≥18 AWG. Krakens on 40 A breakers; NEO 550 fine on 20-30 A. A breaker bigger than the wire it feeds = fire risk; a breaker that trips before the software limit = wrong software limit.

## 6. Commanded vs actual — the bug class

`motor.get()` / `getAppliedOutput()` = what you **asked for** (duty cycle). Only current tells you what's **happening** under load.

| API | Returns | Verified |
|---|---|---|
| SparkMax `.get()` | last commanded speed (duty cycle), `double` | in-repo use `IntakeSubsystem.java:63`; javap `REVLib-java-2026.0.3.jar` |
| SparkBase `.getAppliedOutput()` | applied output duty cycle, `double` — still not load | javap `REVLib-java-2026.0.3.jar` |
| SparkBase `.getOutputCurrent()` | **measured output current, A**, `double` | javap `REVLib-java-2026.0.3.jar` |
| TalonFX `.getStatorCurrent()` | `StatusSignal<Current>` — measured stator A | javap `wpiapi-java-26.1.0.jar` (CoreTalonFX) |
| TalonFX `.getSupplyCurrent()` | `StatusSignal<Current>` — measured battery-side A | same |

**In-repo bug** — `IntakeSubsystem.isIntakeSliderStall()` (`IntakeSubsystem.java:62-64`) reads `intakeSliderMotor.get()`, the commanded duty cycle: `|get()| < STALL_SPEED && get() > 0` with `STALL_SPEED = 0.05` (`SubsystemConstants.java:58`). Commands hold ±0.5 or −0.25 (fails the `< 0.05` clause) and at rest `get()` = 0 (fails `> 0`) — the condition can **never** be true. So `intakeCommand`/`outtakeCommand`/`stowCommand` always run to their 1 s timeouts, and `stowSliderCommand` (no timeout race, `IntakeSubsystem.java:129-141`) **hangs forever**. Fix:
```java
public boolean isIntakeSliderStall() {
    // Stalled = commanded to move but drawing high current. NEO 550 w/ 20 A limit: ~15 A threshold.
    return Math.abs(intakeSliderMotor.get()) > 0.05
        && intakeSliderMotor.getOutputCurrent() > 15.0; // A, measured
}
```
Debounce it (current spikes on direction change): require the condition for ~0.25 s — `new edu.wpi.first.math.filter.Debouncer(0.25)` + `.calculate(boolean)`, javap-verified in `wpimath-java-2026.2.1.jar`.

## 7. Pre-competition power audit (10 items)

1. Beak/measure internal resistance of every battery; label and retire any > ~0.020 Ω.
2. Battery lugs torqued, no corrosion, SB50 seats firmly, main breaker bolts tight.
3. Grep the code: every `TalonFX` gets a `CurrentLimitsConfigs` applied; every `SparkMax` gets `smartCurrentLimit` + `configure(..., kPersistParameters)`.
4. Shooter (all 4 Krakens) has **explicit** stator + supply limits applied in code AND a closed-loop ramp — the #1 Helios risk (today it inherits whatever is persisted on the devices).
5. `driveInitialConfigs` in TunerConstants has an explicit supply limit (e.g. 60 A) — don't rely on the untuned 70 A factory default.
6. Hood NEO 550 + slider (motor type unverified; treat as NEO 550-class) limited to 20 A; stall detection uses `getOutputCurrent()`, not `.get()`.
7. Wire gauge matches breaker size on every PDP/PDH channel; no pinched motor leads.
8. Log battery voltage + per-motor supply current to telemetry; review after every practice match.
9. After each match, open Driver Station Log Viewer and check for brownout events (they're flagged).
10. Brownout drill on a practice battery: full-accel sprint + shooter spin-up + intake simultaneously while watching the voltage trace — must stay above ~7.5 V; if not, lower supply limits or stagger.
