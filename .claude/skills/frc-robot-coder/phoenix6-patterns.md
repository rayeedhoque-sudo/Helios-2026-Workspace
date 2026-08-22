*CTRE Phoenix 6 v26 TalonFX (Kraken/Falcon) patterns, grounded in 2026-Helios-Code source and official CTRE docs — use these exact API shapes, not training-data Phoenix 5/early-v6 forms.*

Project root: `C:\Users\Rayeed Hoque\FRC\2026-Helios-Code` (Phoenix6 vendordep **26.1.0** — `vendordeps/Phoenix6-26.1.0.json`). Paths below are relative to root. "jar-verified" = member confirmed via `javap` against the exact compile-classpath jar `~/.gradle/caches/modules-2/files-2.1/com.ctre.phoenix6/wpiapi-java/26.1.0/.../wpiapi-java-26.1.0.jar`.

## Imports (project-confirmed unless noted)
```java
import com.ctre.phoenix6.hardware.TalonFX;              // ShooterSubsystem.java, IntakeSubsystem.java
import com.ctre.phoenix6.configs.Slot0Configs;          // ShooterSubsystem.java
import com.ctre.phoenix6.configs.TalonFXConfiguration;  // TunerConstants.java (via configs.*)
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;  // TunerConstants.java (via configs.*)
import com.ctre.phoenix6.controls.VelocityVoltage;      // ShooterSubsystem.java
import com.ctre.phoenix6.controls.Follower;             // ShooterSubsystem.java
import com.ctre.phoenix6.signals.MotorAlignmentValue;   // ShooterSubsystem.java
import com.ctre.phoenix6.controls.DutyCycleOut;         // jar-verified
import com.ctre.phoenix6.controls.MotionMagicVoltage;   // jar-verified
import com.ctre.phoenix6.configs.MotorOutputConfigs;    // jar-verified
import com.ctre.phoenix6.signals.NeutralModeValue;      // jar-verified (constants: Coast, Brake)
import com.ctre.phoenix6.signals.InvertedValue;         // jar-verified (CounterClockwise_Positive, Clockwise_Positive)
import com.ctre.phoenix6.StatusCode;                    // jar-verified
import com.ctre.phoenix6.StatusSignal;                  // jar-verified
import edu.wpi.first.wpilibj.DriverStation;             // CommandSwerveDrivetrain.java
import edu.wpi.first.units.measure.Current;             // TunerConstants.java (via measure.*)
import static edu.wpi.first.units.Units.*;              // TunerConstants.java — gives Amps.of(...)
```

## 1. Current limits — TalonFXConfiguration + CurrentLimitsConfigs
v26 **factory defaults already enable** limits: stator 120 A, supply 70 A, supply lower limit 40 A after 1.0 s
(https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html — values jar-verified in the `CurrentLimitsConfigs()` constructor).
Helios state: mechanism TalonFXs apply **no** CurrentLimitsConfigs (ShooterSubsystem applies only Slot0Configs; IntakeSubsystem applies nothing) → all run the generous defaults. Swerve already has: steer 40 A stator (`steerInitialConfigs`), drive `kSlipCurrent = Amps.of(120)` via `withSlipCurrent` — jar-verified: `SwerveModule` writes SlipCurrent into the drive motor's `CurrentLimits.StatorCurrentLimit` (enabled) AND `TorqueCurrent.PeakForward/ReverseTorqueCurrent` (`src/main/java/frc/robot/Constants/TunerConstants.java:55`). Brownout fix = tighten mechanism limits, especially the 4-Kraken flywheel (4 × 70 A supply default = 280 A surge).

- **Stator limit** = torque/heat at the motor: caps stall torque, wheel slip, mechanism damage (https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html).
- **Supply limit** = battery-side draw: prevents brownouts and breaker trips. Set supply < stator.

| Mechanism (Helios) | Stator (A) | Supply (A) | Why |
|---|---|---|---|
| Swerve drive (Kraken) | 120 (= kSlipCurrent, already set) | 60–70 | stator tuned to wheel-slip point; supply guards 40 A breakers + battery |
| Swerve steer | 40 (already set) | 20–30 | low torque need; TunerConstants comment: low stator "to help avoid brownouts" |
| Flywheel (4x Kraken, IDs 13–16) | 70–80 | 30–40 each | spin-up is the brownout event; ×4 motors multiplies supply draw |
| Intake roller (TalonFX ID 18) | 40–60 | 25–30 | jams stall the roller; stator caps heat, supply guards breaker |
| Pivot / arm (Motion Magic) | 40–60 | 30 | survives hard-stop stalls without stripping the mechanism |

Table values are engineering guidance (starting points to tune), NOT vendor-documented numbers — only the factory-defaults row above is doc/jar-verified.

```java
var cfg = new TalonFXConfiguration();   // fresh object == factory defaults
cfg.CurrentLimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Amps.of(60))        // A — torque/heat cap
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(Amps.of(30))        // A — battery/breaker cap
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLowerLimit(Amps.of(20))   // A — drop to this...
    .withSupplyCurrentLowerTime(Seconds.of(1.0)); // ...after this long at the limit
```
`with*` builders accept unit types (`Current`/`Time`) or raw doubles (A / s). Fluent style matches `TunerConstants.java:60-67`.

## 2. Control requests + setControl idiom
Create requests **once as fields**, mutate with `with*`, pass to `setControl` (mutable/reusable, avoids GC — https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/control-requests.html). `setControl` returns a `StatusCode` (`StatusCode.OK` on success).
```java
private final DutyCycleOut m_duty = new DutyCycleOut(0.0);       // output: [-1, 1] duty cycle
private final VelocityVoltage m_velocity = new VelocityVoltage(0); // velocity: rotor rps (ShooterSubsystem.java:43)
private final MotionMagicVoltage m_mm = new MotionMagicVoltage(0); // position: rotations

motor.setControl(m_duty.withOutput(0.25));        // 25% duty — open loop
m_velocity.Slot = 0;                              // use Slot0 gains (ShooterSubsystem.java:212)
motor.setControl(m_velocity.withVelocity(motorRps));   // rps — ShooterSubsystem.java:214
motor.setControl(m_mm.withPosition(targetRotations));  // rotations — needs MotionMagicConfigs below
```
Motion Magic profile lives in config, not the request (https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html):
```java
cfg.MotionMagic.MotionMagicCruiseVelocity = 80;   // rps
cfg.MotionMagic.MotionMagicAcceleration   = 160;  // rps/s
cfg.MotionMagic.MotionMagicJerk           = 1600; // rps/s/s (optional smoothing)
```

## 3. Follower — v26 signature (API DRIFT)
v26 takes a **MotorAlignmentValue**, NOT the old boolean `opposeMasterDirection`. Exact project usage (`ShooterSubsystem.java:91-93`):
```java
shooterB.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Aligned)); // spins same direction
shooterC.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed)); // spins opposite
```
Per the Follower javadoc (https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/controls/Follower.html), the follower copies only the leader's **output** (DutyCycle/Voltage/TorqueCurrent); `Aligned` = match leader's configured invert, `Opposed` = oppose it. Configs do NOT propagate — apply current limits / neutral mode to **every** follower individually.

## 4. Slot0Configs — PID + FF
Project fluent pattern (`ShooterSubsystem.java:44-48`, values from `SubsystemConstants.ShooterSubsystemConstants`):
```java
private final Slot0Configs shooterVelConfigs = new Slot0Configs()
    .withKP(0.4).withKD(0.01).withKV(0.12625);   // flywheel velocity loop, Voltage output
shooterA.getConfigurator().apply(shooterVelConfigs);  // apply() accepts a single config group
```
Voltage-output gain units (https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html):

| Gain | Units | Notes |
|---|---|---|
| kS | V | static friction; `withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)` per TunerConstants.java:30 |
| kV | V/rps | ≈ 12 / free-speed-rps; Helios flywheel uses 0.12625 |
| kA | V/(rps/s) | accel feedforward |
| kP | V per rot (position) or per rps (velocity) error | |
| kI / kD | V/(rot·s) / V/rps | usually 0 / small |
| kG | V | gravity comp; `withGravityType(GravityTypeValue.Arm_Cosine)` — jar-verified (also `Elevator_Static`); `import com.ctre.phoenix6.signals.GravityTypeValue` |

## 5. Reading REAL current — StatusSignal pattern
`motor.get()` returns the **commanded duty cycle**, not a measurement. Helios bug (`src/main/java/frc/robot/subsystems/misc/IntakeSubsystem.java:62-64`):
```java
// BUG — compares commanded output to a threshold; can never detect a physical stall:
return Math.abs(intakeSliderMotor.get()) < IntakeSubsystemConstants.STALL_SPEED && (intakeSliderMotor.get() > 0);
```
Correct TalonFX pattern (https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html). Cache the signal once; refresh before reading:
```java
private final StatusSignal<Current> m_statorCurrent = motor.getStatorCurrent(); // torque-side: spikes on stall
private final StatusSignal<Current> m_supplyCurrent = motor.getSupplyCurrent(); // battery-side: brownout telemetry

// in periodic() — refresh then read (getValueAsDouble() = canonical units, here Amps):
m_statorCurrent.refresh();                         // or BaseStatusSignal.refreshAll(sig1, sig2, ...)
double statorAmps = m_statorCurrent.getValueAsDouble();   // A
boolean stalled = statorAmps > 30.0                       // A — tune per mechanism
    && Math.abs(motor.getVelocity().getValueAsDouble()) < 1.0; // rps ≈ 0 → actually stalled
```
`.getValueAsDouble()` matches the project idiom (`ShooterSubsystem.java:131`). Note: the buggy slider motor is a REV SparkMax — its fix is `getOutputCurrent()`, verified via javap against the project's `REVLib-java-2026.0.3.jar` (`SparkBase.getOutputCurrent()` returns `double` amps; `SparkMax extends SparkBase`). See the REVLib reference file.

## 6. Config-apply idiom — once in constructor, retry on failure
`getConfigurator().apply(...)` is **blocking** — never call it unconditionally in a loop. (Helios caution: `ShooterSubsystem.java:283-290` re-applies inside `periodic()` when Shuffleboard gain entries change — guarded, so tolerable for tuning, but don't copy it for normal config.) Apply the full `TalonFXConfiguration` once in the subsystem constructor; a fresh object starts at factory defaults so unset fields are reset (https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/configuration.html). jar-verified: `apply()` overloads exist for `TalonFXConfiguration`, `Slot0Configs`, `CurrentLimitsConfigs`, `MotorOutputConfigs`, all returning `StatusCode`.
```java
StatusCode status = StatusCode.StatusCodeNotInitialized;  // jar-verified constant
for (int i = 0; i < 5; ++i) {
    status = motor.getConfigurator().apply(cfg);
    if (status.isOK()) break;                              // jar-verified: StatusCode.isOK()
}
if (!status.isOK()) {
    DriverStation.reportError("TalonFX " + motor.getDeviceID() + " config failed: " + status, false);
}
```

## 7. Brake / coast — MotorOutputConfigs
Field `NeutralMode` of type `NeutralModeValue`, builder `withNeutralMode` — fields, builders, and enum constants all jar-verified (https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html):
```java
cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;            // holds position when neutral (pivots, intakes); factory default = Coast (jar-verified)
cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;     // other constant: CounterClockwise_Positive (default)
// fluent: new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)  // flywheels: coast to protect gearbox
```
Brake = mechanisms that must hold (pivot/arm/slider). Coast = flywheels (braking 4 Krakens dumps energy and regen current).
