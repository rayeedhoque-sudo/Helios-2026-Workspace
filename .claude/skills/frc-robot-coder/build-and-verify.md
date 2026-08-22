# Build & Verify (Windows, this machine)

*How to compile, simulate, and verify FRC Java code on this machine — all facts below were observed on 2026-06-12, not guessed.*

## Environment (verified)

- **No WPILib install needed for building.** The project's Gradle wrapper + JDK 17 is sufficient.
- JDK 17 lives at `C:\Program Files\Java\jdk-17`. Gradle finds it via `org.gradle.java.home` in `C:\Users\Rayeed Hoque\.gradle\gradle.properties` (user-level — do not add Java paths to the team repo).
- Helios project: `C:\Users\Rayeed Hoque\FRC\2026-Helios-Code` (GradleRIO 2026.2.1, Gradle 8.11 wrapper, Java 17 source/target, team 9704).
- **Always quote paths** — the home directory contains a space (`Rayeed Hoque`). Unquoted invocations break.

## Commands (PowerShell)

```powershell
Set-Location "C:\Users\Rayeed Hoque\FRC\2026-Helios-Code"
& .\gradlew.bat build --console=plain          # compile + jar + tests
& .\gradlew.bat simulateJava                   # desktop simulation (GUI — interactive, don't run headless)
& .\gradlew.bat deploy                         # roboRIO deploy — ONLY works on the robot's network
& .\gradlew.bat build --offline                # after first build, works without internet
```

## Observed timings

| Scenario | Time | Notes |
|---|---|---|
| First-ever build | ~2 min (1m 57s observed) | Downloads Gradle 8.11 distribution + all WPILib/vendor deps. Needs internet. |
| Warm build (daemon running) | ~2 s | `BUILD SUCCESSFUL in 2s` |

Expected harmless noise: `Deprecated Gradle features were used... incompatible with Gradle 9.0` warning and an `[Incubating] Problems report` link — both appear on successful builds.

## Verification ladder (cheapest first)

1. **`build` after every code change.** Gradle 8.11 prints Java compile errors at the *end* of output — read from the bottom.
2. **Unit tests** — JUnit 5 is configured but the project has no tests yet (`test NO-SOURCE`). Add under `src/test/java/`. Hardware-free logic (unit conversions, setpoint math, state transitions) is testable; motor I/O is not.
3. **`simulateJava`** — launches the sim GUI + driver station panel (desktop support is enabled in build.gradle). Verifies code runs, commands schedule, no startup crashes. Sim ≠ real hardware: no real currents, loads, or sensor noise.
4. **Real robot** — `deploy` requires the robot network (roboRIO at `10.97.4.2` for team 9704). Cannot be done from this machine alone; never claim hardware behavior works from build/sim alone.

## Common failure signatures

| Error | Likely cause |
|---|---|
| `cannot find symbol` on a vendor class/method | API drift — the pattern used doesn't exist in the installed vendordep version. Check `vendordeps\*.json` versions; fix against project source or official docs. |
| `package com.ctre... / com.revrobotics... does not exist` | Missing/renamed vendordep JSON in `vendordeps\` |
| `Could not resolve all dependencies` | Offline / firewalled — first build and new vendordeps need internet |
| Gradle can't find Java | `org.gradle.java.home` missing from user gradle.properties |

## Characterization & tuning pointers

- SysId (feedforward/PID characterization): https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/
- Simulation docs: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/
