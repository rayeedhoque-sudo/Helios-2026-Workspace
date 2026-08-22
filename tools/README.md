# FRC Tools (Helios 2026 — Team 9704)

Portable FRC tool files bundled with the project. **Read this before assuming the tools "just run" from here** — some FRC tools are installed Windows apps, not portable files.

## In this folder (portable — usable on any machine)

| File | What it is | Use |
|---|---|---|
| `ni-frc-2026-game-tools_26.0_online.exe` | **FRC Game Tools 2026 installer** (NI online installer, v26.0) | Run on a new machine to install the **FRC Driver Station** + **roboRIO Imaging Tool** + NI runtimes. (Online installer — needs internet during install.) |
| `roborio-images/FRC_roboRIO_2026_v1.2.zip` | roboRIO **1.0** firmware image, 2026 **v1.2** | Used by the Imaging Tool to image a roboRIO 1.0 |
| `roborio-images/FRC_roboRIO2_2026_v1.2.img.zip` | roboRIO **2.0** SD-card image, 2026 v1.2 | Flash to a microSD for a roboRIO 2.0 |
| `PathPlanner-Windows-v2026.1.2-setup.exe` | **PathPlanner 2026.1.2** installer | Design autonomous paths (no robot needed) — matches PathPlannerLib 2026.1.2 in the code |

> Our roboRIO is **already imaged** (done from another laptop), so the images are here for re-imaging / spares / handoff, not a required step.

## Installed apps — NOT copyable into this folder (here's where they live)

These are registry/service-bound NI installs (~600 MB) — copying the folders does **not** make them work. To get them on another machine, run the **Game Tools installer** above.

| Tool | Installed path |
|---|---|
| FRC Driver Station | `C:\Program Files (x86)\FRC Driver Station\DriverStation.exe` (+ `DS_LogFileViewer.exe`) |
| roboRIO Imaging Tool | `C:\Program Files (x86)\National Instruments\LabVIEW 2025\project\roboRIO Tool\roboRIO_ImagingTool.exe` |

## Must download/install yourself — can't be bundled in this folder
None of these are installed yet. They can't go "in the folder": WPILib is a 2.4 GB ISO, Tuner X is a Store-only app, and they all install to system locations.

| Tool | Why you need it | How to get it |
|---|---|---|
| **WPILib 2026** (VS Code + GradleRIO + JDK + extension) | **The build/deploy toolchain** — nothing compiles or deploys without it. Install this FIRST. | Download the ~2.4 GB installer from <https://github.com/wpilibsuite/allwpilib/releases> (follow <https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html>). Mount the ISO, run `WPILibInstaller`. |
| **Phoenix Tuner X** | Generates the swerve project (`TunerConstants`); later zeroes CANcoders | **Microsoft Store** → search "Phoenix Tuner X" → Get. (No standalone installer.) |
| **REV Hardware Client 2** | SPARK MAX / NEO config. **REVLib 2026 needs RHC2**, not the old client. | <https://docs.revrobotics.com/rev-hardware-client/gs/install> |

- A standalone **JDK 17** installer is in `C:\Users\Rayeed Hoque\Downloads\jdk-17.0.12_windows-x64_bin.exe`, and Java 17 is already on PATH — but WPILib bundles its own JDK, so you usually don't need this.

## Quick setup on a fresh machine
1. Run `ni-frc-2026-game-tools_26.0_online.exe` → installs Driver Station + Imaging Tool.
2. Install **WPILib 2026** (separate download) → build/deploy toolchain.
3. Radio + roboRIO network details: see the project `CLAUDE.md` → "Networking / Radio & Deploy".
