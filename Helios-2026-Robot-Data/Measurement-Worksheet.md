# Helios 2026 — Measurement & Confirmation Worksheet

Fill these in at the robot (or from the game manual / team knowledge) and hand them back — every one is a **concrete value** that CAD/code couldn't give but you *can*. Grouped by the tool you need. CAD's current guess is in parentheses; confirm or correct it.

> Not here on purpose: PID/FF gains, the shooting lookup values, motor inversions, CANcoder zero offsets — those only come from **running/tuning** the robot, not from a measurement. See §G.

## V2 — measure / read these first (2026-09-29 redesign)
- [ ] **CAN IDs** (Tuner X / REV HW Client): flywheel L1 ____ , L2 ____ , R ____ , feeder ____ , hood X44 ____ , intake roller X60 ____ ; hopper A/B still 20/21? ____ ; is a leftover SPARK MAX (old hood, 19) still on the bus? ____
- [ ] **Hood sector teeth** ____ (CAD 170T, 10 DP pinion 10T) and **hood travel** min ____° / max ____° (no CAD mate limits)
- [ ] **Hood encoder?** ____ (what's on the 23T pulley on the hood pinion shaft?) and its homing reference ____
- [ ] **Hopper NEO pinion teeth** ____ (CAD geometry says 12T 20 DP)
- [ ] Flywheel ratio spot-check: 18T motor pulleys → 26T flywheel pulleys ____ (confirm)
- [ ] Feeder tube-roller OD ____ ; upper (flywheel-driven) roller OD ____
- [ ] Limelight lens height / offsets / pitch **in the V2 launcher** ____ ; shot release height ____
- [ ] Swerve module type on the robot: WCP X2 or X3? ____ (CAD says X2)
- [ ] Orange Pi 5 on the robot? ____ role ____

---

## A. Tape measure / calipers
- [ ] Frame outer size: L ____ × W ____  (CAD ≈ 38.5" × 24" / 0.978 × 0.61 m)
- [ ] Bumper thickness ____ ; bumpers centered on frame? offset ____ (CAD assumes centered)
- [ ] Drive wheelbase (front↔back, wheel center to center) ____  (CAD 26.75")
- [ ] Drive track width (left↔right) ____  (CAD 16.75")
- [x] ~~Intake slider total stroke~~ — **CAD says ~11.2"** (mate limit 2"→13.207"). Just spot-check it on the robot; still need rack-vs-geared (for inches/motor-rot).
- [ ] Flywheel wheel diameter ____ (CAD 4") ; ball compression gap ____
- [ ] Intake roller wheel diameters ____ (CAD 4" mecanum, 3" compliant)
- [ ] ~~Hopper roller diameters (V1)~~ V2 hopper is a belt floor (15T HTD5 drive pulleys)
- [ ] ~~Kicker wheel diameter~~ (V1 kicker removed — see V2 feeder above)

## B. Protractor / digital angle gauge
- [ ] **V2 hood travel — see V2 section** *(V1:)* ~~Hood min/max angle~~ — CAD confirmed ~42° of sweep (mate limit; matches code 3.224°→44.5°). Just confirm the zero reference physically.
- [ ] Is the intake slider **rack-and-pinion** or **geared**? ____  (count pinion teeth if rack: ____)
- [ ] Limelight mount **pitch** angle ____° (roll: **mounted upside down — 180°**, user-stated 2026-07-17; LL OS appears to flip in firmware, see Hardware-Data-Sheet §3)

## C. Heights & positions — measure from the floor and from robot center (unlocks the shooting + vision math)
- [ ] Shooter ball **release height** off the floor ____
- [ ] Hood pivot location: height off floor ____ , forward offset from robot center ____
- [ ] Limelight lens: height off floor ____ , forward offset ____ , lateral offset ____
- [ ] Pigeon 2 mount location + **orientation** (which axis points forward / up — for mount-pose config)

## D. Read off labels / Phoenix Tuner X / REV Hardware Client (concrete, no running needed)
- [ ] Confirm every CAN ID 1–23 matches [CAN-Bus-and-DIO-Map.md](CAN-Bus-and-DIO-Map.md)
- [x] ~~**3rd hopper NEO 2.0**~~ (V1 only) — its CAN ID ____ and is it geared with the other two or separate?
- [ ] **2nd Pigeon 2.0** — CAN ID ____ , or confirm it's not on the robot
- [ ] Motor model per location (Kraken X60 vs X44; NEO vs NEO 550 vs NEO 2.0) — read the labels
- [ ] **Intake slider MAXPlanetary cartridge ratio** ____ (V2 CAD: 5:1 + 4:1 = 20:1, team says CAD is right) — confirm like you did the hood
- [x] ~~Hopper "gearbox goon"~~ (V1 only): confirm 10T→60T (6:1) + 18T→36T (2:1); which stage drives the rollers ____
- [x] ~~Kicker reduction~~ (V1 only) — count the pinion + final belt/pulley teeth ____

## E. Electrical (validates the current-limit caps in §7) — read off the PDH/PDP and wiring
- [ ] Breaker size on each motor channel ____ A (Krakens should be 40 A; NEO 550 20–30 A)
- [ ] Wire gauge per motor ____ (40 A → ≥12 AWG, 30 A → ≥14, 20 A → ≥18)
- [ ] Are the **DIO 8/9 indexer beam-breaks physically installed**? Type ____ (through-beam? retroreflective?)
- [ ] Battery count + internal resistance per battery (Battery Beak) ____ Ω (retire > ~0.020)

## F. From the 2026 game manual (concrete, no robot needed)
- [x] ~~AprilTag IDs~~ — **fully resolved** (2026 REBUILT, WPILib field JSON): RED HUB 2,3,4,5,8,9,10,11 · BLUE HUB 18,19,20,21,24,25,26,27 · RED TRENCH 1,6,7,12 · BLUE TRENCH 17,22,23,28. Nothing left here.
- [ ] Field heights/distances that feed the shooting model (goal height, typical shot distances)

---

## G. NOT measurable — these come from bring-up/tuning (don't try to pre-fill)
- Motor **inversions / directions** — test each, flip the boolean.
- **CANcoder zero offsets** — produced by the Tuner X "set wheel offsets" procedure (wheels pointed forward); concrete *after* the procedure, not before.
- **PID/FF gains** for every closed-loop mechanism — tune live.
- The **distance → (flywheel speed, hood angle)** shooting table — built empirically (the §C geometry gives you a physics starting point, then you tune).
- **Robot mass** — you can't weigh it; leave at 63.5 kg (TODO). Fine to defer.

---
*Hand back whatever you can fill; partial is fine. The §C geometry + §F field data are the highest-leverage because they turn the shooting model from blind trial-and-error into a real calculation.*
