*Read whole-robot, electrical-board, and wiring-harness photos: map what's connected to what, sanity-check power wiring, and cross-check against the spec. This is system-level photo reading — for identifying a single device, use [hardware-identification.md](hardware-identification.md) first.*

## When to use this file

- A wide shot of the whole robot or a populated electrical board.
- A photo of the PDH/PDP with wires landed in channels.
- A "is this wired right?" / "why does it brown out?" question with a picture.
- Any time you need to relate multiple devices to each other, not just name one.

**Doctrine (unchanged):** every value read from a photo is `[assumed]` — you can misread a channel, a breaker, or a wire color. A wiring photo yields *candidates* to confirm, never `[verified]` facts. **NEVER** write code (CAN IDs, current limits, polarity) from a wiring photo alone. Photos do not show software config.

## 1. PDH / PDP mapping from a photo

Identify the panel first (PDH vs PDP cues in [hardware-identification.md](hardware-identification.md) §1 — don't duplicate them here), then:

- **Count channels and read breaker ratings.** Snap-Action breakers are stamped (e.g. `40`, `30`, `20`). REV PDH: 20 high-current (40 A max) + 3 low-current (15 A) + 1 switchable. CTRE PDP: 16 channels.
- **Trace each motor lead to its channel.** Follow the two power leads from a motor/controller to the channel pair they land in. Record as a candidate `motor → channel → breaker`.
- **Reconcile against the spec.** Compare to the per-motor breaker/channel rows in `docs/robot-spec.md`. A mismatch (motor in the spec but not on the board, or vice-versa) is a finding to surface.

## 2. CAN bus topology

- **Daisy-chain order.** Note the visible order of CAN devices along the bus (roboRIO/PDH → … → last device). This is the *physical* order; CAN IDs are *software* — map visible order to IDs only as **candidates**, never assert.
- **Termination.** Look for a 120 Ω terminator (or the PDH's switchable CAN term) at the end of the chain. A CANivore provides its own termination. Missing termination → intermittent CAN errors; flag it.
- **Bus identity.** roboRIO bus vs CANivore bus changes the device constructor (`new CANBus("")` vs a named bus). Confirm with the user — not visible in a photo.

## 3. Connection-map output format

Produce a single "what's connected to what" table; every cell `[assumed]` until confirmed against code/CAD/the user:

| Device | CAN ID (candidate) | PDH/PDP channel | Breaker | Wire gauge | Notes |
|---|---|---|---|---|---|
| Kraken (shooter A) | ? — read label / ask | 4 | 40 A | 10 AWG | confirm against `Constants` |

Cross-check the result against `docs/onshape-config.json` / `docs/robot-spec.md` / source before using any of it.

## 4. Cross-checks & red flags

- **Breaker larger than the wire can carry** → fire risk. Surface immediately and link the gauge/breaker guidance in [power-and-safety-checklist.md](power-and-safety-checklist.md).
- **High-draw motor on a channel with no current limit in code** → brownout contributor (Helios has known power problems). Cross-reference the motor's current-limit status in the spec.
- **Motor count on the board ≠ subsystem motor count in the spec** → a phantom or missing mechanism; investigate.
- **Loose/backed-out Wago/Weidmuller or chafed insulation** visible → note it; electrical, not code.

## 5. What wiring photos still can NEVER tell you

Reaffirming [hardware-identification.md](hardware-identification.md) §4 for the system level:

- **Software CAN IDs and CAN bus name** (the photo shows order, not IDs).
- **Configured current limits** (a breaker is hardware; the firmware/software limit is separate — and the thing that actually prevents brownouts).
- **Sensor logic polarity** (NO vs NC, inverted motors) — design intent, not visible.

All of these come from the user or the code, per the interview script in [robot-spec-template.md](robot-spec-template.md).
