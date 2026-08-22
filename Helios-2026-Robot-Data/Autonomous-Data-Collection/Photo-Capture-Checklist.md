# Photo Capture Checklist

Photos are useful for identifying candidates, but every photo-derived value is `[photo-assumed]` until confirmed by code, vendor readback, or the user.

## Whole Robot

- Front, back, left, right, top-down, and three-quarter views.
- Include a tape measure or known-size object in at least one frame.
- Capture bumper placement and frame perimeter.

## Electrical Board

- One wide shot of the full board.
- Close shots of the PDH/PDP with every breaker rating readable.
- Close shots of roboRIO, radio, VRM/PCM if present, CANivore if present, and main breaker.
- Follow motor controller power leads to breaker channels when possible.
- Capture CAN chain endpoints and termination.

## Motor And Controller Labels

Take a readable label photo for each:

- 4 drive Krakens.
- 4 steer Krakens.
- 4 shooter flywheel Krakens.
- Kicker Kraken.
- Intake roller Kraken.
- Hood SPARK MAX / NEO 550.
- Intake slider SPARK MAX / NEO.
- All hopper SPARK MAX / NEO motors, including the suspected 3rd hopper motor.

## Mechanism Ratios

- Shooter belt/pulley teeth and flywheel wheel diameter.
- Hood gearbox/cartridge stack and sector stage.
- Intake roller chain/belt stages and roller diameters.
- Intake slider rack/pinion or gear train, including tooth counts.
- Hopper gear train and roller diameters.
- Kicker pulley/sprocket tooth counts and wheel diameter.

## Sensors And Vision

- Beam-break sensors and wiring path to DIO.
- Limelight front, side, and mount closeup.
- Limelight lens height reference and pitch angle reference.
- Pigeon 2 mounting orientation, including which side points forward and up.
- CANcoders and magnet mounting if accessible.

## File Naming

Use names that preserve context:

```text
2026-06-16-electrical-board-wide.jpg
2026-06-16-pdh-breakers-close.jpg
2026-06-16-hopper-third-neo-label.jpg
2026-06-16-limelight-side-pitch.jpg
```
