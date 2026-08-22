package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins the CONTINUOUS hood tracking (2026-08-22). The hood angle is a datum captured at
 * enable plus every shortest-path delta since, instead of a fresh interpretation of each
 * raw reading.
 *
 * Why: interpreting one reading through a fixed wrap split flips 56 deg across a single
 * count near the split (raw 103.4 = +52.0 deg, raw 103.6 = -4.3 deg). At the negative
 * reading the travel guard believes the hood is at the bottom and drives UP at full
 * feedforward -- the hood shooting up uncontrollably that the team reported. Relative
 * tracking cannot produce that jump, and the noise deadband stops slow encoder drift from
 * moving the angle at all.
 */
public class HoodTrackingTest {

    private static final double DEG_PER_UNIT = ShooterSubsystemConstants.HOOD_DEG_PER_RAW_UNIT;

    // ---- shortest-path delta: the rollover must be a non-event ----

    @Test
    void rolloverUpIsASmallPositiveDelta() {
        assertEquals(1.5, ShooterSubsystem.shortestRawDelta(359.0, 0.5), 1e-9,
            "359 -> 0.5 is +1.5, not -358.5");
    }

    @Test
    void rolloverDownIsASmallNegativeDelta() {
        assertEquals(-1.5, ShooterSubsystem.shortestRawDelta(0.5, 359.0), 1e-9,
            "0.5 -> 359 is -1.5, not +358.5");
    }

    @Test
    void ordinaryMotionIsUnchanged() {
        assertEquals(5.0, ShooterSubsystem.shortestRawDelta(150.0, 155.0), 1e-9);
        assertEquals(-5.0, ShooterSubsystem.shortestRawDelta(155.0, 150.0), 1e-9);
    }

    /** The exact failure: sweeping across the old split must accumulate smoothly. */
    @Test
    void sweepingAcrossTheOldSplitAccumulatesSmoothly() {
        double[] sweep = { 95.0, 100.0, 103.4, 103.6, 108.0, 115.0 };
        double travelled = 0;
        for (int i = 1; i < sweep.length; i++) {
            double step = ShooterSubsystem.hoodDeltaDegrees(
                ShooterSubsystem.shortestRawDelta(sweep[i - 1], sweep[i]));
            assertTrue(Math.abs(step) < 2.0,
                "no step across the old split may be large; got " + step + " deg");
            travelled += step;
        }
        assertEquals((115.0 - 95.0) * DEG_PER_UNIT, travelled, 0.05,
            "total travel must equal the raw span times the scale");
    }

    // ---- noise and glitch rejection: "slight changes must not affect the hood" ----

    @Test
    void noiseBelowTheDeadbandIsNotTravel() {
        assertEquals(0, ShooterSubsystem.hoodDeltaDegrees(0.08), 1e-9, "measured noise is +-0.08");
        assertEquals(0, ShooterSubsystem.hoodDeltaDegrees(-0.15), 1e-9);
    }

    @Test
    void noiseCannotAccumulateOverManyLoops() {
        double drift = 0;
        for (int loop = 0; loop < 5000; loop++) { // ~100 s at 50 Hz, jittering one way
            drift += ShooterSubsystem.hoodDeltaDegrees(0.08);
        }
        assertEquals(0, drift, 1e-9, "a still hood must never accumulate phantom travel");
    }

    @Test
    void glitchesAboveTheMaxStepAreRejected() {
        assertEquals(0, ShooterSubsystem.hoodDeltaDegrees(90.0), 1e-9,
            "90 units in one loop is a garbled frame, not a 14 deg hood movement");
    }

    @Test
    void realMotionStillCountsBetweenTheTwoFilters() {
        assertEquals(2.0 * DEG_PER_UNIT, ShooterSubsystem.hoodDeltaDegrees(2.0), 1e-9);
    }

    // ---- datum capture: a flipped reading must not seed the tracker ----

    @Test
    void datumClampsAFlippedReadingBackOntoARealStop() {
        // Either side of the old split, both readings are fictions; both must land on a stop.
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP,
            ShooterSubsystem.hoodDatumAngle(103.4), 1e-9, "+52 deg fiction clamps to the up stop");
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN,
            ShooterSubsystem.hoodDatumAngle(103.6), 1e-9, "-4.3 deg fiction clamps to the down stop");
    }

    @Test
    void datumPassesThroughARealReading() {
        assertEquals(ShooterSubsystem.hoodDegreesFromRaw(200.0),
            ShooterSubsystem.hoodDatumAngle(200.0), 1e-9);
    }

    // ---- travel window ----

    @Test
    void windowLimitsTravelFromTheEnablePosition() {
        double base = 10.0;
        double window = ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG;
        assertEquals(Math.min(ShooterSubsystemConstants.MAX_ANGLE, base + window),
            ShooterSubsystem.clampDesiredAngle(999, base, true), 1e-9,
            "cannot be commanded further up than the window allows");
        assertEquals(Math.max(ShooterSubsystemConstants.MIN_ANGLE, base - window),
            ShooterSubsystem.clampDesiredAngle(-999, base, true), 1e-9,
            "cannot be commanded further down than the window allows");
    }

    @Test
    void softLimitsStillApplyWithoutADatum() {
        assertEquals(ShooterSubsystemConstants.MAX_ANGLE,
            ShooterSubsystem.clampDesiredAngle(999, 0, false), 1e-9);
        assertEquals(ShooterSubsystemConstants.MIN_ANGLE,
            ShooterSubsystem.clampDesiredAngle(-999, 0, false), 1e-9);
    }

    // ---- anti-windup: the tracked angle can never leave the physical stops ----

    @Test
    void travelSaturatesAtTheUpStop() {
        double base = 40.0;
        double relative = ShooterSubsystem.accumulateHoodTravel(4.0, base, 10.0);
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP, base + relative, 1e-9,
            "accumulation must saturate at the up stop, not run past it");
    }

    @Test
    void travelSaturatesAtTheDownStop() {
        double base = 5.0;
        double relative = ShooterSubsystem.accumulateHoodTravel(-1.0, base, -40.0);
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN, base + relative, 1e-9,
            "a -32.96 deg reading on a 3.2-44.5 deg mechanism must be impossible");
    }

    @Test
    void ordinaryTravelAccumulatesNormally() {
        assertEquals(7.0, ShooterSubsystem.accumulateHoodTravel(5.0, 20.0, 2.0), 1e-9);
    }

    // ---- feedback validity is now ONLY the dead-encoder test ----

    @Test
    void anyDialPositionIsValidFeedback() {
        // The old absolute band rejected these, which forced 0 V and left the hood dead.
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(103.6), "dead-band raw must be usable now");
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(60.0));
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(359.9));
    }

    @Test
    void deadEncoderIsStillRejected() {
        assertTrue(!ShooterSubsystem.isHoodFeedbackValid(0.0),
            "bit-exact 0.0 is a silent controller, not a position");
    }

    // ---- DPAD nudge: repeated clicks accumulate but stay bounded ----

    /**
     * The nudge steps from the CURRENT SETPOINT, so leaning on the button walks the setpoint
     * up -- and must stop at the window edge rather than running away from the hood.
     */
    @Test
    void repeatedNudgesSaturateAtTheWindowEdge() {
        double base = 10.0;
        double setpoint = base;
        for (int click = 0; click < 100; click++) {
            setpoint = ShooterSubsystem.clampDesiredAngle(
                setpoint + ShooterSubsystemConstants.HOOD_NUDGE_DEG, base, true);
        }
        assertEquals(Math.min(ShooterSubsystemConstants.MAX_ANGLE,
                base + ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG),
            setpoint, 1e-9, "100 clicks up must stop at the window edge");

        for (int click = 0; click < 100; click++) {
            setpoint = ShooterSubsystem.clampDesiredAngle(
                setpoint - ShooterSubsystemConstants.HOOD_NUDGE_DEG, base, true);
        }
        assertEquals(Math.max(ShooterSubsystemConstants.MIN_ANGLE,
                base - ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG),
            setpoint, 1e-9, "100 clicks down must stop at the other edge");
    }

    /** One click must move the setpoint by exactly the step, mid-range. */
    @Test
    void oneClickMovesExactlyOneStep() {
        double base = 20.0;
        assertEquals(20.0 + ShooterSubsystemConstants.HOOD_NUDGE_DEG,
            ShooterSubsystem.clampDesiredAngle(
                20.0 + ShooterSubsystemConstants.HOOD_NUDGE_DEG, base, true), 1e-9,
            "one click moves exactly one step, whatever the step is tuned to");
    }

    /** A datum far outside the soft band must not invert the window into an empty range. */
    @Test
    void degenerateWindowHoldsInsteadOfInverting() {
        double clamped = ShooterSubsystem.clampDesiredAngle(20, 80, true);
        assertTrue(Double.isFinite(clamped), "must produce a real setpoint, not NaN");
        assertTrue(clamped >= ShooterSubsystemConstants.MIN_ANGLE
                && clamped <= ShooterSubsystemConstants.MAX_ANGLE,
            "must stay inside the soft limits: " + clamped);
    }
}
