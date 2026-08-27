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

    /**
     * An arbitrary commanded step, in degrees. These tests exercise clampDesiredAngle, which
     * does not care HOW a setpoint was asked for -- the DPAD steps the hood open loop and
     * stops on the measured angle (HOOD_STEP_DEG), so a local constant keeps the clamp tests
     * independent of that tuning. clampDesiredAngle is also what bounds a DPAD step's target.
     */
    private static final double STEP = 2.0;

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
        // Either side of the split the map still has a cliff -- that is unavoidable with a
        // fixed split. What matters is WHERE it sits: re-anchored 2026-08-24 the split is 325,
        // inside the ~69-unit arc the hood never occupies, instead of 103.5 which cut through
        // live travel. Both readings here are fictions; both must land on a stop.
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP,
            ShooterSubsystem.hoodDatumAngle(324.9), 1e-9, "+49 deg fiction clamps to the up stop");
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN,
            ShooterSubsystem.hoodDatumAngle(325.1), 1e-9, "-1.7 deg fiction clamps to the down stop");
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
        assertEquals(base,
            ShooterSubsystem.clampDesiredAngle(-999, base, true), 1e-9,
            "can never be commanded BELOW the enable-time datum");
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
                setpoint + STEP, base, true);
        }
        assertEquals(Math.min(ShooterSubsystemConstants.MAX_ANGLE,
                base + ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG),
            setpoint, 1e-9, "100 steps up must stop at the window edge");

        for (int click = 0; click < 100; click++) {
            setpoint = ShooterSubsystem.clampDesiredAngle(
                setpoint - STEP, base, true);
        }
        assertEquals(base, setpoint, 1e-9,
            "100 steps down must stop at the enable-time datum, never below it");
    }

    /**
     * The floor is the ENABLE-TIME DATUM, whether that sits above or below the fixed MIN_ANGLE
     * soft limit -- a hood enabled raised cannot be driven back down past where it started,
     * and a hood enabled resting below MIN_ANGLE is still not dragged up to it. Without a
     * datum there is nothing measured to floor against, so MIN_ANGLE stands in. The one
     * exception is a hood enabled above MAX_ANGLE, which must still be able to come down to
     * it -- covered by enablingAboveTheCeilingIsHoldThenDownOnly.
     */
    @Test
    void theFloorIsTheEnableTimeDatum() {
        double resting = ShooterSubsystemConstants.MIN_ANGLE - 1.8;   // the ~3.2 deg rest
        assertEquals(resting, ShooterSubsystem.hoodFloorAngle(resting, true), 1e-9);
        assertEquals(resting, ShooterSubsystem.clampDesiredAngle(-999, resting, true), 1e-9);
        double raised = 30.0;
        assertEquals(raised, ShooterSubsystem.hoodFloorAngle(raised, true), 1e-9);
        assertEquals(raised, ShooterSubsystem.clampDesiredAngle(5.0, raised, true), 1e-9,
            "a commanded shot angle below the enable position is floored too");
        assertEquals(ShooterSubsystemConstants.MIN_ANGLE,
            ShooterSubsystem.hoodFloorAngle(raised, false), 1e-9,
            "no datum yet -> the fixed soft limit stands in");
    }

    /** One click must move the setpoint by exactly the step, mid-range. */
    @Test
    void oneClickMovesExactlyOneStep() {
        double base = 20.0;
        assertEquals(20.0 + STEP,
            ShooterSubsystem.clampDesiredAngle(
                20.0 + STEP, base, true), 1e-9,
            "one step moves exactly one step, whatever the step is");
    }

    /** A datum far outside the soft band must still produce a usable, non-inverted range. */
    @Test
    void aDatumOutsideTheBandStillYieldsARealSetpoint() {
        double clamped = ShooterSubsystem.clampDesiredAngle(20, 80, true);
        assertTrue(Double.isFinite(clamped), "must produce a real setpoint, not NaN");
        // The band now widens to include the datum instead of collapsing, so the limits can
        // never cross. The setpoint stays between the datum and the band it is heading toward.
        assertTrue(clamped >= ShooterSubsystemConstants.MIN_ANGLE && clamped <= 80,
            "must be a reachable setpoint between the soft floor and the datum: " + clamped);
    }

    // ---- the soft band must never DEMAND motion (team request 2026-08-26) ----
    // The hood RESTS at ~3.2 deg, below the 5.0 deg MIN_ANGLE floor. The band used to clamp the
    // enable-time seed up to the floor, so every enable drove the hood up at ~7.6 V. These pin
    // that it holds instead.

    /** THE REGRESSION GUARD: enabling with the hood parked below the soft floor must not move it. */
    @Test
    void enablingBelowTheSoftFloorDoesNotCommandMotion() {
        double rest = ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN; // ~3.224, below MIN_ANGLE
        assertTrue(rest < ShooterSubsystemConstants.MIN_ANGLE,
            "precondition: the hood's rest position is below the soft floor");
        assertEquals(rest, ShooterSubsystem.clampDesiredAngle(rest, rest, true), 1e-9,
            "the seeded setpoint must equal the enable position exactly, or the hood lurches on enable");
    }

    /** DPAD-down at the floor holds; it must never jump the hood UP to satisfy MIN_ANGLE. */
    @Test
    void nudgingDownBelowTheFloorHoldsInsteadOfRising() {
        double rest = ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN;
        double commanded = ShooterSubsystem.clampDesiredAngle(
            rest - STEP, rest, true);
        assertEquals(rest, commanded, 1e-9, "must hold at the enable position, not rise to MIN_ANGLE");
    }

    /** Enabled below the floor, the hood may still be commanded UP toward the band. */
    @Test
    void nudgingUpFromBelowTheFloorStillWorks() {
        double rest = ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN;
        assertEquals(rest + STEP,
            ShooterSubsystem.clampDesiredAngle(rest + STEP, rest, true),
            1e-9, "the band opens toward the hood, it does not trap it");
    }

    /** Enabled ABOVE the ceiling: hold, and allow only downward commands (belt protection). */
    @Test
    void enablingAboveTheCeilingIsHoldThenDownOnly() {
        double high = ShooterSubsystemConstants.MAX_ANGLE + 2.0;
        assertEquals(high, ShooterSubsystem.clampDesiredAngle(high, high, true), 1e-9,
            "must hold where it was enabled");
        assertEquals(high, ShooterSubsystem.clampDesiredAngle(high + 5, high, true), 1e-9,
            "must never be commanded further up");
        assertEquals(high - STEP,
            ShooterSubsystem.clampDesiredAngle(high - STEP, high, true),
            1e-9, "but must still come down");
    }

    /** The widened band must never invert, whatever the datum. */
    @Test
    void theBandAlwaysContainsTheDatum() {
        for (double base = -10; base <= 90; base += 0.5) {
            assertEquals(base, ShooterSubsystem.clampDesiredAngle(base, base, true), 1e-9,
                "clamping the datum to itself must be a no-op at base=" + base);
        }
    }
}
