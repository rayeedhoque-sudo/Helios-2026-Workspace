package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins the hood absolute-encoder map (ShooterSubsystem.unwrapHoodRaw /
 * hoodDegreesFromRaw) against the labelled hand sweep measured on the robot 2026-08-22.
 *
 * Why this exists: the hood's travel CROSSES the encoder's 0/360 rollover, and the old
 * map had no wrap handling. Mid-travel it produced -86 deg .. +105 deg, the PID chased
 * that, and the hood was driven into its bottom hard stop while the MIN_ANGLE travel
 * guard stayed blind (the bad mapping read HIGH, not low). This test fails at the desk if
 * the wrap handling is ever dropped or the anchors are edited without re-measuring.
 *
 * Sweep data (2026-08-24, robot disabled, two 50 Hz NT-logged hand sweeps):
 *   full DOWN stop = raw 360.0 unwrapped (raw ~359.6 / ~0.3) -> 3.224 deg
 *   full UP   stop = raw 290.4 (+360 = 650.4)                -> 44.5  deg
 *   never occupies raw 290.4 .. 359.6 (the ~69-unit dead arc the split sits in)
 */
public class HoodAngleMapTest {

    private static final double TOL_DEG = 0.05;
    // How far outside the stop ANGLES a reading may legally map. The down stop straddles the
    // encoder's 0/360 rollover and its rest position varied ~0.7 raw units (0.1 deg) between
    // the two measured sweeps, so the anchor is the midpoint and real readings sit slightly
    // either side of it. This is the measured slop, not a fudge factor.
    private static final double STOP_BAND_DEG = 0.15;

    @Test
    void anchorsMapToTheirPhysicalStops() {
        // The down stop sits ON the rollover: raw 0.0 unwraps to 360.0, the anchor.
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN,
            ShooterSubsystem.hoodDegreesFromRaw(0.0), TOL_DEG,
            "full-DOWN raw must map to the down stop angle");
        // Full up is measured PAST the wrap: raw 290.4 unwraps to 650.4.
        assertEquals(ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP,
            ShooterSubsystem.hoodDegreesFromRaw(290.4), TOL_DEG,
            "full-UP raw (past the rollover) must map to the up stop angle");
    }

    @Test
    void wrapSplitLiftsOnlyThePostRolloverArc() {
        // Below the split = past the rollover -> +360.
        assertEquals(650.4, ShooterSubsystem.unwrapHoodRaw(290.4), 1e-9);
        assertEquals(360.0, ShooterSubsystem.unwrapHoodRaw(0.0), 1e-9);
        // At or above the split = already continuous -> unchanged.
        assertEquals(359.6, ShooterSubsystem.unwrapHoodRaw(359.6), 1e-9);
        assertEquals(340.0, ShooterSubsystem.unwrapHoodRaw(340.0), 1e-9);
        // The reason the split is 325 and not 0: the DOWN stop sits on the rollover, so
        // resting jitter crosses raw 0 constantly. Either side must stay continuous --
        // a 0.2-unit step of raw must not become a 41 deg step of angle.
        assertEquals(0.2, ShooterSubsystem.unwrapHoodRaw(0.1)
            - ShooterSubsystem.unwrapHoodRaw(359.9), 1e-9);
    }

    /**
     * The whole point of the unwrap: sweeping the hood up must produce a MONOTONICALLY
     * increasing angle straight through the rollover. The old map jumped ~191 deg here.
     */
    @Test
    void angleIsMonotonicAcrossTheRollover() {
        // Ascending unwrapped positions spanning the wrap, as the hood actually travels.
        double[] rawUpSweep = { 359.6, 359.9, 0.1, 20, 60, 120, 200, 260, 290.4 };
        double previous = Double.NEGATIVE_INFINITY;
        for (double raw : rawUpSweep) {
            double deg = ShooterSubsystem.hoodDegreesFromRaw(raw);
            assertTrue(deg > previous,
                "angle must rise monotonically across the wrap; raw " + raw + " gave " + deg);
            previous = deg;
        }
        // And it stays inside the physical stops the whole way.
        for (double raw : rawUpSweep) {
            double deg = ShooterSubsystem.hoodDegreesFromRaw(raw);
            assertTrue(deg >= ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN - STOP_BAND_DEG
                    && deg <= ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP + STOP_BAND_DEG,
                "raw " + raw + " mapped outside the physical stops: " + deg);
        }
    }

    /**
     * The soft travel limits must sit INSIDE the measured hard stops, or the travel guard
     * in periodic() can never fire before the mechanism does.
     */
    @Test
    void softLimitsSitInsideTheHardStops() {
        assertTrue(ShooterSubsystemConstants.MIN_ANGLE > ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN,
            "MIN_ANGLE must be above the down stop");
        assertTrue(ShooterSubsystemConstants.MAX_ANGLE < ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP,
            "MAX_ANGLE must be below the up stop");
    }

    /**
     * The sanity band is expressed in UNWRAPPED units and must contain the full measured
     * travel -- the old raw band rejected most of it, which is what held the hood at 0 V.
     */
    @Test
    void sanityBandAcceptsTheWholeMeasuredTravel() {
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(359.6), "full-down anchor must be valid");
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(290.4), "full-up anchor must be valid");
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(359.9), "just below the rollover must be valid");
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(0.5), "just past the rollover must be valid");
    }

    /**
     * The one case the unwrap would otherwise hide: a dead encoder publishes bit-exact 0.0,
     * which unwraps to 360 and maps to a plausible ~35.9 deg. If this ever passes as valid,
     * the hood closes the loop on a fictional angle.
     */
    @Test
    void deadEncoderZeroIsRejected() {
        assertEquals(3.224, ShooterSubsystem.hoodDegreesFromRaw(0.0), 0.05,
            "raw 0 maps into legal travel -- which is exactly why it needs its own check");
        assertTrue(!ShooterSubsystem.isHoodFeedbackValid(0.0),
            "bit-exact 0.0 is a dead encoder, not a position");
    }

    /**
     * Readings outside the old measured travel are now VALID. The absolute band was removed
     * 2026-08-22: the encoder drifts against the hood, so a band eventually rejects wherever
     * it has wandered to and forces 0 V -- which is what left the hood dead on the DPAD at a
     * reported -32.96 deg. With the angle tracked relatively, only CHANGES matter, so any dial
     * position is legal and bad data is caught by the delta filters instead (HoodTrackingTest).
     */
    @Test
    void anyDialPositionIsAcceptedNow() {
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(325.1),
            "a reading in the unused arc must be usable -- rejecting it killed hood control");
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(300.0));
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(340.0));
    }
}
