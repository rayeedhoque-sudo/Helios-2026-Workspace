package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.constants.Constants.Vision;

/**
 * Pins the MegaTag2 measurement-weighting math. A flipped ratio here does not crash anything --
 * it silently makes the robot trust a far, single-tag solve as much as a close two-tag one,
 * which is exactly the class of error that puts the drawn robot in the wrong place.
 */
public class VisionFusionTest {

    private static final double EPS = 1e-9;

    @Test
    void midRangeSingleTagMatchesTheOldFixedStdDev() {
        // BASE was chosen so one tag at 3 m reproduces the previous flat 0.7 m weighting, i.e.
        // this change does not quietly re-trust vision at the typical shooting range. BASE is a
        // round 0.08 rather than the exact 0.0778, so allow the resulting 3% (0.72 vs 0.70).
        assertEquals(0.7, CommandSwerveDrivetrain.megaTag2XyStdDev(1, 3.0), 0.03);
    }

    @Test
    void trustFallsOffWithTheSquareOfDistance() {
        double near = CommandSwerveDrivetrain.megaTag2XyStdDev(1, 2.0);
        double far = CommandSwerveDrivetrain.megaTag2XyStdDev(1, 4.0);
        // Doubling the distance must quadruple the std dev (trust it 4x less), not double it.
        assertEquals(4.0, far / near, EPS);
    }

    @Test
    void moreTagsAreTrustedMore() {
        double one = CommandSwerveDrivetrain.megaTag2XyStdDev(1, 4.0);
        double two = CommandSwerveDrivetrain.megaTag2XyStdDev(2, 4.0);
        assertEquals(one / 2.0, two, EPS);
        assertTrue(two < one);
    }

    @Test
    void noTagsIsInfinitelyUntrusted() {
        // A zero-tag solve must never move the estimate, whatever distance is reported.
        assertTrue(Double.isInfinite(CommandSwerveDrivetrain.megaTag2XyStdDev(0, 3.0)));
        assertTrue(Double.isInfinite(CommandSwerveDrivetrain.megaTag2XyStdDev(-1, 3.0)));
    }

    @Test
    void distanceFloorAndNaNCanNeverProduceAZeroOrNaNStdDev() {
        // A point-blank (or bogus 0 m) reading must not divide the std dev to ~0, which would
        // make the filter adopt that single solve wholesale.
        double floored = Vision.VISION_XY_STDDEV_BASE
            * Vision.VISION_MIN_TAG_DIST_METERS * Vision.VISION_MIN_TAG_DIST_METERS;
        assertEquals(floored, CommandSwerveDrivetrain.megaTag2XyStdDev(1, 0.0), EPS);
        assertEquals(floored, CommandSwerveDrivetrain.megaTag2XyStdDev(1, -5.0), EPS);
        // A NaN avgTagDist (missing field in a short botpose array) must not reach the filter.
        double nan = CommandSwerveDrivetrain.megaTag2XyStdDev(1, Double.NaN);
        assertTrue(!Double.isNaN(nan) && nan > 0);
    }
}
