package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins ShooterSubsystem.hoodLiftFeedforward(). The FF used to STEP from 0 V to the full
 * 5 V the instant error crossed ANGLE_TOLERANCE, which drove the hood in lurches on the
 * DPAD jog: PID alone is ~0.14 V at small error (far under breakaway), so nothing moved
 * until the step slammed on, the hood jumped, error collapsed, the step cut out, repeat.
 *
 * The fix is continuity, so that is what these pin -- along with the two endpoints, which
 * are deliberately UNCHANGED from the stepped version (0 V at the target, 5 V far below it).
 */
public class HoodFeedforwardTest {

    private static final double TOL = ShooterSubsystemConstants.ANGLE_TOLERANCE;
    private static final double FADE = ShooterSubsystemConstants.HOOD_FF_FADE_DEG;
    private static final double FULL = ShooterSubsystemConstants.HOOD_RAISE_FF_VOLTS;

    @Test
    void noLiftAtOrInsideTheTarget() {
        assertEquals(0, ShooterSubsystem.hoodLiftFeedforward(0), 1e-9, "at target = no lift");
        assertEquals(0, ShooterSubsystem.hoodLiftFeedforward(TOL), 1e-9, "at tolerance = no lift");
        assertEquals(0, ShooterSubsystem.hoodLiftFeedforward(-5), 1e-9,
            "lowering must stay FF-free -- gravity does that stroke");
    }

    @Test
    void fullLiftOncePastTheFadeBand() {
        assertEquals(FULL, ShooterSubsystem.hoodLiftFeedforward(TOL + FADE), 1e-9);
        assertEquals(FULL, ShooterSubsystem.hoodLiftFeedforward(30), 1e-9, "large error = full lift");
    }

    @Test
    void rampsLinearlyAcrossTheFadeBand() {
        assertEquals(FULL / 2, ShooterSubsystem.hoodLiftFeedforward(TOL + FADE / 2), 1e-9,
            "halfway through the band = half the lift");
    }

    /**
     * The actual bug: no jump anywhere. Sweeping error across the whole band, no single
     * step may exceed what the ramp itself produces. The old step jumped the full 5 V.
     */
    @Test
    void hasNoDiscontinuity() {
        double stepDeg = 0.01;
        double maxAllowedJump = FULL * stepDeg / FADE * 1.001; // the ramp's own slope, plus rounding
        double previous = ShooterSubsystem.hoodLiftFeedforward(-1);
        for (double err = -1; err <= TOL + FADE + 1; err += stepDeg) {
            double volts = ShooterSubsystem.hoodLiftFeedforward(err);
            assertTrue(Math.abs(volts - previous) <= maxAllowedJump,
                "FF jumped " + (volts - previous) + " V at error " + err + " -- the lurch is back");
            previous = volts;
        }
    }

    @Test
    void neverExceedsTheTunedCeiling() {
        for (double err = -10; err < 60; err += 0.25) {
            double volts = ShooterSubsystem.hoodLiftFeedforward(err);
            assertTrue(volts >= 0 && volts <= FULL, "FF out of range at error " + err + ": " + volts);
        }
    }
}
