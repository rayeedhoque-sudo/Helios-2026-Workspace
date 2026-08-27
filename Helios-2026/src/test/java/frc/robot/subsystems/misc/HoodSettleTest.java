package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins the hood settle band (ShooterSubsystem.hoodShouldHold). Team requirement
 * 2026-08-22: a DPAD click moves the hood 2 deg and it STAYS there -- no hunting.
 *
 * Without the band the hood limit-cycles: it breaks free at ~7.5 V, carries past the
 * target, error flips negative, and the loop drives it back DOWN with up to 6 V plus
 * gravity, past the target again. The band stops driving once the hood arrives; the
 * hysteresis stops the drive chattering on and off at the band edge.
 */
public class HoodSettleTest {

    private static final double TOL = ShooterSubsystemConstants.ANGLE_TOLERANCE;
    private static final double REENGAGE = ShooterSubsystemConstants.HOOD_REENGAGE_DEG;

    @Test
    void holdsOnceInsideTheTolerance() {
        assertTrue(ShooterSubsystem.hoodShouldHold(0, false), "at the target = hold");
        assertTrue(ShooterSubsystem.hoodShouldHold(TOL, false), "at the tolerance edge = hold");
        assertTrue(ShooterSubsystem.hoodShouldHold(-TOL, false), "overshoot side too");
    }

    @Test
    void drivesWhenFarFromTheTarget() {
        assertTrue(!ShooterSubsystem.hoodShouldHold(REENGAGE, false), "a fresh big error must drive");
        assertTrue(!ShooterSubsystem.hoodShouldHold(10, true), "even when holding, a big error re-engages");
    }

    /**
     * The hysteresis: between the settle and re-engage thresholds the answer depends on
     * which side you came from. One threshold here would chatter the drive on and off.
     */
    @Test
    void staysHeldThroughTheHysteresisBand() {
        double midBand = (TOL + REENGAGE) / 2;
        assertTrue(ShooterSubsystem.hoodShouldHold(midBand, true),
            "already parked: small drift must NOT restart the drive");
        assertTrue(!ShooterSubsystem.hoodShouldHold(midBand, false),
            "approaching: the same error must still drive toward the target");
    }

    /**
     * The actual requirement, simulated: a 2 deg click, an overshoot, and the settle must
     * be permanent rather than bouncing. Feeding the previous answer back in is what the
     * periodic loop does with the latched flag.
     */
    @Test
    void aClickSettlesAndStaysSettled() {
        boolean holding = false;
        // Approaching the new target from well outside the band. Scaled off TOL rather than a
        // literal: the bands are in RAW ENCODER UNITS now (2026-08-27), not degrees.
        holding = ShooterSubsystem.hoodShouldHold(4 * TOL, holding);
        assertTrue(!holding, "well outside the band: drive");
        // It arrives, overshooting slightly.
        holding = ShooterSubsystem.hoodShouldHold(-0.6 * TOL, holding);
        assertTrue(holding, "arrived (with overshoot): stop driving");
        // Now it must STAY held through ordinary jitter and small sag -- this is the anti-hunt.
        // Scaled off REENGAGE so retuning the band on the robot doesn't fail this test:
        // every sample is inside it, which is what "parked" means.
        double[] jitter = { -0.3, 0.15, -0.05, 0.5, -0.7, 0.85, -0.95 };
        for (int i = 0; i < jitter.length; i++) { jitter[i] *= REENGAGE; }
        for (double error : jitter) {
            holding = ShooterSubsystem.hoodShouldHold(error, holding);
            assertTrue(holding, "must stay parked at error " + error + " -- this is the hunting case");
        }
        // Only a real departure past the re-engage threshold restarts the drive.
        holding = ShooterSubsystem.hoodShouldHold(REENGAGE, holding);
        assertTrue(!holding, "a genuine departure re-engages the loop");
    }

    /**
     * The open-loop DPAD jog (2026-08-26) must stay inside the drive caps the closed loop is
     * held to -- a jog voltage is still a voltage into a 20 A NEO 550, and UP must clear the
     * ~7 V breakaway or holding the DPAD does nothing at all.
     */
    @Test
    void jogVoltsStayInsideTheDriveCaps() {
        assertTrue(ShooterSubsystemConstants.HOOD_JOG_UP_VOLTS > 0
                && ShooterSubsystemConstants.HOOD_JOG_UP_VOLTS
                    <= ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE,
            "up jog must be positive and within HOOD_MAX_UP_VOLTAGE");
        assertTrue(ShooterSubsystemConstants.HOOD_JOG_DOWN_VOLTS > 0
                && ShooterSubsystemConstants.HOOD_JOG_DOWN_VOLTS
                    <= ShooterSubsystemConstants.HOOD_MAX_DOWN_VOLTAGE,
            "down jog must be positive (the sign is applied at the binding) and within cap");
    }

    @Test
    void settleBandIsInsideTheReengageBand() {
        assertTrue(TOL < REENGAGE, "hysteresis requires the re-engage threshold to be the wider one");
        assertEquals(0.5 * ShooterSubsystemConstants.HOOD_UNITS_PER_DEG, TOL, 1e-9,
            "guards against a tolerance change silently widening the deadband -- the band is the"
                + " tuned half-degree, expressed in raw encoder units");
    }
}
