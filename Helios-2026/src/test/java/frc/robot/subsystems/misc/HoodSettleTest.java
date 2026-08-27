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
        // Approaching the new target from 2 deg away.
        holding = ShooterSubsystem.hoodShouldHold(2.0, holding);
        assertTrue(!holding, "2 deg away: drive");
        // It arrives, overshooting slightly.
        holding = ShooterSubsystem.hoodShouldHold(-0.3, holding);
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
     * A HELD jog must escape the hysteresis band promptly. The setpoint ramps at
     * HOOD_JOG_DEG_PER_SEC from zero error, and the loop stays held (0 V) until the error
     * passes HOOD_REENGAGE_DEG -- so there is an unavoidable dead time of REENGAGE / rate at
     * the start of every press. Too long and holding the DPAD reads as a broken control.
     */
    @Test
    void aHeldJogEscapesTheReengageBandPromptly() {
        double secondsToEscape = REENGAGE / ShooterSubsystemConstants.HOOD_JOG_DEG_PER_SEC;
        assertTrue(secondsToEscape <= 0.5,
            "a held DPAD must start the hood inside 0.5 s; takes " + secondsToEscape + " s at "
                + ShooterSubsystemConstants.HOOD_JOG_DEG_PER_SEC + " deg/s");
    }

    @Test
    void settleBandIsInsideTheReengageBand() {
        assertTrue(TOL < REENGAGE, "hysteresis requires the re-engage threshold to be the wider one");
        assertEquals(0.5, TOL, 1e-9, "guards against a tolerance change silently widening the deadband");
    }
}
