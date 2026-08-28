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
     * THE ONLY HOOD DRIVE IS THE POSITION LOOP (2026-08-28): the open-loop jog volts are gone,
     * so the gains themselves have to be able to break the hood away. At a one-step error the
     * P term plus the gravity feedforward must clear the ~7.5 V breakaway, or a DPAD press
     * writes a setpoint and the hood never moves -- and the total must still fit the up cap.
     */
    @Test
    void aOneStepPressCanBreakTheHoodAway() {
        double step = ShooterSubsystemConstants.HOOD_UP_STEP_UNITS;
        double drive = ShooterSubsystemConstants.SHOOTER_ANGLE_kP * step
            + ShooterSubsystem.hoodLiftFeedforward(step);
        assertTrue(drive >= 7.4, "one step must ask for at least the ~7.5 V breakaway, got " + drive);
        assertTrue(drive <= ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE + 1e-9,
            "and must not exceed the up cap before clamping, got " + drive);
        assertEquals(0.0, ShooterSubsystemConstants.SHOOTER_ANGLE_kI, 1e-12,
            "kI must stay 0 -- an integrator against stiction winds up and then lurches");
        assertTrue(ShooterSubsystemConstants.SHOOTER_ANGLE_kD > 0,
            "kD is the anti-overshoot term; a zero kD is the oscillating tune");
    }

    @Test
    void settleBandIsInsideTheReengageBand() {
        assertTrue(TOL < REENGAGE, "hysteresis requires the re-engage threshold to be the wider one");
        assertEquals(0.5 * ShooterSubsystemConstants.HOOD_UNITS_PER_DEG, TOL, 1e-9,
            "guards against a tolerance change silently widening the deadband -- the band is the"
                + " tuned half-degree, expressed in raw encoder units");
    }
}
