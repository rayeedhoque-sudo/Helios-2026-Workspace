package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins ShooterSubsystem.guardedHoodJogVolts() -- the hard travel guard on the OPEN-LOOP
 * hood jog. The jog bypasses the position loop by design (the closed loop stick-slips on
 * the way up), so this guard is the only thing that stops a held DPAD from driving the
 * hood into a hard stop. It is safety-critical: if it ever stops zeroing at the limits,
 * a held button parks 7.5 V into a stalled NEO 550.
 */
public class HoodJogTest {

    private static final double UP = ShooterSubsystemConstants.HOOD_JOG_UP_VOLTS;
    private static final double DOWN = -ShooterSubsystemConstants.HOOD_JOG_DOWN_VOLTS;
    private static final double MIN = ShooterSubsystemConstants.MIN_ANGLE;
    private static final double MAX = ShooterSubsystemConstants.MAX_ANGLE;

    @Test
    void jogPassesThroughMidTravel() {
        double mid = (MIN + MAX) / 2;
        assertEquals(UP, ShooterSubsystem.guardedHoodJogVolts(UP, mid), 1e-9);
        assertEquals(DOWN, ShooterSubsystem.guardedHoodJogVolts(DOWN, mid), 1e-9);
    }

    @Test
    void upIsBlockedAtAndAboveMax() {
        assertEquals(0, ShooterSubsystem.guardedHoodJogVolts(UP, MAX), 1e-9, "at MAX, no up volts");
        assertEquals(0, ShooterSubsystem.guardedHoodJogVolts(UP, MAX + 5), 1e-9, "past MAX, no up volts");
    }

    @Test
    void downIsBlockedAtAndBelowMin() {
        assertEquals(0, ShooterSubsystem.guardedHoodJogVolts(DOWN, MIN), 1e-9, "at MIN, no down volts");
        assertEquals(0, ShooterSubsystem.guardedHoodJogVolts(DOWN, MIN - 5), 1e-9, "past MIN, no down volts");
    }

    /** Escaping a limit must still work -- the guard blocks only the direction that digs in. */
    @Test
    void theOppositeDirectionStillEscapesALimit() {
        assertEquals(DOWN, ShooterSubsystem.guardedHoodJogVolts(DOWN, MAX), 1e-9,
            "at MAX the hood must still be able to come down");
        assertEquals(UP, ShooterSubsystem.guardedHoodJogVolts(UP, MIN), 1e-9,
            "at MIN the hood must still be able to go up");
    }

    @Test
    void jogVoltsStayUnderTheTunedCaps() {
        assertTrue(ShooterSubsystemConstants.HOOD_JOG_UP_VOLTS
                <= ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE,
            "jog up volts must respect the up cap");
        assertTrue(ShooterSubsystemConstants.HOOD_JOG_DOWN_VOLTS
                <= ShooterSubsystemConstants.HOOD_MAX_DOWN_VOLTAGE,
            "jog down volts must respect the down cap");
    }
}
