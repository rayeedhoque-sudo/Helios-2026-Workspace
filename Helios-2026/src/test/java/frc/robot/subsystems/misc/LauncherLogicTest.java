package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.LauncherConstants;

/** Pins the V2 launcher's pure logic (LauncherSubsystem static helpers). */
class LauncherLogicTest {

    /** RT rpm: DPAD steps can't push the target below 0 or above the 4000 ceiling. */
    @Test
    void flywheelRpmIsClamped() {
        assertEquals(0, LauncherSubsystem.clampFlywheelRpm(-100), 1e-9);
        assertEquals(4000, LauncherSubsystem.clampFlywheelRpm(4100), 1e-9);
        assertEquals(2000, LauncherSubsystem.clampFlywheelRpm(LauncherConstants.FLYWHEEL_START_RPM), 1e-9);
    }

    /** Hood: 0 deg (homed bottom stop) to the 30 deg travel limit, nothing outside. */
    @Test
    void hoodTargetIsClamped() {
        assertEquals(0, LauncherSubsystem.clampHoodDeg(-5), 1e-9);
        assertEquals(30, LauncherSubsystem.clampHoodDeg(35), 1e-9);
        assertEquals(15, LauncherSubsystem.clampHoodDeg(15), 1e-9);
    }

    /** 18T -> 26T: the motor turns 26/18 times per flywheel turn, and the conversion round-trips. */
    @Test
    void flywheelRatioConversion() {
        // 1800 flywheel rpm = 30 rps flywheel = 30 * 26/18 = 43.33 motor rps
        assertEquals(30.0 * 26.0 / 18.0, LauncherSubsystem.flywheelRpmToMotorRps(1800), 1e-9);
        assertEquals(1800, LauncherSubsystem.motorRpsToFlywheelRpm(
            LauncherSubsystem.flywheelRpmToMotorRps(1800)), 1e-9);
        // The 4000 ceiling is right at X60 free speed (~96.7 rps) -- keep it that way.
        assertTrue(LauncherSubsystem.flywheelRpmToMotorRps(LauncherConstants.FLYWHEEL_MAX_RPM) <= 97);
    }

    /** Belt gate: a zero target is never "at speed", and the band is inclusive. */
    @Test
    void atSpeedGate() {
        assertFalse(LauncherSubsystem.atSpeed(0, 0, 100), "stopped wheels must never open the belts");
        assertTrue(LauncherSubsystem.atSpeed(1900, 2000, 100));
        assertFalse(LauncherSubsystem.atSpeed(1899, 2000, 100));
        assertTrue(LauncherSubsystem.atSpeed(2100, 2000, 100));
    }

    /**
     * Homing: startup inrush during the grace window must NOT count as hitting the bottom stop
     * (a false home puts 0 deg mid-travel and the 30 deg limit past the real top stop).
     */
    @Test
    void homingIgnoresInrushDuringGrace() {
        double amps = LauncherConstants.HOOD_STATOR_A; // pegged at the limit
        assertFalse(LauncherSubsystem.homingSawStop(0.0, amps));
        assertFalse(LauncherSubsystem.homingSawStop(LauncherConstants.HOOD_HOMING_GRACE_SEC - 0.01, amps));
        assertTrue(LauncherSubsystem.homingSawStop(LauncherConstants.HOOD_HOMING_GRACE_SEC, amps));
        assertFalse(LauncherSubsystem.homingSawStop(1.0, LauncherConstants.HOOD_HOMING_STOP_AMPS - 1),
            "free travel current must not read as the stop");
        // The stop threshold must be reachable under the hood's own stator limit.
        assertTrue(LauncherConstants.HOOD_HOMING_STOP_AMPS < LauncherConstants.HOOD_STATOR_A);
        assertTrue(LauncherConstants.HOOD_STALL_AMPS < LauncherConstants.HOOD_STATOR_A);
    }
}
