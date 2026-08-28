package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * THE HOOD STAYS WHERE A DPAD PRESS PUT IT (team requirement 2026-08-27): it goes up, it locks
 * at the new angle, and nothing pulls it back down.
 *
 * The bug this pins: the jog branch in periodic() pins the setpoint to the measured angle every
 * loop, but reads it BEFORE that loop's voltage reaches the motor. The final powered loop moves
 * the hood 25-55 raw units, the jog ends, and the position loop wakes up against a setpoint one
 * whole loop of travel below the hood -- a large NEGATIVE error, which is exactly a command to
 * drive back down. These tests exercise the real decision functions on both sides of the fix.
 */
public class HoodHoldTest {

    /** One loop of powered travel, mid-range of the 25-55 raw units measured on the robot. */
    private static final double POWERED_LOOP_TRAVEL = 40.0;

    @Test
    void theEdgeFiresOnlyWhenTheJogEnds() {
        assertTrue(ShooterSubsystem.hoodJogJustEnded(true, false), "jogging -> not jogging");
        assertFalse(ShooterSubsystem.hoodJogJustEnded(true, true), "still jogging is not an end");
        assertFalse(ShooterSubsystem.hoodJogJustEnded(false, true), "starting is not an end");
        assertFalse(ShooterSubsystem.hoodJogJustEnded(false, false), "idle is not an end");
    }

    /**
     * WITHOUT the re-seed the loop drives the hood DOWN. Pinned so the failure mode stays
     * visible: this is what the stale setpoint asks the hardware to do.
     */
    @Test
    void aStaleSetpointWouldDriveTheHoodBackDown() {
        double staleSetpoint = 311.169;                             // pinned pre-move
        double hood = staleSetpoint + POWERED_LOOP_TRAVEL;          // where it actually landed
        double error = staleSetpoint - hood;                        // ~ -40

        assertFalse(ShooterSubsystem.hoodShouldHold(error, false),
            "a stale setpoint leaves an error way past the band, so the loop drives");
        assertTrue(ShooterSubsystem.hoodLiftFeedforward(error) < 0,
            "and the feedforward it fires is a LOWERING one -- the hood sinks after every press");
    }

    /**
     * WITH the re-seed the setpoint is the angle the hood actually reached, so the loop holds
     * on the brake instead of correcting. This is the requirement.
     */
    @Test
    void reSeedingOnTheJogEndLocksTheHoodAtItsNewAngle() {
        double base = 311.169;
        double hood = base + POWERED_LOOP_TRAVEL;                   // after the final powered loop
        double setpoint = hood;                                     // what periodic() now seeds

        assertTrue(ShooterSubsystem.hoodShouldHold(setpoint - hood, false),
            "zero error must latch the settle band -- 0 V, brake holds");
        assertEquals(0, ShooterSubsystem.hoodLiftFeedforward(setpoint - hood), 1e-9,
            "and no feedforward may be applied at the target, in either direction");
    }

    /**
     * The lock has to survive the guards too: the new angle must be a legal setpoint, above the
     * enable-time floor and below the ceiling, or the clamp would drag it somewhere else.
     */
    @Test
    void theNewAngleSurvivesTheTravelGuards() {
        double base = 311.169;
        double hood = base + POWERED_LOOP_TRAVEL;
        assertEquals(hood, ShooterSubsystem.clampDesiredAngle(hood, base, true), 1e-9,
            "the angle the hood reached must be commandable, or the clamp moves it");
        assertTrue(hood < ShooterSubsystem.hoodCeilingAngle(base, true),
            "and it must sit below the ceiling guard, which would otherwise cut the drive");
    }

    /**
     * A press that lands INSIDE the settle band of the previous target must still hold there
     * rather than snapping back -- the small-move case, where the overshoot is only a few units.
     */
    @Test
    void aSmallOvershootAlsoHoldsRatherThanCorrecting() {
        double drift = 0.9 * ShooterSubsystemConstants.ANGLE_TOLERANCE;
        assertTrue(ShooterSubsystem.hoodShouldHold(-drift, false),
            "a landing inside the tolerance holds immediately");
        assertTrue(ShooterSubsystem.hoodShouldHold(-drift, true),
            "and stays held once latched");
    }
}
