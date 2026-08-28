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
 * The failure this pins is the one that outlives the open-loop jog (deleted 2026-08-28): a
 * powered loop carries the hood 25-55 raw units past its target, so the loop is left with a
 * large NEGATIVE error -- which is exactly a command to drive back down. The settle band, not
 * the setpoint arithmetic, is what has to absorb that.
 */
public class HoodHoldTest {

    /** One loop of powered travel, mid-range of the 25-55 raw units measured on the robot. */
    private static final double POWERED_LOOP_TRAVEL = 40.0;

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
     * WITH the arrival latch the setpoint is re-seeded to the angle the hood actually reached,
     * so the loop holds on the brake instead of correcting. This is the requirement.
     */
    @Test
    void theArrivalLatchLocksTheHoodAtItsNewAngle() {
        double base = 311.169;
        double target = base + ShooterSubsystemConstants.HOOD_UP_STEP_UNITS;
        double hood = base + POWERED_LOOP_TRAVEL;                   // after the final powered loop

        assertTrue(ShooterSubsystem.hoodMoveReached(hood, target, true),
            "the hood is past what was asked for, so the latch must fire");
        double setpoint = hood;                                     // what the latch re-seeds

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
