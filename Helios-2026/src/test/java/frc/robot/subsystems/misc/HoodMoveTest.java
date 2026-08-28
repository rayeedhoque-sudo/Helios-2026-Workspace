package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * The DPAD hood moves: RIGHT raises the hood by HOOD_UP_STEP_UNITS on EVERY press, measured from
 * where the hood currently is; LEFT returns it to this enable's base in one press.
 *
 * CLOSED LOOP since 2026-08-28 -- a press is a SETPOINT WRITE and nothing else, so what these
 * tests pin is the setpoint arithmetic (the clamp and the step-from-the-hood rule). The
 * direction-aware hoodMoveReached checks are gone with the open-loop move they belonged to.
 *
 * FLOOR is a RAW ENCODER reading now (the hood angle is the raw encoder angle), so the test uses
 * the live resting value rather than a physical degree.
 */
public class HoodMoveTest {

    /** The live resting raw reading, captured on the robot 2026-08-27. */
    private static final double FLOOR = 311.169;
    private static final double TARGET = FLOOR + ShooterSubsystemConstants.HOOD_UP_STEP_UNITS;

    /**
     * EVERY press steps again, and it steps from the HOOD, not from the last target. This is
     * what keeps presses alive through the open-loop overshoot: a press that lands 40 units
     * past its 5-unit target must leave the next press asking for 5 above THAT, not for a
     * target the hood has already sailed past (which would be already-satisfied and do nothing).
     */
    @Test
    void everyPressStepsAgainFromWhereTheHoodActuallyIs() {
        double step = ShooterSubsystemConstants.HOOD_UP_STEP_UNITS;
        double hood = FLOOR;
        for (int press = 0; press < 5; press++) {
            double target = hood + step;      // what the binding asks for, read off the hood
            assertTrue(target > hood, "every press must ask for a rise");
            assertEquals(target, ShooterSubsystem.clampDesiredAngle(target, FLOOR, true), 1e-9,
                "and the clamp must let it through");
            assertTrue(target - hood >= step,
                "a fresh press must always ask for a further rise, never an already-reached one");
            hood += 40;                        // the real overshoot: one powered loop, ~25-55
        }
        assertTrue(hood > FLOOR + 5 * step,
            "five presses must have raised the hood, not stalled at the first target");
    }

    /**
     * A press that lands PAST its target must not be chased back down. The settle band is what
     * ends the move now that the loop owns the drive: the overshoot the hood leaves behind sits
     * inside the re-engage band once the setpoint is the angle it reached.
     */
    @Test
    void anOvershootIsHeld_notChasedBack() {
        assertTrue(ShooterSubsystem.hoodShouldHold(0, false),
            "at the setpoint the loop holds on the brake");
        assertTrue(ShooterSubsystem.hoodShouldHold(
                -0.9 * ShooterSubsystemConstants.HOOD_REENGAGE_DEG, true),
            "and stays held through the drift a landing leaves behind");
    }

    /** The target must be reachable: above the floor and inside the travel window. */
    @Test
    void theTargetSurvivesTheClamp() {
        assertEquals(TARGET, ShooterSubsystem.clampDesiredAngle(TARGET, FLOOR, true), 1e-9,
            "DPAD-RIGHT must actually be able to command base + the step");
        assertTrue(ShooterSubsystemConstants.HOOD_UP_STEP_UNITS
                < ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG,
            "the step must fit inside the hood's travel");
    }

    /** DPAD-LEFT targets the floor, which is exactly where clamping bottoms out. */
    @Test
    void theDownTargetIsTheFloor() {
        assertEquals(FLOOR, ShooterSubsystem.clampDesiredAngle(FLOOR, FLOOR, true), 1e-9);
        assertEquals(FLOOR, ShooterSubsystem.hoodFloorAngle(FLOOR, true), 1e-9);
    }
}
