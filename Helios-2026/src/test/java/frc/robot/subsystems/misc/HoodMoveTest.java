package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * The two DPAD hood positions (2026-08-27): RIGHT raises to BASE + HOOD_UP_STEP_UNITS and no
 * further, LEFT returns to the base, RIGHT works again after that. What makes "no further" true
 * is that the target is an ABSOLUTE angle, so a second press is already satisfied -- pinned here.
 *
 * FLOOR is a RAW ENCODER reading now (the hood angle is the raw encoder angle), so the test uses
 * the live resting value rather than a physical degree.
 */
public class HoodMoveTest {

    /** The live resting raw reading, captured on the robot 2026-08-27. */
    private static final double FLOOR = 311.169;
    private static final double TARGET = FLOOR + ShooterSubsystemConstants.HOOD_UP_STEP_UNITS;

    /** A rising move is done at or above the target; a falling one at or below it. */
    @Test
    void reachedIsDirectionAware() {
        assertFalse(ShooterSubsystem.hoodMoveReached(TARGET - 10, TARGET, true), "still climbing");
        assertTrue(ShooterSubsystem.hoodMoveReached(TARGET, TARGET, true), "exactly there is there");
        assertTrue(ShooterSubsystem.hoodMoveReached(TARGET + 1, TARGET, true), "overshot is there");

        assertFalse(ShooterSubsystem.hoodMoveReached(FLOOR + 10, FLOOR, false), "still descending");
        assertTrue(ShooterSubsystem.hoodMoveReached(FLOOR, FLOOR, false), "exactly there is there");
        assertTrue(ShooterSubsystem.hoodMoveReached(FLOOR - 1, FLOOR, false), "undershot is there");
    }

    /** Press RIGHT again at the preset: already reached, so the move ends without driving. */
    @Test
    void aSecondUpPressIsAlreadySatisfied() {
        assertTrue(ShooterSubsystem.hoodMoveReached(TARGET, TARGET, true),
            "the hood may never be raised past the preset by repeated presses");
        // ...and the same for the overshoot the open-loop stop inevitably leaves behind.
        assertTrue(ShooterSubsystem.hoodMoveReached(TARGET + 1.2, TARGET, true));
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
