package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * The two DPAD hood positions (2026-08-27): RIGHT raises to HOOD_UP_PRESET_DEG and no further,
 * LEFT returns to the floor, RIGHT works again after that. What makes "no further" true is that
 * the target is an ABSOLUTE angle, so a second press is already satisfied -- pinned here.
 */
public class HoodMoveTest {

    private static final double FLOOR = ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN;
    private static final double PRESET = ShooterSubsystemConstants.HOOD_UP_PRESET_DEG;

    /** A rising move is done at or above the target; a falling one at or below it. */
    @Test
    void reachedIsDirectionAware() {
        assertFalse(ShooterSubsystem.hoodMoveReached(10.0, PRESET, true), "still climbing");
        assertTrue(ShooterSubsystem.hoodMoveReached(PRESET, PRESET, true), "exactly there is there");
        assertTrue(ShooterSubsystem.hoodMoveReached(PRESET + 1, PRESET, true), "overshot is there");

        assertFalse(ShooterSubsystem.hoodMoveReached(10.0, FLOOR, false), "still descending");
        assertTrue(ShooterSubsystem.hoodMoveReached(FLOOR, FLOOR, false), "exactly there is there");
        assertTrue(ShooterSubsystem.hoodMoveReached(FLOOR - 1, FLOOR, false), "undershot is there");
    }

    /** Press RIGHT again at the preset: already reached, so the move ends without driving. */
    @Test
    void aSecondUpPressIsAlreadySatisfied() {
        assertTrue(ShooterSubsystem.hoodMoveReached(PRESET, PRESET, true),
            "the hood may never be raised past the preset by repeated presses");
        // ...and the same for the overshoot the open-loop stop inevitably leaves behind.
        assertTrue(ShooterSubsystem.hoodMoveReached(PRESET + 1.2, PRESET, true));
    }

    /** The preset must be reachable: inside the soft band and not capped by the travel window. */
    @Test
    void thePresetSurvivesTheClamp() {
        assertEquals(PRESET, ShooterSubsystem.clampDesiredAngle(PRESET, FLOOR, true), 1e-9,
            "DPAD-RIGHT must actually be able to command the preset");
        assertTrue(PRESET < ShooterSubsystemConstants.MAX_ANGLE,
            "the preset must stay clear of the belt-skip inset");
    }

    /** DPAD-LEFT targets the floor, which is exactly where clamping bottoms out. */
    @Test
    void theDownTargetIsTheFloor() {
        assertEquals(FLOOR, ShooterSubsystem.clampDesiredAngle(FLOOR, FLOOR, true), 1e-9);
        assertEquals(FLOOR, ShooterSubsystem.hoodFloorAngle(FLOOR, true), 1e-9);
    }
}
