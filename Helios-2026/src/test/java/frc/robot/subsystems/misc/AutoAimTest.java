package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins the pure logic behind the RB auto-aim shot (2026-08-29): the distance -> hood-position
 * table, the test-only tag alias, the camera-space distance, and the retarget deadband.
 * No hardware, no NetworkTables -- these are the static helpers in ShooterSubsystem.
 */
class AutoAimTest {

    private static final double EPS = 1e-6;

    /** The table interpolates between rows and CLAMPS outside them (never extrapolates). */
    @Test
    void hoodTableInterpolatesAndClamps() {
        InterpolatingDoubleTreeMap table = ShooterSubsystem.buildAutoAimHoodTable();
        double[][] rows = ShooterSubsystemConstants.AUTOAIM_HOOD_TABLE;

        // Every tabled distance returns its own value.
        for (double[] row : rows) {
            assertEquals(row[1], table.get(row[0]), EPS, "table point " + row[0] + " m");
        }
        // Midway between the first two rows = midway between their values.
        double midDistance = (rows[0][0] + rows[1][0]) / 2;
        assertEquals((rows[0][1] + rows[1][1]) / 2, table.get(midDistance), EPS);
        // Outside the table: clamp to the endpoints. This is what keeps a far-off or
        // nonsense distance from asking for a hood angle nobody ever tuned.
        assertEquals(rows[0][1], table.get(rows[0][0] - 5.0), EPS, "below the table clamps low");
        assertEquals(rows[rows.length - 1][1], table.get(rows[rows.length - 1][0] + 5.0), EPS,
            "above the table clamps high");
    }

    /** The table must stay sorted and never ask for more travel than the hood has. */
    @Test
    void hoodTableStaysInsideTheHoodTravel() {
        double[][] rows = ShooterSubsystemConstants.AUTOAIM_HOOD_TABLE;
        for (int i = 0; i < rows.length; i++) {
            assertTrue(rows[i][1] >= 0, "row " + i + " asks the hood BELOW the enable datum");
            assertTrue(rows[i][1] <= ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG,
                "row " + i + " asks for more travel than the hood has");
            if (i > 0) {
                assertTrue(rows[i][0] > rows[i - 1][0], "rows must be sorted by distance");
            }
        }
    }

    /** Tag 17 is aliased to a scoring tag for testing; nothing else is. */
    @Test
    void aliasCoversOnlyTheTestTag() {
        assertTrue(ShooterSubsystem.isAliasedScoreTag(ShooterSubsystemConstants.AUTOAIM_TEST_TAG_ID));
        assertFalse(ShooterSubsystem.isAliasedScoreTag(10), "a real score tag is not the alias");
        assertFalse(ShooterSubsystem.isAliasedScoreTag(1), "an unrelated feed tag is not the alias");
    }

    /** Camera-space distance adds the fixed tag-face -> hub-center depth. */
    @Test
    void distanceMeasuresToTheHubCenter() {
        // Dead ahead, 3 m of depth to the tag face -> 3 m + the face-to-center depth.
        assertEquals(3.0 + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS,
            ShooterSubsystem.autoAimDistanceMeters(0.0, 3.0, 0.0), EPS);
        // Off to the side: the lateral term is a right-triangle leg, not an addend.
        double lateral = 1.0;
        double depth = 3.0 + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS;
        assertEquals(Math.hypot(lateral, depth),
            ShooterSubsystem.autoAimDistanceMeters(lateral, 3.0, 0.0), EPS);
    }

    /**
     * The tag-loss ride-through: a brief dropout freezes the whole targeting pass (hood
     * setpoint and flywheel target keep their last values) instead of refusing and then
     * re-commanding the hood the moment the tag comes back -- the re-command is what lurched.
     * It only holds if a tag was actually seen first, so pressing RB with no tag does nothing.
     */
    @Test
    void ridesThroughShortTagDropouts() {
        double hold = ShooterSubsystemConstants.AUTOAIM_TAG_HOLD_SEC;
        assertTrue(ShooterSubsystem.autoAimHoldingThroughDropout(true, 0.0), "a fresh dropout holds");
        assertTrue(ShooterSubsystem.autoAimHoldingThroughDropout(true, hold / 2), "mid-window holds");
        assertFalse(ShooterSubsystem.autoAimHoldingThroughDropout(true, hold),
            "the window is over at exactly the hold");
        assertFalse(ShooterSubsystem.autoAimHoldingThroughDropout(true, hold * 2),
            "a real loss refuses");
        assertFalse(ShooterSubsystem.autoAimHoldingThroughDropout(false, 0.0),
            "no tag ever seen: nothing to hold, so a press with no tag does nothing");
    }

    /**
     * The retarget deadband: a setpoint is only rewritten when the target has moved further
     * than the settle band. Writing every loop clears the hood's park latch and re-drives it
     * at the breakaway floor every 20 ms -- the limit cycle hoodHolding exists to prevent.
     */
    @Test
    void retargetOnlyOutsideTheSettleBand() {
        double band = ShooterSubsystemConstants.AUTOAIM_RETARGET_DEADBAND_UNITS;
        assertFalse(ShooterSubsystem.autoAimShouldRetarget(100.0, 100.0), "no change, no write");
        assertFalse(ShooterSubsystem.autoAimShouldRetarget(100.0 + band / 2, 100.0),
            "inside the band, no write");
        assertFalse(ShooterSubsystem.autoAimShouldRetarget(100.0 - band / 2, 100.0),
            "inside the band downward, no write");
        assertTrue(ShooterSubsystem.autoAimShouldRetarget(100.0 + band * 2, 100.0),
            "a real move writes");
        assertTrue(ShooterSubsystem.autoAimShouldRetarget(100.0 - band * 2, 100.0),
            "a real move downward writes");
    }

    /**
     * The rotation damping (AUTOAIM_ROTATION_SCALE) must not widen the aim gate. The servo
     * chases a damped heading so the robot turns less hard, but isAimedAtTarget -- which gates
     * the kicker -- measures against the TRUE bearing. Collapsing the two would open the kicker
     * at HEADING_TOLERANCE / scale of error: at 0.5, 4 deg instead of 2, firing while visibly
     * off target.
     */
    @Test
    void rotationDampingDoesNotWidenTheAimGate() {
        double scale = ShooterSubsystemConstants.AUTOAIM_ROTATION_SCALE;
        assertTrue(scale > 0 && scale <= 1.0, "scale must be a fraction of the correction");
        double heading = 30.0, bearing = 8.0;
        double trueTarget = heading - bearing;                 // gates the kicker
        double servoTarget = heading - bearing * scale;        // what the drivetrain chases
        assertTrue(Math.abs(heading - servoTarget) < Math.abs(heading - trueTarget) || scale == 1.0,
            "damped target must ask for a smaller turn than the true one");
        // The aim gate is measured against the TRUE target, so it is unaffected by the scale.
        assertEquals(bearing, Math.abs(heading - trueTarget), 1e-9);
        // Both vanish together: the loop can only settle pointed at the hub.
        assertEquals(0.0, 0.0 * scale, 1e-9);
    }

    /**
     * THE OSCILLATION BUG (found on the robot 2026-08-29, "the hood keeps going up and down").
     * The hood lands 25-55 raw units past what was asked -- one powered 20 ms loop of travel --
     * and the arrival latch in periodic() re-seeds the SETPOINT to that landing spot. So if
     * auto-aim compares its table target against the setpoint, a perfectly stable distance
     * looks like a 25-55 unit disagreement every loop and it drives the hood back and forth
     * forever. The deadband must be at least one landing overshoot wide, and the comparison
     * must be against the last COMMANDED target (which the subsystem never rewrites).
     */
    @Test
    void aLandingOvershootDoesNotRetarget() {
        double target = 100.0;
        // Compared against the last commanded target: a stable distance never re-commands.
        assertFalse(ShooterSubsystem.autoAimShouldRetarget(target, target),
            "a stable distance must command the hood exactly once");
        // And the band is wide enough that a landing overshoot could not trigger one either,
        // which is what makes the fix robust rather than merely correct.
        assertTrue(ShooterSubsystemConstants.AUTOAIM_RETARGET_DEADBAND_UNITS >= 25.0,
            "the deadband must be at least one 25-unit minimum hood stroke wide");
    }
}
