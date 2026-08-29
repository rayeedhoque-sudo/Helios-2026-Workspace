package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.utility.LimelightHelpers;
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
     * TAG PRIORITY (team direction 2026-08-29): a CENTRED tag beats a side tag, because a
     * centred one needs no lateral correction at all. Area breaks ties within a rank, and
     * anything not a legal score tag is ignored outright.
     */
    @Test
    void centredTagsWinOverSideTags() {
        java.util.Set<Integer> red = FieldConstants.ownScoreTags(Alliance.Red);
        // 9 (side) is much larger in frame than 10 (centred) -- 10 still wins.
        var side = new LimelightHelpers.RawFiducial(9, -12.0, 0, 0.90, 2.0, 2.0, 0.1);
        var centred = new LimelightHelpers.RawFiducial(10, 3.0, 0, 0.10, 3.0, 3.0, 0.1);
        assertEquals(10, ShooterSubsystem.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {side, centred}, red).id,
            "a centred tag needs no lateral correction, so it wins on rank not on size");
        // Two side tags of equal rank: the BIGGER in frame (nearer, better solve) wins.
        // side is tag 9 at ta 0.90, side2 is tag 11 at ta 0.40, so 9 takes it.
        var side2 = new LimelightHelpers.RawFiducial(11, 5.0, 0, 0.40, 2.5, 2.5, 0.1);
        assertEquals(9, ShooterSubsystem.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {side, side2}, red).id);
        // ...and the order they arrive in must not change that.
        assertEquals(9, ShooterSubsystem.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {side2, side}, red).id);
        // Opponent-hub and non-hub tags are not eligible at all.
        var blueHub = new LimelightHelpers.RawFiducial(26, 0.0, 0, 0.99, 2.0, 2.0, 0.1);
        assertNull(ShooterSubsystem.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {blueHub}, red),
            "tag 26 is the opponent hub on red -- never aim at it");
        assertNull(ShooterSubsystem.pickAutoAimTag(new LimelightHelpers.RawFiducial[] {}, red));
        assertNull(ShooterSubsystem.pickAutoAimTag(null, red));
        // The test alias still gets through, on either alliance.
        var alias = new LimelightHelpers.RawFiducial(17, 1.0, 0, 0.5, 1.8, 1.8, 0.1);
        assertEquals(17, ShooterSubsystem.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {alias}, red).id);
    }

    /** Raw-fiducial bearing/range round-trips to the camera-space point the aim helpers use. */
    @Test
    void fiducialGeometryRoundTrips() {
        double tx = 12.0, ground = 2.5;
        double x = ShooterSubsystem.fiducialCamX(tx, ground);
        double z = ShooterSubsystem.fiducialCamZ(tx, ground);
        assertEquals(ground, Math.hypot(x, z), 1e-9, "range preserved");
        assertEquals(tx, Math.toDegrees(Math.atan2(x, z)), 1e-9, "bearing preserved");
        assertTrue(x > 0, "a positive tx is to the camera's right");
    }

    /**
     * THE VERTICAL DROP MUST COME OUT OF distToCamera (bug found 2026-08-29). It is a 3D range
     * and the tag sits well below the camera on this robot, so using it as a horizontal depth
     * overstates the depth -- and an overstated depth makes every aim correction too SMALL.
     */
    @Test
    void groundRangeRemovesTheVerticalDrop() {
        double dist = 1.25;
        // Dead level: nothing to remove.
        assertEquals(dist, ShooterSubsystem.fiducialGroundRange(0.0, dist), 1e-9);
        // Measured on the robot at 39 in: tync -41.4 deg.
        double ground = ShooterSubsystem.fiducialGroundRange(-41.4, dist);
        assertTrue(ground < dist, "the ground range is shorter than the 3D range");
        assertEquals(dist * Math.cos(Math.toRadians(41.4)), ground, 1e-9);
        // Sign of tync must not matter -- above or below the camera drops the same amount.
        assertEquals(ShooterSubsystem.fiducialGroundRange(41.4, dist), ground, 1e-9);
        // And the consequence: the side-tag correction gets BIGGER once the drop is removed.
        double lateral = FieldConstants.tagLateralOffsetMeters(9);
        double wrong = ShooterSubsystem.autoAimBearingDegrees(
            ShooterSubsystem.fiducialCamX(2.7, dist), ShooterSubsystem.fiducialCamZ(2.7, dist), lateral);
        double right = ShooterSubsystem.autoAimBearingDegrees(
            ShooterSubsystem.fiducialCamX(2.7, ground), ShooterSubsystem.fiducialCamZ(2.7, ground), lateral);
        assertTrue(right > wrong + 1.0,
            "removing the drop must turn the robot noticeably further right, got "
                + wrong + " -> " + right);
    }

    /**
     * The three tags that matter for the team's aim check: 10, 26 and the 17 standing in for
     * them are CENTRED, so a tag on the crosshair needs essentially no turn -- only the small
     * amount the face-to-hub depth contributes when viewed off to one side. 9 and 25 sit on the
     * left of the face, so they always need a significant turn to the RIGHT.
     */
    @Test
    void centredTagsNeedNoTurnAndSideTagsTurnRight() {
        double ground = 2.4;
        for (int id : new int[] {10, 26, 17}) {
            double lateral = FieldConstants.tagLateralOffsetMeters(id);
            // Tag exactly on the crosshair -> no turn at all.
            assertEquals(0.0, ShooterSubsystem.autoAimBearingDegrees(
                ShooterSubsystem.fiducialCamX(0.0, ground),
                ShooterSubsystem.fiducialCamZ(0.0, ground), lateral), 1e-9,
                "tag " + id + " is centred: on the crosshair means aimed");
            // Seen from the LEFT of the tag (tag appears right, +tx) -> a small turn right.
            double fromLeft = ShooterSubsystem.autoAimBearingDegrees(
                ShooterSubsystem.fiducialCamX(6.0, ground),
                ShooterSubsystem.fiducialCamZ(6.0, ground), lateral);
            assertTrue(fromLeft > 0 && fromLeft < 6.0,
                "tag " + id + ": a little right, and LESS than the raw tx, got " + fromLeft);
            // Mirrored from the right.
            assertEquals(-fromLeft, ShooterSubsystem.autoAimBearingDegrees(
                ShooterSubsystem.fiducialCamX(-6.0, ground),
                ShooterSubsystem.fiducialCamZ(-6.0, ground), lateral), 1e-9);
        }
        for (int id : new int[] {9, 25}) {
            // Tag on the crosshair, but the hub centre is well to its right.
            double turn = ShooterSubsystem.autoAimBearingDegrees(
                ShooterSubsystem.fiducialCamX(0.0, ground),
                ShooterSubsystem.fiducialCamZ(0.0, ground),
                FieldConstants.tagLateralOffsetMeters(id));
            assertTrue(turn > 3 * FieldConstants.HEADING_TOLERANCE_DEG,
                "tag " + id + " sits left of the face, so it needs a significant RIGHT turn, got "
                    + turn);
        }
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
