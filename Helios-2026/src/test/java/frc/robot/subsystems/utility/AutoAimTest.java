package frc.robot.subsystems.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;

/**
 * Pins the RB auto-aim (aim only) math. Ported 2026-10-07 from the V1 AutoAimTest +
 * AutoAimGeometryTest (commit ccd5075), minus the hood/flywheel cases. The geometry tests
 * re-derive the tag offsets from the real 2026 field layout instead of trusting typed numbers.
 */
class AutoAimTest {

    private static final AprilTagFieldLayout LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** Hub-centre vector for one tag, rotated into that tag's own frame: {lateral, depth}. */
    private static double[] lateralAndDepth(int tagId) {
        Pose3d tag = LAYOUT.getTagPose(tagId).orElseThrow();
        Translation2d hub = tag.getX() > FieldConstants.FIELD_LENGTH_METERS / 2
            ? FieldConstants.ownHubCenter(Alliance.Red)
            : FieldConstants.ownHubCenter(Alliance.Blue);
        double dx = hub.getX() - tag.getX();
        double dy = hub.getY() - tag.getY();
        double yaw = tag.getRotation().getZ();
        return new double[] {
            -dx * Math.sin(yaw) + dy * Math.cos(yaw),
            dx * Math.cos(yaw) + dy * Math.sin(yaw),
        };
    }

    /** The four tags the team asked auto-aim to work on, plus their mirrors. */
    @Test
    void everyHubTagMatchesTheDeclaredOffsetAndDepth() {
        int[] hubTags = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};
        for (int id : hubTags) {
            double[] ld = lateralAndDepth(id);
            assertEquals(-FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS, ld[1], 5e-4,
                "tag " + id + ": hub centre must sit TAG_FACE_TO_HUB_DEPTH behind the tag face");
            assertEquals(FieldConstants.tagLateralOffsetMeters(id), ld[0], 5e-4,
                "tag " + id + ": declared lateral offset must match the field layout");
        }
    }

    /** The team's four working tags: 9 and 25 offset, 10 and 26 dead centre. */
    @Test
    void theFourAutoAimTagsAreLeftOrCentred() {
        for (int id : new int[] {9, 25}) {
            assertEquals(FieldConstants.TAG_LATERAL_OFFSET_METERS,
                FieldConstants.tagLateralOffsetMeters(id), 5e-4,
                "tag " + id + " is LEFT of centre, so the hub centre is to the camera's right");
        }
        for (int id : new int[] {10, 26}) {
            assertEquals(0.0, FieldConstants.tagLateralOffsetMeters(id), 5e-4,
                "tag " + id + " is centred on its face -- nothing to correct");
        }
    }

    /**
     * The sign is NOT global: tags 8 and 24 are mirrored, so one global sign would aim
     * ~0.71 m off on those faces -- twice the offset, in the wrong direction.
     */
    @Test
    void theOffsetSignIsNotGlobal() {
        assertTrue(FieldConstants.tagLateralOffsetMeters(9) > 0, "tag 9 positive");
        assertTrue(FieldConstants.tagLateralOffsetMeters(8) < 0, "tag 8 is mirrored");
        assertTrue(FieldConstants.tagLateralOffsetMeters(24) < 0, "tag 24 is mirrored");
        // Non-hub tags must contribute nothing rather than a default guess.
        assertEquals(0.0, FieldConstants.tagLateralOffsetMeters(7), 1e-9);
        assertEquals(0.0, FieldConstants.tagLateralOffsetMeters(17), 1e-9);
    }

    /** Aiming at the tag instead of the hub centre is a real miss, not a rounding error. */
    @Test
    void aimingAtTheTagWouldMissOnAnOffsetFace() {
        double lateral = FieldConstants.tagLateralOffsetMeters(9);
        // Tag dead ahead, robot 3 m from the HUB CENTRE (so the tag face is nearer than that
        // by the face-to-hub depth). Stating it in hub distance matters: quoting the same
        // bearing "at 3 m" of tag depth would be 5.6 deg, not 6.8 -- the depth term moves it.
        double camZ = 3.0 - FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS;
        double bearing = AutoAim.bearingDegrees(0.0, camZ, lateral);
        assertTrue(Math.abs(bearing) > 3 * FieldConstants.HEADING_TOLERANCE_DEG,
            "an offset tag is worth >3x the aim tolerance, got " + bearing + " deg");
        assertTrue(bearing > 0, "hub centre is to the camera's RIGHT, so the robot turns right");
        // A centred tag needs no turn at all.
        assertEquals(0.0,
            AutoAim.bearingDegrees(0.0, camZ, FieldConstants.tagLateralOffsetMeters(10)),
            1e-9);
    }

    /**
     * WHICH TAGS AUTO-AIM ACTUALLY ACCEPTS. classifyTag returns SCORE for the test alias
     * first, then for our OWN alliance's score tags -- so the four tags the team named are
     * split by alliance and can never all be live at once. That is deliberate (never shoot
     * into the opponent hub, G407) but it is exactly the kind of thing that looks like a bug
     * on the field, so it is pinned here.
     */
    @Test
    void theNamedTagsAreAcceptedOnTheRightAlliance() {
        // 9 and 10 are RED hub faces.
        assertTrue(FieldConstants.ownScoreTags(Alliance.Red).contains(9));
        assertTrue(FieldConstants.ownScoreTags(Alliance.Red).contains(10));
        assertFalse(FieldConstants.ownScoreTags(Alliance.Blue).contains(9),
            "tag 9 is the opponent hub on blue -- auto-aim must refuse it");
        assertFalse(FieldConstants.ownScoreTags(Alliance.Blue).contains(10),
            "tag 10 is the opponent hub on blue -- auto-aim must refuse it");
        // 25 and 26 are BLUE hub faces.
        assertTrue(FieldConstants.ownScoreTags(Alliance.Blue).contains(25));
        assertTrue(FieldConstants.ownScoreTags(Alliance.Blue).contains(26));
        assertFalse(FieldConstants.ownScoreTags(Alliance.Red).contains(25));
        assertFalse(FieldConstants.ownScoreTags(Alliance.Red).contains(26));
    }

    /**
     * The TEST alias tag (18 since 2026-10-09 -- a centred blue SCORE tag) must reach
     * auto-aim on RED too, where it belongs to the opponent hub. Lateral offset 0, like the
     * centred tag 10 it stands in for.
     */
    @Test
    void testAliasTagStillReachesAutoAim() {
        assertTrue(AutoAim.isAliasedScoreTag(18),
            "the alias is what makes 18 usable on red -- if this is false, RB does nothing on it");
        assertFalse(FieldConstants.ownScoreTags(Alliance.Red).contains(18));
        assertEquals(FieldConstants.tagLateralOffsetMeters(10),
            FieldConstants.tagLateralOffsetMeters(18), 1e-9,
            "18 must aim like the tag 10 it stands in for");
    }

    /** Tag 17 is aliased to a scoring tag for testing; nothing else is. */
    @Test
    void aliasCoversOnlyTheTestTag() {
        assertTrue(AutoAim.isAliasedScoreTag(AutoAimConstants.TEST_TAG_ID));
        assertFalse(AutoAim.isAliasedScoreTag(10), "a real score tag is not the alias");
        assertFalse(AutoAim.isAliasedScoreTag(1), "an unrelated feed tag is not the alias");
    }

    /**
     * The tag-loss ride-through: a brief dropout freezes the whole targeting pass (hood
     * setpoint and flywheel target keep their last values) instead of refusing and then
     * re-commanding the hood the moment the tag comes back -- the re-command is what lurched.
     * It only holds if a tag was actually seen first, so pressing RB with no tag does nothing.
     */
    @Test
    void ridesThroughShortTagDropouts() {
        double hold = AutoAimConstants.TAG_HOLD_SEC;
        assertTrue(AutoAim.holdingThroughDropout(true, 0.0), "a fresh dropout holds");
        assertTrue(AutoAim.holdingThroughDropout(true, hold / 2), "mid-window holds");
        assertFalse(AutoAim.holdingThroughDropout(true, hold),
            "the window is over at exactly the hold");
        assertFalse(AutoAim.holdingThroughDropout(true, hold * 2),
            "a real loss refuses");
        assertFalse(AutoAim.holdingThroughDropout(false, 0.0),
            "no tag ever seen: nothing to hold, so a press with no tag does nothing");
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
        assertEquals(10, AutoAim.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {side, centred}, red).id,
            "a centred tag needs no lateral correction, so it wins on rank not on size");
        // Two side tags of equal rank: the BIGGER in frame (nearer, better solve) wins.
        // side is tag 9 at ta 0.90, side2 is tag 11 at ta 0.40, so 9 takes it.
        var side2 = new LimelightHelpers.RawFiducial(11, 5.0, 0, 0.40, 2.5, 2.5, 0.1);
        assertEquals(9, AutoAim.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {side, side2}, red).id);
        // ...and the order they arrive in must not change that.
        assertEquals(9, AutoAim.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {side2, side}, red).id);
        // Opponent-hub and non-hub tags are not eligible at all.
        var blueHub = new LimelightHelpers.RawFiducial(26, 0.0, 0, 0.99, 2.0, 2.0, 0.1);
        assertNull(AutoAim.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {blueHub}, red),
            "tag 26 is the opponent hub on red -- never aim at it");
        assertNull(AutoAim.pickAutoAimTag(new LimelightHelpers.RawFiducial[] {}, red));
        assertNull(AutoAim.pickAutoAimTag(null, red));
        // The test alias still gets through, on either alliance.
        var alias = new LimelightHelpers.RawFiducial(18, 1.0, 0, 0.5, 1.8, 1.8, 0.1);
        assertEquals(18, AutoAim.pickAutoAimTag(
            new LimelightHelpers.RawFiducial[] {alias}, red).id);
    }

    /** Raw-fiducial bearing/range round-trips to the camera-space point the aim helpers use. */
    @Test
    void fiducialGeometryRoundTrips() {
        double tx = 12.0, ground = 2.5;
        double x = AutoAim.fiducialCamX(tx, ground);
        double z = AutoAim.fiducialCamZ(tx, ground);
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
        assertEquals(dist, AutoAim.fiducialGroundRange(0.0, dist), 1e-9);
        // Measured on the robot at 39 in: tync -41.4 deg.
        double ground = AutoAim.fiducialGroundRange(-41.4, dist);
        assertTrue(ground < dist, "the ground range is shorter than the 3D range");
        assertEquals(dist * Math.cos(Math.toRadians(41.4)), ground, 1e-9);
        // Sign of tync must not matter -- above or below the camera drops the same amount.
        assertEquals(AutoAim.fiducialGroundRange(41.4, dist), ground, 1e-9);
        // And the consequence: the side-tag correction gets BIGGER once the drop is removed.
        double lateral = FieldConstants.tagLateralOffsetMeters(9);
        double wrong = AutoAim.bearingDegrees(
            AutoAim.fiducialCamX(2.7, dist), AutoAim.fiducialCamZ(2.7, dist), lateral);
        double right = AutoAim.bearingDegrees(
            AutoAim.fiducialCamX(2.7, ground), AutoAim.fiducialCamZ(2.7, ground), lateral);
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
            assertEquals(0.0, AutoAim.bearingDegrees(
                AutoAim.fiducialCamX(0.0, ground),
                AutoAim.fiducialCamZ(0.0, ground), lateral), 1e-9,
                "tag " + id + " is centred: on the crosshair means aimed");
            // Seen from the LEFT of the tag (tag appears right, +tx) -> a small turn right.
            double fromLeft = AutoAim.bearingDegrees(
                AutoAim.fiducialCamX(6.0, ground),
                AutoAim.fiducialCamZ(6.0, ground), lateral);
            assertTrue(fromLeft > 0 && fromLeft < 6.0,
                "tag " + id + ": a little right, and LESS than the raw tx, got " + fromLeft);
            // Mirrored from the right.
            assertEquals(-fromLeft, AutoAim.bearingDegrees(
                AutoAim.fiducialCamX(-6.0, ground),
                AutoAim.fiducialCamZ(-6.0, ground), lateral), 1e-9);
        }
        for (int id : new int[] {9, 25}) {
            // Tag on the crosshair, but the hub centre is well to its right.
            double turn = AutoAim.bearingDegrees(
                AutoAim.fiducialCamX(0.0, ground),
                AutoAim.fiducialCamZ(0.0, ground),
                FieldConstants.tagLateralOffsetMeters(id));
            assertTrue(turn > 3 * FieldConstants.HEADING_TOLERANCE_DEG,
                "tag " + id + " sits left of the face, so it needs a significant RIGHT turn, got "
                    + turn);
        }
    }

    /**
     * THE TAG MUST STAY IN VIEW (team requirement 2026-08-29, after the robot turned itself
     * until the tag left frame). After turning by the commanded bearing b, a tag at txnc sits
     * at (txnc - b) in the image -- so clamping b to within MAX_TAG_OFFSET of txnc is a hard
     * guarantee about where the tag ends up, not a hope.
     */
    @Test
    void theCommandedTurnCanNeverSweepPastTheTag() {
        double max = AutoAimConstants.MAX_TAG_OFFSET_DEG;
        for (double txnc : new double[] {0.0, 6.0, -6.0, 20.0, -20.0}) {
            for (double raw : new double[] {0.0, 45.0, -45.0, 200.0, -200.0}) {
                double b = AutoAim.clampTurnToKeepTagInView(raw, txnc);
                assertTrue(Math.abs(txnc - b) <= max + 1e-9,
                    "tag would end " + (txnc - b) + " deg off the crosshair (max " + max + ")");
            }
        }
        // A sane correction passes through untouched -- the clamp is a guard, not the behaviour.
        double sane = AutoAim.bearingDegrees(
            AutoAim.fiducialCamX(6.0, 2.4), AutoAim.fiducialCamZ(6.0, 2.4),
            FieldConstants.tagLateralOffsetMeters(10));
        assertEquals(sane, AutoAim.clampTurnToKeepTagInView(sane, 6.0), 1e-9,
            "a centred tag's small correction must not be clipped");
    }

    /**
     * The aim target may only be refreshed while the robot is turning slowly. Vision lags a few
     * frames, so a bearing measured mid-turn belongs to a heading the robot has already left;
     * subtracting it from a LIVE heading yields a target that advances with the robot, and the
     * robot chases it round. That is what drove it off the tag.
     */
    @Test
    void theAimTargetIsHeldWhileTurningFast() {
        double max = AutoAimConstants.AIM_UPDATE_MAX_DEG_PER_SEC;
        assertTrue(AutoAim.targetIsTrustworthy(0.0), "standing still: trust it");
        assertTrue(AutoAim.targetIsTrustworthy(max / 2));
        assertTrue(AutoAim.targetIsTrustworthy(-max / 2), "direction is irrelevant");
        assertFalse(AutoAim.targetIsTrustworthy(max * 2),
            "mid-turn the measurement is stale -- hold the target");
        assertFalse(AutoAim.targetIsTrustworthy(-max * 2));
    }

    /**
     * VIEWING-ANGLE COMPENSATION. The hub centre is 0.6035 m behind the tag along the TAG'S
     * normal; stepping back along the CAMERA's axis instead is only the same thing when square
     * to the face. phi = 0 must reproduce the old maths exactly, and a nonzero phi must move
     * the aim point across the line by depth*sin(phi).
     */
    @Test
    void viewAngleShiftsTheAimPointAcrossTheLine() {
        double camX = 0.1, camZ = 2.4;
        double lateral = FieldConstants.tagLateralOffsetMeters(10);   // centred tag, 0
        // Square on: identical to the uncompensated helpers, to the last bit.
        double[] square = AutoAim.hubPointInCameraFrame(camX, camZ, lateral, 0.0);
        assertEquals(camX + lateral, square[0], 1e-12);
        assertEquals(camZ + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS, square[1], 1e-12);

        // Viewed from one side: the aim point steps across by depth*sin(phi).
        double phi = 20.7;   // a real reading off this robot on the bench
        double[] angled = AutoAim.hubPointInCameraFrame(camX, camZ, lateral, phi);
        double expectedShift = FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS
            * Math.sin(Math.toRadians(phi));
        assertEquals(square[0] + expectedShift, angled[0], 1e-9);
        assertTrue(Math.abs(expectedShift) > 0.2, "20.7 deg is worth >0.2 m, not a rounding error");
        // Mirrored view mirrors the shift.
        assertEquals(square[0] - expectedShift,
            AutoAim.hubPointInCameraFrame(camX, camZ, lateral, -phi)[0], 1e-9);
    }

    /** The compensation never applies to a tag that is not the camera's primary. */
    @Test
    void viewAngleOnlyEverUsesThePrimaryTag() {
        // targetpose_cameraspace always describes the PRIMARY tag, so reading it for a tag
        // auto-aim picked separately would silently mix two different tags. No orientation for
        // that tag means fall back to the square-on assumption, which is 0.
        assertEquals(0.0, AutoAim.viewAngleDegrees(10, 26, 25.0), 1e-12);
        // Enabled and looking at the primary: the reported angle comes through, signed.
        double v = AutoAim.viewAngleDegrees(10, 10, 25.0);
        assertEquals(25.0 * AutoAimConstants.VIEW_ANGLE_SIGN, v, 1e-12);
        assertTrue(AutoAimConstants.VIEW_ANGLE_COMP_ENABLED,
            "enabled 2026-08-29 by team direction");
        // The gate is the enable flag, so switching it off restores square-on behaviour whole.
        assertEquals(-AutoAim.viewAngleDegrees(10, 10, 25.0),
            AutoAim.viewAngleDegrees(10, 10, -25.0), 1e-12,
            "mirrored view mirrors the compensation");
    }

    /**
     * THE ROTATION SIGN IS EMPIRICAL (team, 2026-08-29): heading + bearing. The textbook minus
     * turned the robot the wrong way and ran the tag out of frame. A "fix" back to minus has
     * to argue with the robot first.
     */
    @Test
    void theAimHeadingUsesTheEmpiricalSign() {
        assertEquals(26.0, AutoAim.aimHeadingDegrees(20.0, 6.0), 1e-9);
        assertEquals(14.0, AutoAim.aimHeadingDegrees(20.0, -6.0), 1e-9);
    }

    /** The servo filter steps a fraction of the way, the SHORT way round +-180, and settles on target. */
    @Test
    void servoFilterTakesTheShortWayAndSettles() {
        double scale = AutoAimConstants.ROTATION_SCALE;
        assertEquals(10.0 * scale, AutoAim.filterServoHeading(0.0, 10.0), 1e-9);
        // 170 -> -170 is 20 deg the short way (through 180), not 340 the long way.
        double step = AutoAim.filterServoHeading(170.0, -170.0);
        assertEquals(170.0 + 20.0 * scale, step, 1e-9);
        double s = 0.0;
        for (int i = 0; i < 200; i++) {
            s = AutoAim.filterServoHeading(s, 30.0);
        }
        assertEquals(30.0, s, 1e-6, "settles exactly on the aim heading");
    }
}
