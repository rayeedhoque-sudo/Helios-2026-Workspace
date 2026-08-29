package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.FieldConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Pins the auto-aim tag geometry against the REAL 2026 field layout rather than against
 * numbers someone typed in. Every constant these tests check (the 0.6035 m face-to-hub
 * depth, the 0.3556 m lateral offset, and the per-tag SIGN of that offset) is re-derived
 * here from AprilTagFields and compared with FieldConstants.
 *
 * This is what makes the auto-aim rotation trustworthy: the tags sit pinwheel-asymmetric on
 * the hub, so a single global sign is wrong for some of them, and aiming at the tag instead
 * of the hub centre misses by ~6.8 deg at 3 m -- over 3x the aim tolerance.
 */
class AutoAimGeometryTest {

    private static final AprilTagFieldLayout LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** Hub-centre vector for one tag, rotated into that tag's own frame. */
    private static double[] lateralAndDepth(int tagId) {
        Pose3d tag = LAYOUT.getTagPose(tagId).orElseThrow();
        Translation2d hub = tag.getX() > FieldConstants.FIELD_LENGTH_METERS / 2
            ? FieldConstants.ownHubCenter(Alliance.Red)
            : FieldConstants.ownHubCenter(Alliance.Blue);
        double dx = hub.getX() - tag.getX();
        double dy = hub.getY() - tag.getY();
        double yaw = tag.getRotation().getZ();      // tag's outward normal
        // lateral: across the face, positive = hub centre to the RIGHT in the image of a robot
        // facing this tag. depth: along the outward normal, so negative = centre behind the tag.
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
        double bearing = ShooterSubsystem.autoAimBearingDegrees(0.0, camZ, lateral);
        assertTrue(Math.abs(bearing) > 3 * FieldConstants.HEADING_TOLERANCE_DEG,
            "an offset tag is worth >3x the aim tolerance, got " + bearing + " deg");
        assertTrue(bearing > 0, "hub centre is to the camera's RIGHT, so the robot turns right");
        // A centred tag needs no turn at all.
        assertEquals(0.0,
            ShooterSubsystem.autoAimBearingDegrees(0.0, camZ, FieldConstants.tagLateralOffsetMeters(10)),
            1e-9);
    }

    /** Bearing and distance must be built from the SAME corrected point. */
    @Test
    void bearingAndDistanceAgree() {
        double lateral = FieldConstants.tagLateralOffsetMeters(25);
        double camX = 0.12, camZ = 2.4;
        double d = ShooterSubsystem.autoAimDistanceMeters(camX, camZ, lateral);
        double b = Math.toRadians(ShooterSubsystem.autoAimBearingDegrees(camX, camZ, lateral));
        // Polar and cartesian descriptions of one point.
        assertEquals(camX + lateral, d * Math.sin(b), 1e-9);
        assertEquals(camZ + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS, d * Math.cos(b), 1e-9);
    }
}
