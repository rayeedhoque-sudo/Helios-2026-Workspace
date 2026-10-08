package frc.robot.Constants;


import com.pathplanner.lib.util.FlippingUtil;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// 2026 REBUILT field geometry used by the drivetrain (field bounds + auto start spots).
// Every number here is dumped/derived from the official WPILib 2026-rebuilt-welded.json tag
// layout (apriltag-java-2026.2.1.jar) via the shot-math derivation of 2026-07-16 -- cited per
// constant. Tunables (team strategy choices / on-robot unknowns) are non-final per team convention.
public class FieldConstants {

    //FIELD DIMENSIONS (WPILib 2026-rebuilt-welded.json; blue origin at x = 0)
        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS = 8.069;

    //AUTO START SPOT (blue-origin), picked per match on the driver-companion Field tab.
        // The companion publishes one of these names to /Companion/StartSpot; at auto start
        // the robot resets odometry (X, Y AND heading) to that spot. No pick = NO reset.
        // Every spot: robot FACING THE NEUTRAL ZONE (heading 0 deg on blue -- team 2026-09-30),
        // bumpers flush against the named field element.
        // TODO MEASURE ON THE FIELD: all coordinates below are [inferred], not measured.
        //   Trench y = the trench tag y (2026-rebuilt-welded.json tags 17/22/23/28); hub face
        //   x = tags 25/26 (4.022 m). Bump extents were read off the companion's field image
        //   (assets/2026-field.png, calibrated by 2026-rebuilt.json): x 3.99..5.19 m,
        //   left bump y 4.67..6.79 m, right bump y 1.31..3.43 m.
        // ROBOT_HALF_LENGTH: half of the 0.9779 m with-bumpers length (PathPlanner config,
        // team confirmed 2026-09-30 it includes bumpers and is front-to-back).
        public static final double ROBOT_HALF_LENGTH_METERS = 0.9779 / 2.0;
        public static final double HUB_X_METERS = 4.626;            // TODO [inferred] trench/hub centerline x
        public static final double HUB_ALLIANCE_FACE_X_METERS = 4.022; // TODO [inferred] tags 25/26
        public static final double BUMP_ALLIANCE_EDGE_X_METERS = 3.99;  // TODO [inferred] field image

        /** Start spots. Left/right are from OUR driver station's view (blue: left = +Y). */
        public enum StartSpot {
            LEFT_TRENCH(HUB_X_METERS, 7.425),   // TODO [inferred] under the trench, tag 22/23 y
            LEFT_BUMP(BUMP_ALLIANCE_EDGE_X_METERS - ROBOT_HALF_LENGTH_METERS, 5.73), // TODO [inferred]
            HUB(HUB_ALLIANCE_FACE_X_METERS - ROBOT_HALF_LENGTH_METERS, 4.035),       // TODO [inferred]
            RIGHT_BUMP(BUMP_ALLIANCE_EDGE_X_METERS - ROBOT_HALF_LENGTH_METERS, 2.37), // TODO [inferred]
            RIGHT_TRENCH(HUB_X_METERS, 0.644);  // TODO [inferred] under the trench, tag 17/28 y

            /** Blue-alliance pose, facing the neutral zone (+X). */
            public final Pose2d bluePose;

            StartSpot(double xMeters, double yMeters) {
                this.bluePose = new Pose2d(xMeters, yMeters, Rotation2d.kZero);
            }
        }

        /**
         * Pure lookup: the pose for a start-spot name on the given alliance, or empty for a
         * blank/unknown name. Red is the blue pose rotated by PathPlanner's FlippingUtil (same
         * flip AutoBuilder uses for paths), which keeps "left" = the red driver's left.
         */
        public static Optional<Pose2d> startPose(String spotName, boolean isRed) {
            for (StartSpot spot : StartSpot.values()) {
                if (spot.name().equals(spotName)) {
                    return Optional.of(isRed ? FlippingUtil.flipFieldPose(spot.bluePose) : spot.bluePose);
                }
            }
            return Optional.empty();
        }

    //RB AUTO-AIM GEOMETRY (restored 2026-10-07 from commit ccd5075, values unchanged)
        // Geometric hub centers = midpoints of the 4 tag face planes. NOT the mean of the 8 tag
        // positions -- the tags sit pinwheel-asymmetric on the faces (shot-math spec 1.1).
        public static final Translation2d HUB_CENTER_BLUE = new Translation2d(4.6255, 4.0346);
        public static final Translation2d HUB_CENTER_RED = new Translation2d(11.9155, 4.0346);

        // Tag partition (team spec 2026-07-16). Auto-aim only uses OUR alliance's SCORE tags
        // (never the opponent hub); FEED_TAGS is kept only so the tag-17 alias test can pin it.
        public static final Set<Integer> FEED_TAGS = Set.of(1, 3, 4, 6, 17, 19, 20, 22);
        public static final Set<Integer> SCORE_TAGS_RED = Set.of(2, 5, 8, 9, 10, 11);
        public static final Set<Integer> SCORE_TAGS_BLUE = Set.of(18, 21, 24, 25, 26, 27);

        // Every hub tag plane is 0.6035 m in front of the hub center along its inward normal;
        // on each face one tag is centered and one is offset 0.3556 m to a side.
        public static final double TAG_FACE_TO_HUB_DEPTH_METERS = 0.6035;
        public static final double TAG_LATERAL_OFFSET_METERS = 0.3556;
        // Per-tag sign (measured from the 2026 layout, NOT global -- the tags are pinwheel-
        // asymmetric). Positive = hub centre to the RIGHT in the camera image of a robot facing
        // that tag. Centred tags (2, 4, 5, 10, 18, 20, 21, 26) are absent = 0.
        public static final java.util.Map<Integer, Double> TAG_LATERAL_OFFSET_SIGN_BY_TAG =
            java.util.Map.of(
                3, +1.0,  9, +1.0, 11, +1.0,   // red hub: hub centre to the camera's right
                8, -1.0,                        // red hub: the mirrored one -- hub centre LEFT
                19, +1.0, 25, +1.0, 27, +1.0,   // blue hub
                24, -1.0);                      // blue hub: the mirrored one

        // "Aimed At Target" dashboard tolerance (shot-math spec 6).
        public static final double HEADING_TOLERANCE_DEG = 2.0;

        // Own hub center.
        public static Translation2d ownHubCenter(Alliance alliance) {
            return alliance == Alliance.Red ? HUB_CENTER_RED : HUB_CENTER_BLUE;
        }

        // The SCORE tag set for the given alliance (never aim at the opponent hub).
        public static Set<Integer> ownScoreTags(Alliance alliance) {
            return alliance == Alliance.Red ? SCORE_TAGS_RED : SCORE_TAGS_BLUE;
        }

        // Signed lateral offset (m) to ADD TO CAMERA X to go from the tag to its hub centre.
        // 0 for a centred tag and for anything not on a hub.
        public static double tagLateralOffsetMeters(int tagId) {
            return TAG_LATERAL_OFFSET_SIGN_BY_TAG.getOrDefault(tagId, 0.0)
                * TAG_LATERAL_OFFSET_METERS;
        }
}
