package frc.robot.subsystems.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;

/**
 * RB AUTO-AIM, aim only (restored 2026-10-07 from the V1 ShooterSubsystem, commit ccd5075).
 * Turns the robot so the Limelight -- and the shooter beside it -- points at the HUB CENTRE
 * (not the tag: side tags sit 0.3556 m off-centre). No flywheel, hood or feed: the driver
 * shoots with RT whenever they like.
 *
 * Not a subsystem (no requirement): the RB command calls update() every loop from the
 * drivetrain's request supplier, and the drivetrain servos to the heading it returns.
 * The math helpers are package-private + static for AutoAimTest.
 */
public class AutoAim {

    private final CommandSwerveDrivetrain drivetrain;

    // aimHeadingDeg = the TRUE absolute heading to the hub centre (drives "Aimed At Target").
    // servoHeadingDeg = the same target, low-pass filtered -- what the drivetrain chases.
    private double aimHeadingDeg = 0;
    private double servoHeadingDeg = 0;
    private boolean servoSeeded = false;
    private boolean aimValid = false;
    // Tag-dropout ride-through: counts from the last frame that had a usable tag.
    private boolean hadTag = false;
    private final Timer tagTimer = new Timer();
    private boolean allianceWarned = false;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter Subsystem Tab");
    private final GenericEntry aimedEntry = tab.add("Aimed At Target", false).getEntry();
    private final GenericEntry tagEntry = tab.add("Auto-Aim Tag", -1.0).getEntry();
    private final GenericEntry bearingEntry =
        tab.add("Auto-Aim Bearing (deg, + = hub right of crosshair)", 0.0).getEntry();
    private final GenericEntry viewAngleEntry = tab.add("Auto-Aim View Angle (deg)", 0.0).getEntry();

    public AutoAim(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    /** Start every RB press fresh: no remembered tag, and the servo adopts the first target outright. */
    public void reset(){
        hadTag = false;
        servoSeeded = false;
        aimValid = false;
    }

    /**
     * One aim pass. Returns the blue-origin field heading (deg) the drivetrain should face:
     * the filtered hub heading, or the CURRENT heading when there is no target (hold still
     * rather than chase a stale bearing).
     */
    public double update(){
        double headingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        runAim(headingDeg);
        aimedEntry.setBoolean(aimValid && Math.abs(
            MathUtil.inputModulus(headingDeg - aimHeadingDeg, -180, 180))
                <= FieldConstants.HEADING_TOLERANCE_DEG);
        return aimValid ? servoHeadingDeg : headingDeg;
    }

    private void runAim(double headingDeg){
        // Primary tag must be one of OUR scoring tags (or the tag-17 test alias).
        int primaryId = LimelightHelpers.getTV(Vision.CAM_LIMELIGHT)
            ? (int) LimelightHelpers.getFiducialID(Vision.CAM_LIMELIGHT) : -1;
        var ownTags = FieldConstants.ownScoreTags(ownAlliance());
        if (!(isAliasedScoreTag(primaryId) || ownTags.contains(primaryId))) {
            // Short dropout: keep chasing the last target untouched. Longer: give up and hold.
            if (holdingThroughDropout(hadTag, tagTimer.get())) {
                return;
            }
            hadTag = false;
            aimValid = false;
            return;
        }
        hadTag = true;
        tagTimer.restart();

        // Prefer a centred tag (no lateral correction) over a side tag, then biggest area.
        LimelightHelpers.RawFiducial picked =
            pickAutoAimTag(LimelightHelpers.getRawFiducials(Vision.CAM_LIMELIGHT), ownTags);
        if (picked == null || !(picked.distToCamera > 0.01)) {
            aimValid = false;
            return;
        }
        tagEntry.setDouble(picked.id);
        double groundRange = fiducialGroundRange(picked.tync, picked.distToCamera);
        double camX = fiducialCamX(picked.txnc, groundRange);
        double camZ = fiducialCamZ(picked.txnc, groundRange);
        double lateral = FieldConstants.tagLateralOffsetMeters(picked.id);
        // Tag rotation about the camera's vertical axis (camera space: x right, y DOWN, z fwd).
        double viewAngleDeg = viewAngleDegrees(picked.id, primaryId, Math.toDegrees(
            LimelightHelpers.getTargetPose3d_CameraSpace(Vision.CAM_LIMELIGHT).getRotation().getY()));
        viewAngleEntry.setDouble(viewAngleDeg);
        double[] hub = hubPointInCameraFrame(camX, camZ, lateral, viewAngleDeg);
        if (!(Math.hypot(hub[0], hub[1]) > 0.01)) {   // also catches NaN from a garbage solve
            aimValid = false;
            return;
        }
        double bearingDeg = clampTurnToKeepTagInView(
            Math.toDegrees(Math.atan2(hub[0], hub[1])), picked.txnc);
        bearingEntry.setDouble(bearingDeg);
        aimHeadingDeg = aimHeadingDegrees(headingDeg, bearingDeg);

        // Seed on the first frame; after that only refine while turning slowly (vision lags).
        double omegaDegPerSec = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        if (!servoSeeded) {
            servoHeadingDeg = aimHeadingDeg;
            servoSeeded = true;
        } else if (targetIsTrustworthy(omegaDegPerSec)) {
            servoHeadingDeg = filterServoHeading(servoHeadingDeg, aimHeadingDeg);
        }
        aimValid = true;
    }

    // Own alliance; DriverStation may not know yet -- default BLUE and warn once.
    private Alliance ownAlliance(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            if (!allianceWarned) {
                DriverStation.reportWarning("AutoAim: alliance unknown -- defaulting to BLUE tags", false);
                allianceWarned = true;
            }
            return Alliance.Blue;
        }
        return alliance.get();
    }

    //Pure helpers (no I/O)
        // TEST-ONLY alias: tag 17 counts as a scoring tag while TEST_TAG_ALIAS_ENABLED.
        static boolean isAliasedScoreTag(int tagId){
            return AutoAimConstants.TEST_TAG_ALIAS_ENABLED && tagId == AutoAimConstants.TEST_TAG_ID;
        }

        // Inside the tag-loss ride-through window?
        static boolean holdingThroughDropout(boolean hadTag, double secSinceLastTag){
            return hadTag && secSinceLastTag < AutoAimConstants.TAG_HOLD_SEC;
        }

        // Bearing (deg, + = hub centre to the camera's RIGHT), square-on to the face.
        static double bearingDegrees(double camX, double camZ, double lateralOffsetMeters){
            return Math.toDegrees(Math.atan2(camX + lateralOffsetMeters,
                camZ + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS));
        }

        // Which visible tag to aim from: legal SCORE tags only, centred before offset, then
        // biggest area. Null when nothing qualifies.
        static LimelightHelpers.RawFiducial pickAutoAimTag(
                LimelightHelpers.RawFiducial[] fiducials, java.util.Set<Integer> ownScoreTags){
            if (fiducials == null) {
                return null;
            }
            LimelightHelpers.RawFiducial best = null;
            int bestRank = Integer.MAX_VALUE;
            for (LimelightHelpers.RawFiducial f : fiducials) {
                if (f == null || !(isAliasedScoreTag(f.id) || ownScoreTags.contains(f.id))) {
                    continue;
                }
                int rank = FieldConstants.tagLateralOffsetMeters(f.id) == 0.0 ? 0 : 1;
                if (best == null || rank < bestRank || (rank == bestRank && f.ta > best.ta)) {
                    best = f;
                    bestRank = rank;
                }
            }
            return best;
        }

        // Camera-space GROUND point of a fiducial. distToCamera is a 3D range, so the vertical
        // drop comes out first (cos(tync)) -- this is also why a camera height change needs no
        // code change.
        static double fiducialGroundRange(double tyncDeg, double distToCamera){
            return distToCamera * Math.cos(Math.toRadians(tyncDeg));
        }
        static double fiducialCamX(double txncDeg, double groundRange){
            return groundRange * Math.sin(Math.toRadians(txncDeg));
        }
        static double fiducialCamZ(double txncDeg, double groundRange){
            return groundRange * Math.cos(Math.toRadians(txncDeg));
        }

        // Hub centre in camera ground coords {x right, z forward}: step across the face by the
        // tag's offset and back by the face-to-centre depth, both rotated by the view angle phi.
        static double[] hubPointInCameraFrame(double camXTag, double camZTag, double lateral,
                double phiDeg){
            double phi = Math.toRadians(phiDeg);
            double depth = FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS;
            return new double[] {
                camXTag + lateral * Math.cos(phi) + depth * Math.sin(phi),
                camZTag + depth * Math.cos(phi) - lateral * Math.sin(phi),
            };
        }

        // View angle, only trusted for the Limelight's PRIMARY tag (the only one with a pose).
        static double viewAngleDegrees(int pickedTagId, int primaryTagId, double tagYawDeg){
            if (!AutoAimConstants.VIEW_ANGLE_COMP_ENABLED || pickedTagId != primaryTagId) {
                return 0;
            }
            return tagYawDeg * AutoAimConstants.VIEW_ANGLE_SIGN;
        }

        // Clamp the turn so the tag ends no more than MAX_TAG_OFFSET_DEG off the crosshair.
        static double clampTurnToKeepTagInView(double bearingDeg, double txncDeg){
            return MathUtil.clamp(bearingDeg,
                txncDeg - AutoAimConstants.MAX_TAG_OFFSET_DEG,
                txncDeg + AutoAimConstants.MAX_TAG_OFFSET_DEG);
        }

        // Only refresh the target while turning slowly (a mid-turn bearing is stale).
        static boolean targetIsTrustworthy(double omegaDegPerSec){
            return Math.abs(omegaDegPerSec) <= AutoAimConstants.AIM_UPDATE_MAX_DEG_PER_SEC;
        }

        // Absolute aim heading. SIGN IS PLUS, set ON THE ROBOT 2026-08-29 (minus -- the paper
        // convention -- turned the wrong way and ran the tag out of frame). The cause (Pigeon
        // yaw sense or camera x) was never found. Do not "fix" this without a robot test.
        static double aimHeadingDegrees(double headingDeg, double bearingDeg){
            return headingDeg + bearingDeg;
        }

        // One low-pass step of the servo target toward the aim heading, short way round.
        static double filterServoHeading(double servoDeg, double aimDeg){
            double err = MathUtil.inputModulus(aimDeg - servoDeg, -180, 180);
            return MathUtil.inputModulus(servoDeg + AutoAimConstants.ROTATION_SCALE * err, -180, 180);
        }
}
