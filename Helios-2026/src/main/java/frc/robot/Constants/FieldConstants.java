package frc.robot.Constants;


import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// 2026 REBUILT field geometry used by the drivetrain (field bounds + auto start pose).
// Every number here is dumped/derived from the official WPILib 2026-rebuilt-welded.json tag
// layout (apriltag-java-2026.2.1.jar) via the shot-math derivation of 2026-07-16 -- cited per
// constant. Tunables (team strategy choices / on-robot unknowns) are non-final per team convention.
public class FieldConstants {

    //FIELD DIMENSIONS (WPILib 2026-rebuilt-welded.json; blue origin at x = 0)
        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS = 8.069;

    //AUTO START POSE (blue-origin), used by the odometry autos' start/end pose reset.
        // The robot starts these autos in the RIGHT TRENCH. The exact field coordinates have
        // NOT been measured -- this is a deliberate do-nothing placeholder, not a value.
        // TODO MEASURE ON THE FIELD: blue-alliance right-trench start pose (x m, y m, heading).
        // WHILE THIS IS ZERO the reset writes (0, 0, 0 deg) into odometry, which makes every
        // pose-based feature wrong (auto-aim bearing, G407 zone legality). Do not run these
        // autos in a match until it is filled in.
        // Reference for whoever fills it in: the BLUE trench tags sit at x = 4.588/4.663 m,
        // y = 0.644 m (driver's right) and y = 7.425 m (left) in 2026-rebuilt-welded.json.
        public static Pose2d AUTO_START_POSE_BLUE = Pose2d.kZero;

        /**
         * Auto start pose for the current alliance: the blue value, mirrored by PathPlanner's
         * FlippingUtil on red (same flip AutoBuilder uses for paths, so both stay consistent).
         * TODO verify FlippingUtil.symmetryType matches the 2026 field once the pose is real.
         */
        public static Pose2d autoStartPose() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? FlippingUtil.flipFieldPose(AUTO_START_POSE_BLUE)
                : AUTO_START_POSE_BLUE;
        }
}
