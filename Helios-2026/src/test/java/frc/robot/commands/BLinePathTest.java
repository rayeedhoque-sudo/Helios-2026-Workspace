package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.Optional;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants.FieldConstants;
import frc.robot.lib.BLine.Path;

/**
 * Proves the BLine files in src/main/deploy/autos parse at the desk, and pins the
 * start-spot guard (CommandSwerveDrivetrain.startSpotMatchesPath).
 */
public class BLinePathTest {
    private static Path path;

    @BeforeAll
    static void load() {
        HAL.initialize(500, 0);
        path = new Path(new File("src/main/deploy/autos"), "forward-3m-left-3m");
    }

    @Test
    void testPathParsesAndStartsAtRightTrench() {
        assertTrue(path.isValid());
        assertEquals(0.0, path.getStartPose().getTranslation().getDistance(
            FieldConstants.StartSpot.RIGHT_TRENCH.bluePose.getTranslation()), 1e-9);
        // config.json must win over BLine's 4.5 m/s / 7 m/s^2 fallback (team: match PathPlanner).
        assertEquals(3.0, path.getDefaultGlobalConstraints().getMaxVelocityMetersPerSec(), 1e-9);
        assertEquals(3.0, path.getDefaultGlobalConstraints().getMaxAccelerationMetersPerSec2(), 1e-9);
    }

    @Test
    void guardOnlyPassesForTheMatchingStartSpot() {
        assertTrue(CommandSwerveDrivetrain.startSpotMatchesPath(path, false,
            FieldConstants.startPose("RIGHT_TRENCH", false)));
        assertTrue(CommandSwerveDrivetrain.startSpotMatchesPath(path, true,
            FieldConstants.startPose("RIGHT_TRENCH", true)), "red flip lines up with red spot");
        assertFalse(CommandSwerveDrivetrain.startSpotMatchesPath(path, false,
            FieldConstants.startPose("HUB", false)), "wrong spot refuses");
        assertFalse(CommandSwerveDrivetrain.startSpotMatchesPath(path, true,
            FieldConstants.startPose("RIGHT_TRENCH", false)), "blue spot on red refuses");
        assertFalse(CommandSwerveDrivetrain.startSpotMatchesPath(path, false, Optional.empty()),
            "no pick refuses");
    }
}
