package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Sanity checks on the distance -> (hood angle, surface speed) shot tables, so a typo'd
 * seed point fails the build instead of a match shot. Physics expectations (bank-shot
 * policy, 2026-07-17): as distance grows the hood FLATTENS (angle non-increasing) and the
 * wheels SPEED UP (speed non-decreasing); every angle stays inside the hood's mechanical
 * range AND at/above the stow-interlock floor; every speed stays under the motor ceiling.
 */
public class ShotTableTest {

    private static double toMotorRps(double surface) {
        return surface / (2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
            * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION);
    }

    private void checkTables(String name,
            edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap hood,
            edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap speed,
            double dMin, double dMax) {
        double prevHood = Double.POSITIVE_INFINITY;
        double prevSpeed = 0;
        for (double d = dMin; d <= dMax + 1e-9; d += 0.1) {
            double a = hood.get(d);
            double v = speed.get(d) * ShooterSubsystemConstants.SHOT_SPEED_SCALE;
            // Upper bound is the PHYSICAL stop, not the soft MAX_ANGLE limit: the table may seed a
            // close-shot angle above the inset soft cap (it clamps at command time), but it must
            // never exceed the hood's mechanical range.
            assertTrue(a >= ShooterSubsystemConstants.HOOD_STOW_INTERLOCK_FLOOR_DEG
                    && a <= ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP,
                name + " hood angle out of firing band at d=" + d + ": " + a);
            assertTrue(a <= prevHood + 1e-9, name + " hood angle must flatten with distance at d=" + d);
            assertTrue(v >= prevSpeed - 1e-9, name + " speed must grow with distance at d=" + d);
            assertTrue(toMotorRps(v) <= ShooterSubsystemConstants.SHOT_MAX_MOTOR_RPS,
                name + " speed over motor ceiling at d=" + d + ": " + v + " m/s");
            prevHood = a;
            prevSpeed = v;
        }
    }

    @Test
    public void scoreTablesArePhysical() {
        checkTables("SCORE",
            ShooterSubsystemConstants.SCORE_HOOD_DEG_BY_DIST,
            ShooterSubsystemConstants.SCORE_SURFACE_MPS_BY_DIST,
            ShooterSubsystemConstants.MIN_SCORE_DISTANCE_METERS,
            ShooterSubsystemConstants.MAX_SCORE_DISTANCE_METERS);
    }

    @Test
    public void feedTablesArePhysical() {
        checkTables("FEED",
            ShooterSubsystemConstants.FEED_HOOD_DEG_BY_DIST,
            ShooterSubsystemConstants.FEED_SURFACE_MPS_BY_DIST,
            ShooterSubsystemConstants.MIN_FEED_DISTANCE_METERS,
            ShooterSubsystemConstants.MAX_FEED_DISTANCE_METERS);
    }

    @Test
    public void tofFitsAreSane() {
        // Moving-comp uses these every loop; a negative ToF would flip the compensation sign.
        for (double d = 3.0; d <= 9.0; d += 0.5) {
            assertTrue(ShooterSubsystemConstants.SCORE_TOF_BASE_SEC
                + ShooterSubsystemConstants.SCORE_TOF_SEC_PER_METER * d > 0, "score ToF <= 0");
            assertTrue(ShooterSubsystemConstants.FEED_TOF_BASE_SEC
                + ShooterSubsystemConstants.FEED_TOF_SEC_PER_METER * d > 0, "feed ToF <= 0");
        }
    }
}
