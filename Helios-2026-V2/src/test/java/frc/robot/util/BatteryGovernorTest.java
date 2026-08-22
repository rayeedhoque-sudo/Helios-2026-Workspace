package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * Pins the governor's pure tier ladder, so a flipped comparison or a lost hysteresis gap fails
 * the build instead of the robot: a sag must shed only after the dwell, must not flap back at
 * the same voltage it tripped on, and full charge must always read NORMAL.
 */
public class BatteryGovernorTest {

    private static final double LONG = BatteryGovernor.DWELL_SEC + 0.01;
    private static final double SHORT = BatteryGovernor.DWELL_SEC - 0.01;

    @Test
    void healthyBatteryStaysNormal() {
        assertEquals(0, BatteryGovernor.step(0, 12.4, LONG));
        assertEquals(0, BatteryGovernor.step(0, 8.0, LONG));
        assertEquals(1.0, BatteryGovernor.scaleFor(0));
    }

    @Test
    void shedsOnlyAfterTheDwell() {
        // Wanted, but not held long enough -- a single accel sag must not cut anything.
        assertEquals(0, BatteryGovernor.step(0, 7.6, SHORT));
        assertEquals(0, BatteryGovernor.step(0, 6.9, SHORT));
        // Held past the dwell -> one tier, never two at once.
        assertEquals(1, BatteryGovernor.step(0, 7.6, LONG));
        assertEquals(1, BatteryGovernor.step(0, 6.9, LONG));
        assertEquals(2, BatteryGovernor.step(1, 7.0, LONG));
    }

    @Test
    void recoversOnlyAboveTheHigherThreshold() {
        // In the deadband between trip (7.6) and recover (8.0): hold tier 1, no flapping.
        assertEquals(1, BatteryGovernor.step(1, 7.8, LONG));
        assertEquals(1, BatteryGovernor.step(1, 7.99, LONG));
        // At/above recover, held past the dwell.
        assertEquals(0, BatteryGovernor.step(1, 8.0, LONG));
        assertEquals(1, BatteryGovernor.step(2, 7.6, LONG));
        // ...but not before it.
        assertEquals(1, BatteryGovernor.step(1, 12.4, SHORT));
    }

    @Test
    void ladderIsSelfBounding() {
        // Tier 0 never climbs above itself; the last tier never sheds past itself.
        assertEquals(0, BatteryGovernor.step(0, 13.0, LONG));
        assertEquals(2, BatteryGovernor.step(2, 5.0, LONG));
    }

    @Test
    void deeperTierShedsMoreSupply() {
        assertEquals(0.75, BatteryGovernor.scaleFor(1));
        assertEquals(0.50, BatteryGovernor.scaleFor(2));
    }

    @Test
    void theShooterAlwaysWins() {
        final int SHOT = BatteryGovernor.TIER_SHOT;   // flywheels: shed last
        final int FEED = BatteryGovernor.TIER_FEED;   // roller/belts/slider/hood: shed first

        // Tier 1 sheds feed and acquisition but leaves the shot bit-for-bit untouched.
        assertEquals(0.75, BatteryGovernor.scaleFor(1, FEED, false, false));
        assertEquals(1.0, BatteryGovernor.scaleFor(1, SHOT, true, false));

        // Tier 2 finally reaches the flywheels -- but only when no shot is live.
        assertEquals(0.50, BatteryGovernor.scaleFor(2, SHOT, true, false));
        assertEquals(1.0, BatteryGovernor.scaleFor(2, SHOT, true, true));

        // The interlock is per-motor: a live shot must NOT hand the feed motors their amps back.
        assertEquals(0.50, BatteryGovernor.scaleFor(2, FEED, false, true));

        // Full charge is full limits for everyone, shot or no shot.
        assertEquals(1.0, BatteryGovernor.scaleFor(0, FEED, false, false));
        assertEquals(1.0, BatteryGovernor.scaleFor(0, SHOT, true, true));
    }

    @Test
    void theFloorIsWhatKeepsAShedSafe() {
        // Flywheels as registered: 25 A configured, 15 A floor.
        assertEquals(25.0, BatteryGovernor.supplyFor(25, BatteryGovernor.scaleFor(0), 15));
        assertEquals(18.75, BatteryGovernor.supplyFor(25, BatteryGovernor.scaleFor(1), 15));
        // Tier 2 would ask for 12.5 A -- the floor must refuse it, or the flywheels can no longer
        // re-accelerate between balls (the failure the 2026-07-17 anti-bog raise cured).
        assertEquals(15.0, BatteryGovernor.supplyFor(25, BatteryGovernor.scaleFor(2), 15));
        // A floor above the configured limit can never RAISE a motor's draw.
        assertEquals(25.0, BatteryGovernor.supplyFor(25, 1.0, 40));
    }

    @Test
    void stallCutoffsSurviveEveryTier() {
        // The slider and hood are floored AT their base limit precisely because their stall
        // cutoffs only fire when current can peg at that limit (SLIDER_STALL_CURRENT_AMPS = 12
        // against 15 A; HOOD_STALL_CURRENT_AMPS = 15 against 20 A). No tier may erode that.
        for (int tier = 0; tier < 3; tier++) {
            double slider = BatteryGovernor.supplyFor(
                15, BatteryGovernor.scaleFor(tier, BatteryGovernor.TIER_FEED, false, false), 15);
            double hood = BatteryGovernor.supplyFor(
                20, BatteryGovernor.scaleFor(tier, BatteryGovernor.TIER_FEED, true, false), 20);
            assertEquals(15.0, slider, "slider dropped below its 12 A stall threshold at tier " + tier);
            assertEquals(20.0, hood, "hood dropped below its 15 A stall threshold at tier " + tier);
        }
    }
}
