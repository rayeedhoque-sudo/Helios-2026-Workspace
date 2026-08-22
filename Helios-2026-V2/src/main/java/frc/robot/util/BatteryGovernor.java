package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Battery-voltage-adaptive SUPPLY current limits (the "variable current limit" load-shed
 * supervisor sketched in {@code Helios-2026\docs\power-limiting-review.md} and
 * {@code docs\power-review-2026-07-25.md} sec.4).
 *
 * <p>Watches the bus voltage every loop, walks a tier ladder with hysteresis + a dwell time, and
 * on a TIER CHANGE ONLY re-applies each governed motor's supply limit scaled down by that tier.
 * A subsystem-style {@code periodic()} (same auto-run-via-scheduler pattern as
 * {@link LimelightThermalManager}) — no bindings, no commands, no setpoints touched.
 *
 * <h2>SHIPS DISABLED — read before flipping {@link #ENABLED}</h2>
 * The 2026-07-25 power review's conclusion still stands: the cause of the late-match brownouts is
 * NOT identified. The two live hypotheses are a hot SB50/lug/main-breaker (rising path resistance)
 * and the unlimited kicker CIM — and a governor that quietly holds the rail up would MASK either
 * one rather than fix it. Measure first (PowerDistribution total current + the V-vs-I resistance
 * slope + a brownout counter), then enable this. Until then it runs the tier machine and publishes
 * what it WOULD have done to {@code BatteryGovernor/*}, and applies nothing.
 *
 * <h2>THE SHOOTER ALWAYS WINS (team requirement 2026-07-25)</h2>
 * Scoring is never traded for headroom. Three independent mechanisms enforce it:
 * <ol>
 *   <li><b>Shed order.</b> Every motor carries a {@code firstSheddingTier}. Feed and acquisition
 *       (intake roller, hopper belts) shed at tier 1; the flywheels not until tier 2. Tier 1
 *       therefore leaves the shot bit-for-bit untouched.</li>
 *   <li><b>Shot interlock.</b> Motors registered {@code shotCritical} are forced back to their
 *       FULL configured limits while a shot is live, whatever the tier — restored when the shot
 *       STARTS, not when it ends. This is affordable precisely because the shot freezes the
 *       drivetrain ({@code SwerveRequest.Idle}), so the bus is near-idle exactly then. The signal
 *       covers both shot buttons: RB commands flywheel velocity immediately, and RT — which
 *       deliberately withholds velocity until the aim is made — is covered by its shot target
 *       for the whole aim phase.</li>
 *   <li><b>Per-motor floors.</b> A shed can never take a motor below the draw it needs to do its
 *       job — see the floors at the registration call in {@code RobotContainer}.</li>
 * </ol>
 * The hood is shot-critical and floored at its full limit, so hood ANGLE — the accuracy term — is
 * never affected at all. On the flywheels a shed can only lengthen spin-up and between-ball
 * recovery, never weaken a shot: the ball-strike impulse comes from the STATOR limit (unscaled)
 * plus wheel inertia, and the kicker is already gated on {@code isFlywheelAtSpeed}, so a slower
 * recovery makes the kicker WAIT rather than fire into a sagging wheel. The cost of a tier-2 shed
 * is fire rate, not accuracy.
 *
 * <h2>What is governed, and what is deliberately NOT</h2>
 * Governed: every mechanism motor that has a software current limit — 4 flywheel Krakens, the
 * intake roller Kraken, both hopper belt NEOs, the intake slider NEO and the hood NEO 550.
 * <ul>
 *   <li><b>Swerve drive/steer:</b> excluded, and this is the one exclusion that is NOT a floor.
 *       Their limits are Tuner X's, and the drive motors use the supply LOWER-limit fold-back
 *       fields; re-applying a {@code CurrentLimitsConfigs} here would blank those. Throttle drive
 *       with {@code MaxSpeed}/{@code MaxAngularRate} instead — same tier machine, zero CAN
 *       traffic. The 40/20 steer starve regression (2026-07-08) is why steer stays untouched.</li>
 *   <li><b>Kicker (Victor SPX):</b> no current sensing at all — cannot be governed in software.
 *       Its breaker is still the only protection (open TODO).</li>
 * </ul>
 *
 * <p><b>Two motors are registered with a floor equal to their base limit, i.e. present in the
 * table but currently unable to shed.</b> That is deliberate, not an oversight — each has a
 * measured stall-detection threshold wired to its current limit:
 * <ul>
 *   <li><b>Intake slider:</b> {@code SLIDER_STALL_CURRENT_AMPS = 12} means the stall cutoff only
 *       fires because current can peg at the 15 A limit. Shed below 12 A and the cutoff can NEVER
 *       fire — every slider move would instead grind to its 2.5 s timeout, which is the pulley-snap
 *       failure mode the tiered limit exists to prevent. Its breakaway current is also still
 *       unverified on robot ("TODO verify 15 A still breaks the slider away from rest").</li>
 *   <li><b>Hood:</b> {@code HOOD_STALL_CURRENT_AMPS = 15} against a 20 A limit, same coupling —
 *       and it is shot-critical for accuracy besides. A settled hood draws ~0 A anyway.</li>
 * </ul>
 * Lower either floor only together with that mechanism's stall threshold, on the robot.
 *
 * <h2>Why only the SUPPLY limit is scaled</h2>
 * Supply current is the battery-draw knob; stator is torque/heat and is re-sent UNCHANGED. Scaling
 * stator would undo the 2026-07-17 flywheel 40 -> 80 A anti-bog raise, which is a shot-quality fix,
 * not a power one. Phoenix applies a config GROUP wholesale, so every field of
 * {@code CurrentLimitsConfigs} is re-sent each time — the base values come from
 * {@link MotorConfigs#appliedLimits} so they can never drift from what the subsystem configured.
 */
public class BatteryGovernor extends SubsystemBase {

    /**
     * MASTER GATE. false = observe/publish only, no motor is ever written to.
     * TODO enable only after the brownout cause is measured (see the class javadoc) AND the
     * thresholds below are tuned on the robot.
     */
    public static final boolean ENABLED = false;

    /**
     * One shed tier. {@code tripVolts} = bus voltage at/below which we drop INTO the next tier;
     * {@code recoverVolts} = voltage at/above which we climb OUT of this one (hysteresis);
     * {@code supplyScale} = multiplier applied to every governed motor's supply limit.
     */
    private record Tier(double tripVolts, double recoverVolts, double supplyScale) {}

    // TODO tune all six numbers on the robot. Starting points from the V1 power review:
    // trip ~7.0 V under load, recover ~7.6 V. The RIO2 hardware brownout floor is 6.75 V, so
    // tier 2 is the last stop before the FPGA takes over — this cannot catch a tens-of-ms
    // transient, only a sustained sag. The +inf/-inf ends make the ladder self-bounding.
    private static final Tier[] TIERS = {
        new Tier(7.6, Double.POSITIVE_INFINITY, 1.00), // 0 NORMAL   -- configured limits, never recovers upward
        new Tier(7.0, 8.0,                      0.75), // 1 REDUCED  -- feed/acquisition only; the shot is untouched
        new Tier(Double.NEGATIVE_INFINITY, 7.6, 0.50), // 2 CRITICAL -- flywheels join in, never trips further
    };

    /** Tier at which feed/acquisition loads start shedding. */
    public static final int TIER_FEED = 1;
    /** Tier at which the flywheels start shedding -- the LAST thing to give up authority. */
    public static final int TIER_SHOT = 2;

    /** How long the trip/recover condition must hold before the tier actually moves (s). */
    static final double DWELL_SEC = 0.2;

    /** Applies one motor's limits for a given tier and shot state. */
    private interface Applier {
        void apply(int tier, boolean shotActive);
    }

    private final List<Applier> appliers = new ArrayList<>();
    private final Timer dwell = new Timer();
    // "A shot is live." Motors registered shot-critical ignore the tier entirely while this holds.
    private BooleanSupplier shotActive = () -> false;
    private int tier = 0;
    // Round-robin cursor: ONE motor is visited per loop, so a shed costs at most one CAN config
    // write per 20 ms rather than nine at once, and a full sweep lands within ~180 ms. The sweep
    // is CONTINUOUS and each applier no-ops unless its own commanded amps actually changed
    // (see register), which is what makes this correct rather than edge-triggered: a tier move or
    // a shot edge part-way through a sweep cannot leave the motors in a mixed configuration,
    // because every motor is revisited unconditionally and reconciles itself against the CURRENT
    // state. Nothing needs to remember that an edge happened.
    private int cursor = 0;

    private final DoublePublisher tierPub;
    private final DoublePublisher scalePub;
    private final DoublePublisher voltsPub;
    private final BooleanPublisher shotProtectedPub;

    public BatteryGovernor() {
        var table = NetworkTableInstance.getDefault().getTable("BatteryGovernor");
        tierPub = table.getDoubleTopic("tier").publish();
        scalePub = table.getDoubleTopic("supplyScale").publish();
        voltsPub = table.getDoubleTopic("busVolts").publish();
        shotProtectedPub = table.getBooleanTopic("shotProtected").publish();
        dwell.start();
    }

    /**
     * Supply the "a shot is live" signal. While it reads true, every shot-critical motor is held
     * at its FULL configured limits regardless of tier — restored when the shot STARTS, not when
     * it ends. Restoration lands within one sweep of the cursor (~180 ms at 9 motors / 50 Hz),
     * which is a small fraction of flywheel spin-up. Chainable; call once.
     */
    public BatteryGovernor protectShotWhile(BooleanSupplier live) {
        shotActive = live;
        return this;
    }

    /**
     * Put TalonFXs under the governor. Base limits are read back from {@link MotorConfigs} — a
     * motor that was not configured through that factory is REFUSED loudly rather than governed
     * with guessed limits. Chainable.
     *
     * @param floorAmps the limit this motor is never scaled below, whatever the tier. A pure
     *     multiplier is not safe on its own: every mechanism here has a draw floor under which it
     *     stops doing its job (a flywheel that cannot re-accelerate between balls is the failure
     *     the 2026-07-17 anti-bog work existed to cure) -- and on the slider and hood the floor
     *     also keeps the stall cutoffs alive. Size it from measured draw, not from the tier table.
     * @param firstSheddingTier the first tier at which this motor gives anything up
     *     ({@link #TIER_FEED} or {@link #TIER_SHOT}); it runs at full limits below that.
     * @param shotCritical true = held at full limits for the whole duration of a live shot.
     */
    public BatteryGovernor register(double floorAmps, int firstSheddingTier, boolean shotCritical,
                                    TalonFX... motors) {
        for (TalonFX motor : motors) {
            double[] base = MotorConfigs.appliedLimits(motor);
            if (base == null) {
                DriverStation.reportError("BatteryGovernor: CAN " + motor.getDeviceID()
                    + " was not configured through MotorConfigs -- NOT governed.", false);
                continue;
            }
            final double stator = base[0];
            final double supply = base[1];
            // Last value actually written to this controller; NaN = never, so the first sweep
            // always writes. Comparing against it makes each visit idempotent -- see cursor.
            final double[] lastWritten = { Double.NaN };
            appliers.add((tier, shot) -> {
                double amps = supplyFor(supply, scaleFor(tier, firstSheddingTier, shotCritical, shot),
                    floorAmps);
                if (amps == lastWritten[0]) {
                    return;   // nothing changed for this motor -- no CAN traffic
                }
                lastWritten[0] = amps;
                CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
                limits.StatorCurrentLimit = stator;          // re-sent UNCHANGED (see class javadoc)
                limits.StatorCurrentLimitEnable = true;
                limits.SupplyCurrentLimit = amps;   // the only field the tier moves
                limits.SupplyCurrentLimitEnable = true;
                // 0 s timeout = fire-and-forget, so a dead controller can't stall the loop waiting
                // for a CAN response. Nothing to check: with no wait there is no status to report.
                motor.getConfigurator().apply(limits, 0.0);
            });
        }
        return this;
    }

    /**
     * Same, for SparkMaxes. The smart current limit is scaled; the motor's ENTIRE original config
     * (idle mode, ramp, follower-disable, inversion) is replayed with it, never factory-reset and
     * never written to flash — see {@link MotorConfigs#rescaler}.
     */
    public BatteryGovernor register(double floorAmps, int firstSheddingTier, boolean shotCritical,
                                    SparkMax... motors) {
        for (SparkMax motor : motors) {
            Integer base = MotorConfigs.appliedSmartLimit(motor);
            MotorConfigs.SparkLimitRescaler rescaler = MotorConfigs.rescaler(motor);
            if (base == null || rescaler == null) {
                DriverStation.reportError("BatteryGovernor: CAN " + motor.getDeviceId()
                    + " was not configured through MotorConfigs -- NOT governed.", false);
                continue;
            }
            final double baseAmps = base;
            final int[] lastWritten = { Integer.MIN_VALUE };   // see the TalonFX overload
            appliers.add((tier, shot) -> {
                // ceil, not round: smartCurrentLimit takes an int, and rounding DOWN could push a
                // motor a whole amp under a floor sized to keep its stall cutoff alive.
                int amps = (int) Math.ceil(supplyFor(baseAmps,
                    scaleFor(tier, firstSheddingTier, shotCritical, shot), floorAmps));
                if (amps == lastWritten[0]) {
                    return;
                }
                lastWritten[0] = amps;
                rescaler.rescale(amps);
            });
        }
        return this;
    }

    /** Tier the ladder wants right now, ignoring dwell. Pure. */
    static int nextTier(int tier, double volts) {
        if (volts <= TIERS[tier].tripVolts()) {
            return tier + 1;
        }
        if (volts >= TIERS[tier].recoverVolts()) {
            return tier - 1;
        }
        return tier;
    }

    /**
     * Tier after one governor step. Pure — the dwell clock is the caller's. A move only happens
     * once the wanted tier has been wanted continuously for {@link #DWELL_SEC}, so a single
     * accel/spin-up sag (or one noisy sample) can never cut the shot it is firing.
     */
    static int step(int tier, double volts, double dwellElapsedSec) {
        int wanted = nextTier(tier, volts);
        return (wanted != tier && dwellElapsedSec >= DWELL_SEC) ? wanted : tier;
    }

    /**
     * Supply limit to command: the configured limit scaled by the tier, clamped at the
     * mechanism's floor, and never raised above the configured value. Pure.
     */
    static double supplyFor(double baseAmps, double scale, double floorAmps) {
        return Math.min(baseAmps, Math.max(baseAmps * scale, floorAmps));
    }

    /** The tier's own multiplier, ignoring any per-motor rules. Pure; used for telemetry. */
    static double scaleFor(int tier) {
        return TIERS[tier].supplyScale();
    }

    /**
     * Multiplier for ONE motor: 1.0 (untouched) unless the ladder has reached that motor's own
     * shedding tier, and always 1.0 for a shot-critical motor while a shot is live. Pure — this
     * is the function that makes "the shooter can always hit target" true. See the class javadoc.
     */
    static double scaleFor(int tier, int firstSheddingTier, boolean shotCritical, boolean shotActive) {
        if (shotCritical && shotActive) {
            return 1.0;
        }
        return tier >= firstSheddingTier ? scaleFor(tier) : 1.0;
    }

    @Override
    public void periodic() {
        double volts = RobotController.getBatteryVoltage();
        boolean shot = shotActive.getAsBoolean();

        int next = step(tier, volts, dwell.get());
        if (next != tier) {
            tier = next;
            dwell.restart();
        } else if (nextTier(tier, volts) == tier) {
            dwell.restart(); // condition stopped holding -- the dwell clock starts over
        }

        if (ENABLED && !appliers.isEmpty()) {
            appliers.get(cursor).apply(tier, shot);
            cursor = (cursor + 1) % appliers.size();
        }

        tierPub.set(tier);
        scalePub.set(scaleFor(tier));
        voltsPub.set(volts);
        shotProtectedPub.set(shot);
    }
}
