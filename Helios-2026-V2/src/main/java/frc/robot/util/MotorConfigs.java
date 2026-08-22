package frc.robot.util;

import java.util.function.Consumer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Central motor-setup factory (pattern borrowed from Team 254's shared motor helpers).
 *
 * <p>Every method REQUIRES a current limit and a neutral/idle mode as arguments, so a motor
 * physically cannot be configured through here without them. This turns the project's
 * "every motor gets a current limit + neutral mode from line one" rule (see CLAUDE.md /
 * Hardware-Data-Sheet sec.7) into a call-site guarantee instead of a per-subsystem
 * convention -- the exact class of omission behind this team's brownout/burnout history.
 *
 * <p>These helpers only CENTRALIZE the config calls the subsystems already made; they change
 * no values. Every limit/neutral/ramp/reset-mode passed by a caller is identical to what that
 * subsystem applied inline before. Swerve motors are configured by Phoenix Tuner X
 * (TunerConstants), not here.
 */
public final class MotorConfigs {
    private MotorConfigs() {}

    // {stator, supply} last handed to configureTalonFX, per motor. Exists so anything that
    // re-applies a limit at runtime (BatteryGovernor) scales the value the SUBSYSTEM chose
    // instead of a copy that can silently drift out of date when that subsystem is retuned.
    private static final java.util.Map<TalonFX, double[]> APPLIED_TALON_LIMITS = new java.util.HashMap<>();

    // Same idea for SparkMaxes: the smart limit last applied, plus a closure that can REPLAY
    // that motor's complete original config with the limit changed.
    private static final java.util.Map<SparkMax, Integer> APPLIED_SPARK_LIMIT = new java.util.HashMap<>();
    private static final java.util.Map<SparkMax, SparkLimitRescaler> SPARK_RESCALERS = new java.util.HashMap<>();

    /** Re-applies a SparkMax's original config with its smart current limit changed. */
    @FunctionalInterface
    public interface SparkLimitRescaler {
        /** @param primaryAmps new limit (the STALL tier for a tiered motor; the free tier scales with it) */
        REVLibError rescale(int primaryAmps);
    }

    /** Limits last applied by {@link #configureTalonFX}, as {stator A, supply A}; null if never. */
    public static double[] appliedLimits(TalonFX motor) {
        return APPLIED_TALON_LIMITS.get(motor);
    }

    /** Smart current limit last applied by the SparkMax helpers (stall tier if tiered); null if never. */
    public static Integer appliedSmartLimit(SparkMax spark) {
        return APPLIED_SPARK_LIMIT.get(spark);
    }

    /**
     * A rescaler for a SparkMax configured through this class, or null if it never was.
     *
     * <p>It replays the motor's ENTIRE original recipe -- idle mode, ramp, and the caller's
     * {@code extra} customizer (the hopper belts' disableFollowerMode + inverted, the 2026-07-18
     * dead-belts fix) -- with only the limit changed, so nothing can be dropped by a partial
     * config. It always uses kNoResetSafeParameters (never factory-resets a live motor mid-match)
     * and kNoPersistParameters (a runtime limit must never be written to flash), and it uses
     * configureAsync so a shed cannot block the robot loop on a CAN round-trip.
     */
    public static SparkLimitRescaler rescaler(SparkMax spark) {
        return SPARK_RESCALERS.get(spark);
    }

    /**
     * Configure a TalonFX with stator + supply current limits (both enabled), a neutral mode,
     * and an optional open-loop duty-cycle ramp -- applied as separate config groups in the
     * same order the subsystems used inline. A ramp of 0 (or less) skips the ramp config.
     *
     * @param motor          the TalonFX to configure
     * @param statorAmps     stator current limit (A) -- torque/heat cap; enabled
     * @param supplyAmps     supply current limit (A) -- battery-draw/brownout cap; enabled
     * @param neutral        Brake or Coast
     * @param openLoopRampSec duty-cycle open-loop ramp period (s); <= 0 to skip
     * @return the first non-OK StatusCode, or {@code StatusCode.OK} if every apply() succeeded.
     *         CHECK IT. An apply() that fails (CAN timeout = controller unpowered or off the bus)
     *         leaves that motor with NO current limit and the vendor-default Coast neutral mode --
     *         silently, which is the exact failure this class exists to prevent. The SparkMax
     *         helpers below have always returned their REVLibError; this one used to discard the
     *         StatusCode, so a flywheel Kraken could come up unlimited with nothing said.
     */
    public static StatusCode configureTalonFX(TalonFX motor, double statorAmps, double supplyAmps,
                                              NeutralModeValue neutral, double openLoopRampSec) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = statorAmps;
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = supplyAmps;
        limits.SupplyCurrentLimitEnable = true;
        StatusCode result = motor.getConfigurator().apply(limits);
        APPLIED_TALON_LIMITS.put(motor, new double[] { statorAmps, supplyAmps });

        MotorOutputConfigs output = new MotorOutputConfigs();
        output.NeutralMode = neutral;
        StatusCode outputResult = motor.getConfigurator().apply(output);
        if (result.isOK()) {
            result = outputResult;
        }

        if (openLoopRampSec > 0) {
            OpenLoopRampsConfigs ramp = new OpenLoopRampsConfigs();
            ramp.DutyCycleOpenLoopRampPeriod = openLoopRampSec;
            StatusCode rampResult = motor.getConfigurator().apply(ramp);
            if (result.isOK()) {
                result = rampResult;
            }
        }
        return result;
    }

    /**
     * Configure a SparkMax with a single-tier smart current limit, an idle mode, and an
     * optional open-loop ramp, then apply with the given reset mode (persisting parameters).
     * The optional {@code extra} customizer runs on the config before it is applied -- for the
     * specialized bits a plain motor doesn't need (e.g. disableFollowerMode + inverted on the
     * hopper belts). Returns the REVLibError so callers can surface a failed CAN/power config.
     *
     * @param spark          the SparkMax to configure
     * @param smartLimitAmps smart current limit (A)
     * @param idle           kBrake or kCoast
     * @param openLoopRampSec open-loop ramp rate (s); <= 0 to skip
     * @param resetMode      kResetSafeParameters (factory reset first) or kNoResetSafeParameters
     * @param extra          extra config mutations applied before configure(); may be null
     */
    public static REVLibError configureSparkMax(SparkMax spark, int smartLimitAmps, IdleMode idle,
                                                double openLoopRampSec, ResetMode resetMode,
                                                Consumer<SparkMaxConfig> extra) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(smartLimitAmps);
        config.idleMode(idle);
        if (openLoopRampSec > 0) {
            config.openLoopRampRate(openLoopRampSec);
        }
        if (extra != null) {
            extra.accept(config);
        }
        APPLIED_SPARK_LIMIT.put(spark, smartLimitAmps);
        SPARK_RESCALERS.put(spark, amps ->
            reapply(spark, c -> c.smartCurrentLimit(amps), idle, openLoopRampSec, extra));
        return spark.configure(config, resetMode, PersistMode.kPersistParameters);
    }

    /** Shared body of the two rescalers -- see {@link #rescaler} for why the modes are fixed. */
    private static REVLibError reapply(SparkMax spark, Consumer<SparkMaxConfig> limit, IdleMode idle,
                                       double openLoopRampSec, Consumer<SparkMaxConfig> extra) {
        SparkMaxConfig config = new SparkMaxConfig();
        limit.accept(config);
        config.idleMode(idle);
        if (openLoopRampSec > 0) {
            config.openLoopRampRate(openLoopRampSec);
        }
        if (extra != null) {
            extra.accept(config);
        }
        return spark.configureAsync(config, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    /**
     * Same as {@link #configureSparkMax} but with a TIERED smart current limit
     * (stall limit ramping to a free-running limit above a motor-rpm threshold) -- e.g. the
     * intake slider's 15 A@stall / 20 A@>1000 rpm pulley-snap guard.
     *
     * @param stallLimitAmps limit at/near 0 rpm (A)
     * @param freeLimitAmps  limit above {@code limitRpm} (A)
     * @param limitRpm       motor-rpm threshold between the two tiers
     */
    public static REVLibError configureSparkMaxTiered(SparkMax spark, int stallLimitAmps,
                                                      int freeLimitAmps, int limitRpm, IdleMode idle,
                                                      double openLoopRampSec, ResetMode resetMode,
                                                      Consumer<SparkMaxConfig> extra) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(stallLimitAmps, freeLimitAmps, limitRpm);
        config.idleMode(idle);
        if (openLoopRampSec > 0) {
            config.openLoopRampRate(openLoopRampSec);
        }
        if (extra != null) {
            extra.accept(config);
        }
        APPLIED_SPARK_LIMIT.put(spark, stallLimitAmps);
        SPARK_RESCALERS.put(spark, amps ->
            // Keep the two tiers in proportion: scaling only the stall tier would let the free
            // tier sit ABOVE it and quietly undo the shed as soon as the motor spins up.
            reapply(spark, c -> c.smartCurrentLimit(amps,
                    Math.max(amps, (int) Math.round(freeLimitAmps * (double) amps / stallLimitAmps)),
                    limitRpm),
                idle, openLoopRampSec, extra));
        return spark.configure(config, resetMode, PersistMode.kPersistParameters);
    }
}
