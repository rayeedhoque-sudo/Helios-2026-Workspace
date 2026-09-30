package frc.robot.subsystems.misc;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.LauncherConstants;

/**
 * V2 LAUNCHER (drum shooter, 2026-09-30): 3 flywheel Kraken X60s (15 leader, 14 + 13
 * followers), a Kraken X60 feeder (CAN 24) and a Kraken X44 hood (CAN 18).
 *
 * The hood has NO absolute encoder -- its angle comes from the X44's internal encoder, zeroed
 * by homing into the bottom hard stop on every enable. Angle = degrees above that stop.
 * The hood is driven from periodic() (not a command) so a shot can never interrupt homing,
 * and the DPAD hood steps need no subsystem requirement.
 */
public class LauncherSubsystem extends SubsystemBase {

    /** Hood state. Only HOMED accepts DPAD moves; FAILED/FAULT hold neutral until the next enable. */
    public enum HoodState { WAITING_FOR_ENABLE, HOMING, HOMED, HOMING_FAILED, STALL_FAULT }

    //Motors
        private final TalonFX flywheelLeader = new TalonFX(LauncherConstants.FLYWHEEL_LEADER_ID);
        private final TalonFX flywheelBelted = new TalonFX(LauncherConstants.FLYWHEEL_BELTED_ID);
        private final TalonFX flywheelRight = new TalonFX(LauncherConstants.FLYWHEEL_RIGHT_ID);
        private final TalonFX feeder = new TalonFX(LauncherConstants.FEEDER_ID);
        private final TalonFX hood = new TalonFX(LauncherConstants.HOOD_ID);

    //Control requests (reused every loop)
        private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0);
        private final VelocityVoltage feederVelocity = new VelocityVoltage(0);
        // "Off" = coast down freely, never VelocityVoltage(0), which would brake the wheels.
        private final CoastOut coast = new CoastOut();
        private final MotionMagicVoltage hoodMotion = new MotionMagicVoltage(0);
        // Homing runs below the soft limits (they are relative to an unknown zero until homed).
        private final VoltageOut hoodHoming = new VoltageOut(0).withIgnoreSoftwareLimits(true);
        private final NeutralOut hoodNeutral = new NeutralOut(); // Brake neutral holds the hood

    //State
        private double flywheelTargetRpm = LauncherConstants.FLYWHEEL_START_RPM;
        private boolean shooting = false;
        private boolean flywheelAtSpeed = false;
        private boolean feederAtSpeed = false;
        private HoodState hoodState = HoodState.WAITING_FOR_ENABLE;
        private double hoodTargetDeg = 0;
        private boolean wasEnabled = false;
        private final Timer homingTimer = new Timer();
        private Debouncer homingStopDebouncer;
        private Debouncer hoodStallDebouncer;

    //Data -- the tab name and the entry names the driver companion's shooter panel already
    // reads are kept, so it keeps working (angles are now degrees above the homed stop).
        private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter Subsystem Tab");
        private final GenericEntry targetRpmEntry = tab.add("Desired Velocity", 0.0).getEntry();
        private final GenericEntry rpmEntry = tab.add("Current Velocity", 0.0).getEntry();
        private final GenericEntry atSpeedEntry = tab.add("Desired Velocity Reached", false).getEntry();
        private final GenericEntry shootingEntry = tab.add("Subsystem State", false).getEntry();
        private final GenericEntry motorRpmEntry = tab.add("Current Flywheel (motor RPM)", 0.0).getEntry();
        private final GenericEntry motorTargetRpmEntry = tab.add("RT Target (motor RPM)", 0.0).getEntry();
        private final GenericEntry feederRpmEntry = tab.add("Feeder (motor RPM)", 0.0).getEntry();
        private final GenericEntry feederAtSpeedEntry = tab.add("Feeder At Speed", false).getEntry();
        private final GenericEntry readyToFeedEntry = tab.add("Ready To Feed (belts on)", false).getEntry();
        private final GenericEntry hoodStateEntry = tab.add("Hood State", hoodState.name()).getEntry();
        private final GenericEntry hoodAngleEntry = tab.add("Current Angle", 0.0).getEntry();
        private final GenericEntry hoodTargetEntry = tab.add("Desired Angle", 0.0).getEntry();
        private final GenericEntry hoodReachedEntry = tab.add("Desired Angle Reached", false).getEntry();
        private final GenericEntry hoodAmpsEntry = tab.add("Hood Current (A)", 0.0).getEntry();
        private final GenericEntry hoodVoltsEntry = tab.add("Hood Volts (cmd)", 0.0).getEntry();
        private final GenericEntry hoodRawEntry = tab.add("Hood Encoder Raw (rot)", 0.0).getEntry();

    public LauncherSubsystem(){
        // Flywheels: V1's team-tuned 50 A stator / 25 A supply on EACH motor (followers enforce
        // their own configs), Coast so they spin down freely.
            CurrentLimitsConfigs flywheelLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(LauncherConstants.FLYWHEEL_STATOR_A).withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(LauncherConstants.FLYWHEEL_SUPPLY_A).withSupplyCurrentLimitEnable(true);
            TalonFXConfiguration leaderConfig = new TalonFXConfiguration()
                .withCurrentLimits(flywheelLimits)
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(LauncherConstants.FLYWHEEL_LEADER_INVERTED
                        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs()
                    .withKP(LauncherConstants.FLYWHEEL_kP)
                    .withKD(LauncherConstants.FLYWHEEL_kD)
                    .withKV(LauncherConstants.FLYWHEEL_kV));
            flywheelLeader.getConfigurator().apply(leaderConfig);
            TalonFXConfiguration followerConfig = new TalonFXConfiguration()
                .withCurrentLimits(flywheelLimits)
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            flywheelBelted.getConfigurator().apply(followerConfig);
            flywheelRight.getConfigurator().apply(followerConfig);
            flywheelBelted.setControl(new Follower(flywheelLeader.getDeviceID(),
                LauncherConstants.FLYWHEEL_BELTED_OPPOSED ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
            flywheelRight.setControl(new Follower(flywheelLeader.getDeviceID(),
                LauncherConstants.FLYWHEEL_RIGHT_OPPOSED ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
            flywheelLeader.setControl(coast);

        // Feeder: 30 A stator / 20 A supply (team-approved), Coast, 0.25 s closed-loop ramp to
        // soften spin-up inrush. A jam drops it out of the at-speed band, which stops the belts.
            feeder.getConfigurator().apply(new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(LauncherConstants.FEEDER_STATOR_A).withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(LauncherConstants.FEEDER_SUPPLY_A).withSupplyCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(LauncherConstants.FEEDER_INVERTED
                        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs().withKP(LauncherConstants.FEEDER_kP).withKV(LauncherConstants.FEEDER_kV))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(LauncherConstants.FEEDER_RAMP_SEC)));
            feeder.setControl(coast);

        // Hood: 20 A stator / 15 A supply (team-approved), Brake to hold. Positions are in HOOD
        // rotations (SensorToMechanismRatio = 42.5). Soft limits 0..HOOD_MAX_DEG stay on
        // permanently; homing uses a request that ignores them.
            hood.getConfigurator().apply(new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(LauncherConstants.HOOD_STATOR_A).withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(LauncherConstants.HOOD_SUPPLY_A).withSupplyCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(LauncherConstants.HOOD_INVERTED
                        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive))
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(LauncherConstants.HOOD_MOTOR_ROT_PER_HOOD_ROT))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(LauncherConstants.HOOD_MAX_DEG / 360.0)
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(0)
                    .withReverseSoftLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(LauncherConstants.HOOD_CRUISE_RPS)
                    .withMotionMagicAcceleration(LauncherConstants.HOOD_ACCEL_RPS2))
                .withSlot0(new Slot0Configs().withKP(LauncherConstants.HOOD_kP).withKV(LauncherConstants.HOOD_kV)));
            hood.setControl(hoodNeutral);
    }

    //Pure helpers (package-private for LauncherLogicTest)
        static double clampFlywheelRpm(double rpm){
            return MathUtil.clamp(rpm, 0, LauncherConstants.FLYWHEEL_MAX_RPM);
        }

        static double clampHoodDeg(double deg){
            return MathUtil.clamp(deg, 0, LauncherConstants.HOOD_MAX_DEG);
        }

        static double flywheelRpmToMotorRps(double flywheelRpm){
            return flywheelRpm / 60.0 * LauncherConstants.FLYWHEEL_MOTOR_ROT_PER_FLYWHEEL_ROT;
        }

        static double motorRpsToFlywheelRpm(double motorRps){
            return motorRps * 60.0 / LauncherConstants.FLYWHEEL_MOTOR_ROT_PER_FLYWHEEL_ROT;
        }

        /** True only for a nonzero target with the measurement inside the band. */
        static boolean atSpeed(double measured, double target, double tolerance){
            return target != 0 && Math.abs(measured - target) <= tolerance;
        }

        /**
         * Homing stop test (before debounce): only after the grace window, so the startup inrush
         * of a stationary motor can't read as "hit the bottom stop" before the hood has moved --
         * a false home would put 0 deg mid-travel and HOOD_MAX_DEG past the real top stop.
         */
        static boolean homingSawStop(double elapsedSec, double statorAmps){
            return elapsedSec >= LauncherConstants.HOOD_HOMING_GRACE_SEC
                && statorAmps >= LauncherConstants.HOOD_HOMING_STOP_AMPS;
        }

    //Getters
        public boolean isShooting(){ return shooting; }
        /** Belt gate: flywheels AND feeder at speed (team spec 2026-09-30). */
        public boolean isReadyToFeed(){ return flywheelAtSpeed && feederAtSpeed; }
        public double getFlywheelTargetRpm(){ return flywheelTargetRpm; }
        public HoodState getHoodState(){ return hoodState; }

        private double getFlywheelRpm(){
            return motorRpsToFlywheelRpm(flywheelLeader.getVelocity().getValueAsDouble());
        }

        private double getHoodDeg(){
            return hood.getPosition().getValueAsDouble() * 360.0;
        }

    //Commands
        /**
         * RT (hold): flywheels at the DPAD-set rpm and the feeder at its fixed speed, re-read
         * every loop so a DPAD change mid-hold applies at once. Hood is NOT commanded.
         * Release: both coast down.
         */
        public Command shootCommand(){
            return runEnd(
                () -> {
                    shooting = true;
                    flywheelLeader.setControl(flywheelVelocity.withVelocity(
                        flywheelRpmToMotorRps(flywheelTargetRpm)));
                    feeder.setControl(feederVelocity.withVelocity(
                        LauncherConstants.FEEDER_TARGET_MOTOR_RPM / 60.0));
                },
                () -> {
                    shooting = false;
                    flywheelLeader.setControl(coast);
                    feeder.setControl(coast);
                });
        }

        /** Feeder backward at a low duty (VIEW unjam, with the hopper belts). */
        public Command feederUnjamCommand(){
            return runEnd(
                () -> feeder.set(-LauncherConstants.FEEDER_UNJAM_DUTY),
                () -> feeder.setControl(coast));
        }

        /** DPAD left/right: flywheel target -/+ one step. No requirement, so it works mid-shot. */
        public Command stepFlywheelRpmCommand(double deltaRpm){
            return Commands.runOnce(() -> flywheelTargetRpm = clampFlywheelRpm(flywheelTargetRpm + deltaRpm));
        }

        /** DPAD up/down: hood target +/- one step. Ignored unless homed. No requirement. */
        public Command stepHoodCommand(double deltaDeg){
            return Commands.runOnce(() -> {
                if (hoodState == HoodState.HOMED) {
                    hoodTargetDeg = clampHoodDeg(hoodTargetDeg + deltaDeg);
                }
            });
        }

    @Override
    public void periodic(){
        //Hood -- re-home on every disabled -> enabled edge
            boolean enabled = DriverStation.isEnabled();
            if (enabled && !wasEnabled) {
                hoodState = HoodState.HOMING;
                homingTimer.restart();
                homingStopDebouncer = new Debouncer(LauncherConstants.HOOD_HOMING_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
                hoodStallDebouncer = new Debouncer(LauncherConstants.HOOD_STALL_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
            }
            wasEnabled = enabled;

            double hoodAmps = hood.getStatorCurrent().getValueAsDouble();
            double hoodDeg = getHoodDeg();
            switch (hoodState) {
                case HOMING:
                    if (homingStopDebouncer.calculate(homingSawStop(homingTimer.get(), hoodAmps))) {
                        hood.setPosition(0);
                        hoodTargetDeg = 0;
                        hoodState = HoodState.HOMED;
                        hood.setControl(hoodNeutral);
                    } else if (homingTimer.hasElapsed(LauncherConstants.HOOD_HOMING_TIMEOUT_SEC)) {
                        hoodState = HoodState.HOMING_FAILED;
                        hood.setControl(hoodNeutral);
                        DriverStation.reportWarning("Hood homing timed out -- hood locked until next enable", false);
                    } else {
                        hood.setControl(hoodHoming.withOutput(-LauncherConstants.HOOD_HOMING_VOLTS));
                    }
                    break;
                case HOMED:
                    boolean awayFromTarget = Math.abs(hoodDeg - hoodTargetDeg) > LauncherConstants.HOOD_STALL_MIN_ERROR_DEG;
                    if (hoodStallDebouncer.calculate(awayFromTarget && hoodAmps >= LauncherConstants.HOOD_STALL_AMPS)) {
                        hoodState = HoodState.STALL_FAULT;
                        hood.setControl(hoodNeutral);
                        DriverStation.reportWarning("Hood stalled short of its target -- hood locked until next enable", false);
                    } else {
                        hood.setControl(hoodMotion.withPosition(hoodTargetDeg / 360.0));
                    }
                    break;
                default:
                    hood.setControl(hoodNeutral);
                    break;
            }

        //At-speed gates (only meaningful while shooting)
            double flywheelRpm = getFlywheelRpm();
            double feederRpm = feeder.getVelocity().getValueAsDouble() * 60.0;
            flywheelAtSpeed = shooting && atSpeed(flywheelRpm, flywheelTargetRpm,
                LauncherConstants.FLYWHEEL_AT_SPEED_TOL_RPM);
            feederAtSpeed = shooting && atSpeed(feederRpm, LauncherConstants.FEEDER_TARGET_MOTOR_RPM,
                LauncherConstants.FEEDER_TARGET_MOTOR_RPM * LauncherConstants.FEEDER_AT_SPEED_FRACTION);

        //Data
            targetRpmEntry.setDouble(flywheelTargetRpm);
            rpmEntry.setDouble(flywheelRpm);
            atSpeedEntry.setBoolean(flywheelAtSpeed);
            shootingEntry.setBoolean(shooting);
            motorRpmEntry.setDouble(flywheelLeader.getVelocity().getValueAsDouble() * 60.0);
            motorTargetRpmEntry.setDouble(flywheelRpmToMotorRps(flywheelTargetRpm) * 60.0);
            feederRpmEntry.setDouble(feederRpm);
            feederAtSpeedEntry.setBoolean(feederAtSpeed);
            readyToFeedEntry.setBoolean(isReadyToFeed());
            hoodStateEntry.setString(hoodState.name());
            hoodAngleEntry.setDouble(hoodDeg);
            hoodTargetEntry.setDouble(hoodTargetDeg);
            hoodReachedEntry.setBoolean(hoodState == HoodState.HOMED
                && Math.abs(hoodDeg - hoodTargetDeg) <= LauncherConstants.HOOD_STALL_MIN_ERROR_DEG);
            hoodAmpsEntry.setDouble(hoodAmps);
            hoodVoltsEntry.setDouble(hood.getMotorVoltage().getValueAsDouble());
            hoodRawEntry.setDouble(hood.getRotorPosition().getValueAsDouble());
    }

    // Read-only motor access for PowerTelemetry (no control). Order: 15, 14, 13.
    public TalonFX[] getFlywheelMotors(){
        return new TalonFX[] { flywheelLeader, flywheelBelted, flywheelRight };
    }

    public TalonFX getFeederMotor(){ return feeder; }

    public TalonFX getHoodMotor(){ return hood; }
}
