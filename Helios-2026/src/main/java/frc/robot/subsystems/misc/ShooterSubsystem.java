package frc.robot.subsystems.misc;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.OptionalDouble;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.util.Tunable;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.utility.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase{

    //Type
        public boolean enableComp;
   
    //Shooter Motor
        private final TalonFX shooterA = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_A);
          private final TalonFX shooterB = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_B);
        private final TalonFX shooterC = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_C);
          private final TalonFX shooterD = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_D);
        private final SparkMax shooterAngle = new SparkMax(ShooterSubsystemConstants.SHOOTER_ANGLE_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
            private final SparkAbsoluteEncoder shooterAngleEncoder = shooterAngle.getAbsoluteEncoder();

    // HOOD CLOSED-LOOP ENABLE (kill switch). false = hold the hood at 0 V (brake idle holds
    // position) -- the safe state if the angle feedback ever misbehaves again. Re-enabled 2026-07-18
    // after getShooterAngleDegrees() was recalibrated to the absolute encoder's real two-point map
    // (HOOD_*_AT_FULL_UP / HOOD_*_AT_FULL_DOWN); the old motor-rotation math railed the hood.
    // RE-ENABLED 2026-08-22 after the wrap fix was verified on the robot: a disabled hand
    // sweep traced 3.2 deg (down stop) -> 44.7 deg (up stop) continuously, max 2.03 deg
    // between samples straight through the rollover -- where the old map jumped 191 deg in
    // one sample and drove the hood into the bottom stop. See unwrapHoodRaw() and
    // HoodAngleMapTest. NOTE: the hood PID gains and HOOD_RAISE_FF_VOLTS were tuned against
    // the OLD broken mapping, so they are effectively untuned -- expect overshoot or
    // sluggishness and retune on robot.
    private static final boolean HOOD_CLOSED_LOOP_ENABLED = true;

     //Shooter Speed - PID & FF
        private final VelocityVoltage m_velocity = new VelocityVoltage(0);
        // Coast request for "flywheels off": closing the velocity loop on 0 rps would actively
        // reverse-brake 4 spinning Krakens (regen current dump + gearbox stress) -- the Coast
        // neutral mode only applies when no closed-loop request is latched.
        private final CoastOut m_flywheelCoast = new CoastOut();
        // All six gains are applied, not just kP/kD/kV: kI/kS/kA existed as constants but were
        // never reaching the controller, so the PID panel would have shown three knobs that did
        // nothing. Their defaults are 0, so wiring them up changes no behavior.
        private final Slot0Configs shooterVelConfigs =
            new Slot0Configs().
            withKP(ShooterSubsystemConstants.SHOOTER_SPEED_kP).
            withKI(ShooterSubsystemConstants.SHOOTER_SPEED_kI).
            withKD(ShooterSubsystemConstants.SHOOTER_SPEED_kD).
            withKS(ShooterSubsystemConstants.SHOOTER_SPEED_kS).
            withKV(ShooterSubsystemConstants.SHOOTER_SPEED_kV).
            withKA(ShooterSubsystemConstants.SHOOTER_SPEED_kA);
    //Shooter Angle - PID 
        private final PIDController shooterAnglePID = new PIDController(
            ShooterSubsystemConstants.SHOOTER_ANGLE_kP, 
            ShooterSubsystemConstants.SHOOTER_ANGLE_kI, 
            ShooterSubsystemConstants.SHOOTER_ANGLE_kD
        );
    //Drive Train
        private final CommandSwerveDrivetrain drivetrain;
        private double degreesToAlignToTarget;
    //Data
        private final ShuffleboardTab ShooterSubsystemTab = Shuffleboard.getTab("Shooter Subsystem Tab");
            private final GenericEntry currentVelEntry;
            private final GenericEntry desiredVelEntry;
            private final GenericEntry currentAngleEntry;
            // MEASURED flywheel speed in motor RPM (2026-08-22). Current Velocity is the same
            // reading in m/s of SURFACE speed -- the units the shot model works in -- but the
            // DPAD trims in RPM and RPM is what the team reads off the shooter, so publish both
            // rather than making anyone convert.
            private final GenericEntry currentRpmEntry;
            private final GenericEntry desiredAngleEntry;
            private final GenericEntry targetDistanceEntry;
            private final GenericEntry targetClassEntry;
            private final GenericEntry shotSetpointEntry;
            private final GenericEntry readyToShootEntry;
            private final GenericEntry aimedEntry;
            private final GenericEntry movingCompEntry;
            private final GenericEntry subsystemStateEntry;
            private final GenericEntry degreedToAlignToTargEntry;
            // Gains moved off Shuffleboard onto the driver-companion PID panel (NT /Tuning/*).
            // The old GenericEntry boxes had the robot and the dashboard both writing the same
            // entry every loop, so a typed value survived only if it landed in the gap between
            // the echo and the read -- tuning appeared to work, then silently reverted. Tunable
            // is read-only on the robot side, which removes that race. See frc.robot.util.Tunable.
            private final Tunable tuneSpeedKp = new Tunable("Shooter/Flywheel/kP", ShooterSubsystemConstants.SHOOTER_SPEED_kP);
            private final Tunable tuneSpeedKi = new Tunable("Shooter/Flywheel/kI", ShooterSubsystemConstants.SHOOTER_SPEED_kI);
            private final Tunable tuneSpeedKd = new Tunable("Shooter/Flywheel/kD", ShooterSubsystemConstants.SHOOTER_SPEED_kD);
            private final Tunable tuneSpeedKs = new Tunable("Shooter/Flywheel/kS", ShooterSubsystemConstants.SHOOTER_SPEED_kS);
            private final Tunable tuneSpeedKv = new Tunable("Shooter/Flywheel/kV", ShooterSubsystemConstants.SHOOTER_SPEED_kV);
            private final Tunable tuneSpeedKa = new Tunable("Shooter/Flywheel/kA", ShooterSubsystemConstants.SHOOTER_SPEED_kA);
            private final Tunable tuneAngleKp = new Tunable("Shooter/Hood/kP", ShooterSubsystemConstants.SHOOTER_ANGLE_kP);
            private final Tunable tuneAngleKi = new Tunable("Shooter/Hood/kI", ShooterSubsystemConstants.SHOOTER_ANGLE_kI);
            private final Tunable tuneAngleKd = new Tunable("Shooter/Hood/kD", ShooterSubsystemConstants.SHOOTER_ANGLE_kD);
            private final GenericEntry desiredVelReachedEntry;
            private final GenericEntry desiredAngleReachedEntry;
            private final GenericEntry debugEntry;
            // Raw absolute-encoder position (rotations, pre-conversion) -- the anchor readout for
            // rebuilding getShooterAngleDegrees(). Hand-move the hood to each hard stop (robot
            // DISABLED) and record this at min + max; if it never changes, the encoder isn't tracking.
            private final GenericEntry rawHoodEncoderEntry;
            // Hood drive telemetry (2026-08-22): the commanded volts and the measured current,
            // published every loop. Added to tune the ascent out of stick-slip -- the volts at
            // the instant the hood starts moving IS the breakaway threshold, and there was no
            // way to see it before. Current also shows a stall (driving hard, not moving).
            private final GenericEntry hoodVoltsEntry;
            private final GenericEntry hoodCurrentEntry;
            // Tracker state (2026-08-22): the datum the angle is measured from, and how far the
            // hood has travelled since. Without these the tracked angle is unfalsifiable.
            private final GenericEntry hoodBaseAngleEntry;
            private final GenericEntry hoodRelativeEntry;
            // Live RT flywheel target, both units -- the DPAD trims in RPM, the model works in
            // surface speed, and the driver needs to see what the next press is adjusting.
            private final GenericEntry rtSpeedEntry;
            private final GenericEntry rtSpeedRpmEntry;

    //Tracker Variables
       private boolean enableSubsystem;
       // Last voltage actually commanded to the hood, for the telemetry block at the bottom of
       // periodic() (which publishes whether or not the subsystem is enabled).
       private double lastHoodVolts;
       // CONTINUOUS HOOD TRACKING (2026-08-22). The live angle is a datum captured at enable
       // plus every shortest-path delta since, NOT a fresh interpretation of each reading --
       // see HOOD_DEG_PER_RAW_UNIT for why the absolute reading alone cannot be trusted.
       // Live RT flywheel target (m/s surface), trimmed by the DPAD UP/DOWN bindings. Seeded
       // from the constant and NOT persisted -- a reboot or redeploy returns it to the constant.
       private double rtSurfaceSpeed = ShooterSubsystemConstants.RT_FLYWHEEL_SURFACE_SPEED;
       // True while the hood is parked inside its settle band and held at 0 V by the brake idle
       // mode. Latches, so the loop cannot chatter at the band edge -- see hoodShouldHold().
       private boolean hoodHolding;
       private double hoodBaseAngleDeg;   // absolute angle at the datum
       private double hoodRelativeDeg;    // signed degrees travelled since the datum
       private double hoodLastRaw;        // previous raw reading, for the delta
       private boolean hoodDatumValid;    // false until a datum has been captured
       private boolean hoodWasEnabled;    // rising-edge detect on DriverStation.isEnabled()
       // Open-loop hood jog request, volts; 0 = not jogging. Set by jogHoodCommand while the
       // DPAD is held, applied (and guarded) in periodic(). See jogHoodCommand.
       private double hoodJogVolts;
       private double desired_Velocity;
       private double desired_Angle;
       private double target_distance;

    //Targeting State (team spec 2026-07-16). Tag classification refreshes every loop in
    // periodic() (cheap NT reads); the distance/velocity model runs only while
    // visionShotCommand() is scheduled. velReached/angleReached REPLACE the old mutable
    // statics in ShooterSubsystemConstants (hidden cross-subsystem coupling) -- the hopper
    // now gates its kicker through the isReadyToShoot() supplier instead.
       private enum TargetClass { SCORE, FEED, NONE }
       private TargetClass targetClass = TargetClass.NONE;
       private int visibleTagId = -1;
       private Pose3d tagCameraPose = new Pose3d();
       private boolean hasShotTarget = false;
       private boolean velReached = false;
       private boolean angleReached = false;
       private double movingCompMeters = 0.0;
       private boolean allianceWarned = false;
       // RT auto-aim (team request 2026-07-17): the blue-origin field heading the robot must
       // face to take the current shot. Valid only while runVisionTargeting has a target;
       // consumed by the drivetrain's aim-lock drive layer and the isAimedAtTarget() gate.
       private double aimHeadingDeg = 0.0;
       private boolean aimHeadingValid = false;
       // Tag-flicker ride-through: remember the last non-NONE classification so a momentary
       // dropout (< TARGET_HOLD_SEC) rides on the drivetrain's fused pose instead of
       // restarting the flywheels. Initial acquisition still requires a real tag.
       private TargetClass heldClass = TargetClass.NONE;
       private final Timer targetHoldTimer = new Timer();

       // Vision targeting now lives entirely inside visionShotCommand() -- the old enableVision
       // constructor flag is gone. INTEGRATOR NOTE: construct with just the drivetrain and
       // delete the RobotContainer boot-time disableSubsystem() call; the shot commands enable
       // the subsystem themselves (shooter re-enabled by team directive 2026-07-16).
       public ShooterSubsystem(CommandSwerveDrivetrain drivetrain){

            //Coniguring Motors
                shooterA.getConfigurator().apply(shooterVelConfigs);

                // Current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
                // Flywheels: Kraken X60 - stator 80A / supply 25A, Coast (let them spin down freely).
                // STATOR was raised 40 -> 80 A (Regular tier) 2026-07-17: the feed/precision shots
                // "barely left the robot" -- the 40 A cap choked the torque IMPULSE when a ball hits,
                // so the wheel sagged deep on contact instead of driving the ball. 80 A doubled the
                // anti-bog torque; that spike is brief (ball contact ~tens of ms) and sourced mostly
                // from the flywheel's inertia + DC link, NOT sustained battery draw.
                // STATOR 80 -> 50 A (2026-08-21, team-directed). NOTE: this walks the 07-17 fix most
                // of the way back toward the 40 A that produced weak shots, so the "barely left the
                // robot" symptom is the expected failure mode -- if it returns, restore 80 FIRST.
                // It buys nothing on either axis it might look like it should: supply (unchanged at
                // 25 A) is the brownout knob, and a Kraken X60 stalls at 366 A so 80 A intermittent
                // was never a thermal problem (watch DeviceTemp in PowerTelemetry).
                // UNVERIFIED ON ROBOT: the hood encoder reads 0 (dead) as of this change, so no
                // representative shot could be taken -- retest shots once the hood is restored.
                // SUPPLY deliberately HELD at 25 A: supply is the battery-draw / BROWNOUT knob and the
                // team browns out late-match -- keeping it at 25 A means this change adds ~0 to the
                // spin-up draw (still 25 A x4 = 100 A). If rapid-fire recovery between balls is still
                // weak on retest, raise supply toward 30 A (Regular tier) -- but only on a healthy
                // battery, and expect ~+20 A worst-case drivetrain-shot overlap. Cap is 120/40, never exceed.
                CurrentLimitsConfigs flywheelLimits = new CurrentLimitsConfigs();
                flywheelLimits.StatorCurrentLimit = 50;
                flywheelLimits.StatorCurrentLimitEnable = true;
                flywheelLimits.SupplyCurrentLimit = 25;
                flywheelLimits.SupplyCurrentLimitEnable = true;
                MotorOutputConfigs flywheelOutput = new MotorOutputConfigs();
                flywheelOutput.NeutralMode = NeutralModeValue.Coast;
                for (TalonFX flywheel : new TalonFX[] { shooterA, shooterB, shooterC, shooterD }) {
                    flywheel.getConfigurator().apply(flywheelLimits);
                    flywheel.getConfigurator().apply(flywheelOutput);
                }

                // Hood: NEO 550 - 20A smart limit (fragile; hard cap 30A), Brake to hold angle.
                // kNoResetSafeParameters keeps any inversion/encoder settings flashed via REV Hardware Client.
                SparkMaxConfig hoodConfig = new SparkMaxConfig();
                hoodConfig.smartCurrentLimit(20);
                hoodConfig.idleMode(IdleMode.kBrake);
                shooterAngle.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

                // Follower alignments. All 4 must drive the flywheel the same physical direction.
                // B Aligned, D (CAN 16) Opposed (spun wrong way Aligned, 2026-07-16). C (CAN 15)
                // set Opposed 2026-07-17: team observed it spinning opposite the array while
                // Aligned (likely rewired/remounted since the 2026-07-16 test that had it Aligned).
                // TODO on-robot: confirm C no longer fights/skips -- if it does, revert to Aligned.
                shooterB.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Aligned));
                shooterC.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed));
                shooterD.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed));

            //Initializing Drivetrain
                this.drivetrain = drivetrain;
                this.degreesToAlignToTarget = 0.0;
        
            //Initializing Tracker Variables
                enableSubsystem = true;
                desired_Velocity = 0.0;
                desired_Angle = 0.0;
                target_distance = 0.0;

            //Initializing Shuffleboard Entries
                currentVelEntry = ShooterSubsystemTab.add("Current Velocity", 0.0).getEntry();
                desiredVelEntry = ShooterSubsystemTab.add("Desired Velocity", 0.0).getEntry();
                currentAngleEntry = ShooterSubsystemTab.add("Current Angle", 0.0).getEntry();
                currentRpmEntry = ShooterSubsystemTab.add("Current Flywheel (motor RPM)", 0.0).getEntry();
                desiredAngleEntry = ShooterSubsystemTab.add("Desired Angle", 0.0).getEntry();
                targetDistanceEntry = ShooterSubsystemTab.add("Target Distance", 0.0).getEntry();
                targetClassEntry = ShooterSubsystemTab.add("Target Class", TargetClass.NONE.name()).getEntry();
                shotSetpointEntry = ShooterSubsystemTab.add("Shot Velocity Setpoint", 0.0).getEntry();
                readyToShootEntry = ShooterSubsystemTab.add("Ready To Shoot", false).getEntry();
                aimedEntry = ShooterSubsystemTab.add("Aimed At Target", true).getEntry();
                movingCompEntry = ShooterSubsystemTab.add("Moving Comp Delta", 0.0).getEntry();
                subsystemStateEntry = ShooterSubsystemTab.add("Subsystem State", true).getEntry();
                degreedToAlignToTargEntry = ShooterSubsystemTab.add("Degrees to Align to Target", 0.0).getEntry();
                // Initialized false: with the subsystem disabled at boot these entries were
                // never written again, so a `true` here showed "At Speed: YES" on dashboards
                // while the flywheels coasted.
                desiredVelReachedEntry = ShooterSubsystemTab.add("Desired Velocity Reached", false).getEntry();
                desiredAngleReachedEntry = ShooterSubsystemTab.add("Desired Angle Reached", false).getEntry();
                debugEntry = ShooterSubsystemTab.add("Debug Field", true).getEntry();
                rawHoodEncoderEntry = ShooterSubsystemTab.add("Hood Encoder Raw (rot)", 0.0).getEntry();
                hoodVoltsEntry = ShooterSubsystemTab.add("Hood Volts (cmd)", 0.0).getEntry();
                hoodCurrentEntry = ShooterSubsystemTab.add("Hood Current (A)", 0.0).getEntry();
                hoodBaseAngleEntry = ShooterSubsystemTab.add("Hood Base Angle (deg)", 0.0).getEntry();
                hoodRelativeEntry = ShooterSubsystemTab.add("Hood Travel Since Datum (deg)", 0.0).getEntry();
                rtSpeedEntry = ShooterSubsystemTab.add("RT Target (m/s)", 0.0).getEntry();
                rtSpeedRpmEntry = ShooterSubsystemTab.add("RT Target (motor RPM)", 0.0).getEntry();
       }

    //Utility Methods
        private double getShooterFlywheelVelocity(){
            return 
                shooterA.getVelocity().getValueAsDouble() * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION * 2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS ;
        }

        // Lift a raw hood encoder reading into the CONTINUOUS coordinate the anchors are
        // expressed in. The hood's travel crosses the absolute encoder's 0/360 rollover
        // (measured 2026-08-22: full DOWN raw 151.7, up through 359 -> 0, full UP raw 55.3),
        // so a plain raw value is not monotonic across the stroke -- mid-travel the old map
        // produced -86 deg .. +105 deg and the PID drove the hood into the bottom stop.
        // Anything below the split lies past the rollover and belongs 360 higher.
        // Package-private + static for HoodAngleMapTest.
        static double unwrapHoodRaw(double raw){
            return raw < ShooterSubsystemConstants.HOOD_RAW_WRAP_SPLIT ? raw + 360.0 : raw;
        }

        // Two-point linear map of the UNWRAPPED hood encoder reading -> PHYSICAL hood degrees,
        // from the on-robot HARD-STOP anchors (SubsystemConstants): unwrapped 415.3 (full up) =
        // 44.5 deg, unwrapped 151.7 (full down) = 3.224 deg. Anchored to the PHYSICAL stops, NOT
        // the soft MIN/MAX_ANGLE limits, so insetting those limits never shifts this scale.
        // Package-private + static for HoodAngleMapTest.
        static double hoodDegreesFromRaw(double raw){
            double unwrapped = unwrapHoodRaw(raw);
            return ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP
                + (unwrapped - ShooterSubsystemConstants.HOOD_RAW_AT_FULL_UP)
                  * (ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN - ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP)
                  / (ShooterSubsystemConstants.HOOD_RAW_AT_FULL_DOWN - ShooterSubsystemConstants.HOOD_RAW_AT_FULL_UP);
        }

        // Is a raw hood reading trustworthy enough to close the loop on? Package-private +
        // static for HoodAngleMapTest.
        //
        // TWO checks, and the second one only exists because of the unwrap. Before the wrap
        // fix, a dead encoder's raw 0.0 fell outside the raw band and was caught for free.
        // Unwrapped, 0.0 lifts to 360 -- a perfectly plausible mid-travel ~35.9 deg -- so the
        // band alone would now close the loop on a silent encoder and hold the hood at a
        // fictional angle. A live encoder crossing the rollover reads 0.024, 359.98, and so on;
        // bit-exact 0.0 is what a controller with no encoder data publishes. Rejecting it costs
        // at most a loop or two of 0 V if a real reading ever lands exactly on zero, and the
        // brake idle holds the hood through that.
        // TODO on-robot: replace the zero test with the SPARK MAX's own sensor fault flag once
        // someone confirms which REVLib 2026 fault bit reports a missing absolute encoder.
        static boolean isHoodFeedbackValid(double raw){
            // ONLY the dead-encoder test. The absolute band that used to live here was removed
            // 2026-08-22: with the angle tracked RELATIVELY, no dial position is illegal -- the
            // encoder drifts against the hood, so the band eventually rejects wherever the
            // encoder has wandered to, forcing 0 V and leaving the hood dead on the DPAD. That
            // is exactly the failure it caused: an angle of -32.96 deg with a +38 deg error that
            // never produced a volt. Bad DATA is caught where it belongs now -- in the delta
            // filters (hoodDeltaDegrees), which reject noise and glitches frame by frame.
            //
            // A silent SPARK MAX still publishes bit-exact 0.0, and that is not a position.
            return raw != 0.0;
        }

        // Should the hood be HELD at 0 V (brake idle holding position) rather than driven?
        // True once inside ANGLE_TOLERANCE of the target, and it STAYS true until the hood has
        // drifted past the wider HOOD_REENGAGE_DEG -- hysteresis, so the loop cannot chatter on
        // and off at a single threshold. This is what stops the hood hunting after a DPAD click:
        // it arrives, the drive stops, and the brake holds it there.
        // Package-private + static for HoodSettleTest.
        static boolean hoodShouldHold(double errorDeg, boolean wasHolding){
            double magnitude = Math.abs(errorDeg);
            if (magnitude <= ShooterSubsystemConstants.ANGLE_TOLERANCE) {
                return true;
            }
            return wasHolding && magnitude < ShooterSubsystemConstants.HOOD_REENGAGE_DEG;
        }

        // Gravity LIFT feedforward (volts) for a given angle error, target minus current.
        // RAMPS from 0 V at ANGLE_TOLERANCE to HOOD_RAISE_FF_VOLTS at ANGLE_TOLERANCE +
        // HOOD_FF_FADE_DEG. It used to be a step at ANGLE_TOLERANCE, which drove the hood in
        // visible lurches on the DPAD jog (see HOOD_FF_FADE_DEG). Endpoints are deliberately
        // unchanged: 0 V at/inside the target so the hood is never held against a stop by the
        // FF, full 5 V at a large error so breakaway still clears gravity.
        // Never negative -- lowering is gravity-assisted and stays FF-free.
        // Package-private + static for HoodFeedforwardTest.
        static double hoodLiftFeedforward(double errorDeg){
            // LOWERING (error negative): the hood used to get P only on this stroke, and P is
            // far too small -- -4.90 V measured on the robot moved it not at all. Mirror the
            // ramp downward, at HOOD_LOWER_FF_VOLTS (less than the raise value, since gravity
            // helps here). Without this the 2 deg click could never descend: 2 deg of error is
            // 0.55 V from P alone.
            if (errorDeg < 0) {
                double belowTolerance = -errorDeg - ShooterSubsystemConstants.ANGLE_TOLERANCE;
                double fraction = MathUtil.clamp(
                    belowTolerance / ShooterSubsystemConstants.HOOD_FF_FADE_DEG, 0, 1);
                return -fraction * ShooterSubsystemConstants.HOOD_LOWER_FF_VOLTS;
            }
            double aboveTolerance = errorDeg - ShooterSubsystemConstants.ANGLE_TOLERANCE;
            double fraction = MathUtil.clamp(aboveTolerance / ShooterSubsystemConstants.HOOD_FF_FADE_DEG, 0, 1);
            return fraction * ShooterSubsystemConstants.HOOD_RAISE_FF_VOLTS;
        }

        // Surface speed (m/s) equivalent to a motor speed in RPM. Same geometry the velocity
        // command uses in reverse: rev/s at the motor -> rev/s at the wheel -> rim speed.
        // Package-private + static for ShooterSpeedTrimTest.
        static double surfaceSpeedForMotorRpm(double rpm){
            return rpm / 60.0 * 2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
                * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION;
        }

        // Apply an RPM trim to an RT target, clamped to [0, the motor ceiling]. Zero is a legal
        // floor: it means "refuse", which the at-speed gate can never satisfy, so the kicker
        // stays shut. The ceiling is SHOT_MAX_MOTOR_RPS, the same limit the shot model refuses
        // above -- the DPAD must not be able to ask for a speed the motors cannot hold.
        // Package-private + static for ShooterSpeedTrimTest.
        static double trimSurfaceSpeed(double currentSurfaceSpeed, double rpmDelta){
            double ceiling = ShooterSubsystemConstants.SHOT_MAX_MOTOR_RPS * 2 * Math.PI
                * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
                * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION;
            return MathUtil.clamp(currentSurfaceSpeed + surfaceSpeedForMotorRpm(rpmDelta), 0, ceiling);
        }

        // Current RT flywheel target, m/s surface (dashboard + telemetry).
        public double getRtSurfaceSpeed(){
            return rtSurfaceSpeed;
        }

        // DPAD LEFT/RIGHT (HOLD) = jog the hood at a constant VOLTAGE, left down / right up.
        // Hold to move, release to stop where it is.
        //
        // OPEN LOOP on purpose (2026-08-26). This used to ramp the SETPOINT at
        // HOOD_JOG_DEG_PER_SEC and let the position loop chase it, which oscillated: the hood
        // needs ~7 V to break away, so the loop sat HELD at 0 V until the ramped error passed
        // HOOD_REENGAGE_DEG and the feedforward faded in, then slammed ~7.5 V, jumped past the
        // target into the settle band and stopped dead -- stick-slip, roughly 0.75 s per cycle.
        // No gain retune fixes that; a position loop fed a slow setpoint through that much
        // stiction must stick-slip. A held jog is a velocity request, so it is driven as one.
        //
        // The voltage is applied in periodic() (see hoodJogVolts), NOT here, so the jog still
        // passes through the feedback sanity gate and the MIN/MAX_ANGLE hard travel guards --
        // the dead-encoder and belt-skip protections are unchanged. periodic() also keeps the
        // setpoint pinned to the live angle while jogging, so releasing hands back to the
        // position loop at zero error and the brake holds it there.
        //
        // Requires NO subsystem, deliberately. The RT shot group runs kCancelIncoming, so a
        // requiring command would be BLOCKED for the whole hold -- which would kill the mid-hold
        // hood trim that flywheelOnlyShotCommand explicitly depends on.
        public Command jogHoodCommand(double volts){
            return Commands.runEnd(() -> hoodJogVolts = volts, () -> hoodJogVolts = 0);
        }

        // DPAD LEFT/RIGHT (press) = move the hood a FIXED amount, left down / right up (team
        // request 2026-08-27). One press = one step of HOOD_STEP_DEG.
        //
        // This is NOT the per-click setpoint step that 5af2471 removed. That handed the
        // position loop a step it answered at full feedforward (a 2 deg click flew 9.5 deg).
        // Here the DRIVE is still the proven open-loop jog voltage; only the STOP is new, and
        // it comes from the measured hood angle, so the travel cannot outrun the request.
        //
        // The voltage goes through hoodJogVolts, so periodic() still applies the feedback
        // sanity gate and the MIN/MAX hard travel guards exactly as for the held jog. The
        // target is clamped with clampDesiredAngle (not a raw MIN/MAX clamp) so a hood parked
        // outside the soft band is not dragged back into it, and a press already at the limit
        // ends at once instead of pushing into the guard for the whole timeout.
        //
        // The timeout is load-bearing, not a formality: if the hood does not move (the
        // 2026-08-26 trace showed -6 V for 4.5 s producing no motion at all), this is the only
        // thing that ends the press. Drive is bounded by the guards and the 20 A smart limit
        // throughout. A second press of the same direction while a step is still running is
        // swallowed (one instance per binding) -- one step per press, by design.
        //
        // Requires NO subsystem, for the same reason jogHoodCommand does not: the RT shot group
        // runs kCancelIncoming and would block a requiring command for the whole hold.
        public Command stepHoodCommand(double degreesDelta){
            double volts = degreesDelta > 0
                ? ShooterSubsystemConstants.HOOD_JOG_UP_VOLTS
                : -ShooterSubsystemConstants.HOOD_JOG_DOWN_VOLTS;
            double[] stopAt = new double[1];
            return new FunctionalCommand(
                    () -> stopAt[0] = clampDesiredAngle(getShooterAngleDegrees() + degreesDelta,
                                                        hoodBaseAngleDeg, hoodDatumValid),
                    () -> hoodJogVolts = volts,
                    interrupted -> hoodJogVolts = 0,
                    () -> degreesDelta > 0
                        ? getShooterAngleDegrees() >= stopAt[0]
                        : getShooterAngleDegrees() <= stopAt[0])
                .withTimeout(ShooterSubsystemConstants.HOOD_STEP_TIMEOUT_SEC);
        }

        // DPAD UP/DOWN (press) = trim the RT flywheel target by rpmDelta motor RPM (team request
        // 2026-08-22). Takes effect on the NEXT loop of a live shot too, since
        // flywheelOnlyShotCommand re-reads the field every loop. runOnce, so a press is one step.
        public Command trimRtSpeedCommand(double rpmDelta){
            return Commands.runOnce(() -> rtSurfaceSpeed = trimSurfaceSpeed(rtSurfaceSpeed, rpmDelta));
        }

        // Shortest-path difference between two raw readings, in raw units. A 359 -> 0.5 step
        // is +1.5, not -358.5. This is what makes the rollover a non-event: the tracker never
        // sees the discontinuity that the fixed-split interpretation turns into a 56 deg flip.
        // Package-private + static for HoodTrackingTest.
        static double shortestRawDelta(double previousRaw, double currentRaw){
            double delta = (currentRaw - previousRaw) % 360.0;
            if (delta > 180.0) {
                delta -= 360.0;
            } else if (delta < -180.0) {
                delta += 360.0;
            }
            return delta;
        }

        // Convert one loop's raw delta into degrees of hood travel, rejecting what is not
        // motion: noise below the deadband (so drift/jitter can never accumulate into phantom
        // travel -- the team's "slight changes in angle must not affect the hood") and glitches
        // above the max step (a garbled frame is not a 157 deg/sec hood).
        // Package-private + static for HoodTrackingTest.
        static double hoodDeltaDegrees(double rawDelta){
            double magnitude = Math.abs(rawDelta);
            if (magnitude < ShooterSubsystemConstants.HOOD_RAW_NOISE_DEADBAND
                    || magnitude > ShooterSubsystemConstants.HOOD_RAW_MAX_STEP) {
                return 0;
            }
            return rawDelta * ShooterSubsystemConstants.HOOD_DEG_PER_RAW_UNIT;
        }

        // Clamp a requested angle into what the hood can actually reach: never BELOW the
        // enable-time datum (hoodFloorAngle), and never more than HOOD_TRAVEL_WINDOW_DEG ABOVE
        // it. Wherever it was enabled, a setpoint can never ask for more travel than the
        // mechanism has, and never asks for a descent past where this enable started.
        //
        // THE SOFT BAND MAY NEVER DEMAND MOTION (2026-08-26, team request: "it should not move
        // at all when enabled"). The hood RESTS at ~3.2 deg, below the 5.0 deg MIN_ANGLE floor.
        // The old band clamped the enable-time seed (see captureHoodDatum) up to 5.0, so every
        // enable produced a 1.8 deg error -- past ANGLE_TOLERANCE and past HOOD_FF_FADE_DEG, so
        // the full lift feedforward fired and drove the hood UP at ~7.6 V the instant the robot
        // was enabled. Widening each limit to include the datum makes the seeded setpoint equal
        // the enable position EXACTLY, so the hood holds until something commands it.
        //
        // The band still only ever OPENS toward where the hood already is, so it cannot be used
        // to escape: enabled below MIN_ANGLE the hood can only be commanded UP, enabled above
        // MAX_ANGLE only DOWN. The hard travel guard in periodic() is what actually protects the
        // belt at the top, and it is unchanged.
        //
        // low <= baseAngleDeg <= high always holds now, so the limits can no longer cross and
        // the old degenerate-window branch is gone. Package-private + static for HoodTrackingTest.
        static double clampDesiredAngle(double angle, double baseAngleDeg, boolean datumValid){
            double low = hoodFloorAngle(baseAngleDeg, datumValid);
            double high = ShooterSubsystemConstants.MAX_ANGLE;
            if (datumValid) {
                high = Math.max(high, baseAngleDeg); // never demand a drop to reach the ceiling
                high = Math.min(high, baseAngleDeg + ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG);
            }
            return MathUtil.clamp(angle, low, Math.max(low, high));
        }

        // THE FLOOR THE HOOD MAY NEVER GO BELOW (team request 2026-08-27): the enable-time
        // datum itself. hoodBaseAngleDeg is re-captured from the raw encoder on every
        // disable -> enable edge (see captureHoodDatum), so the floor is wherever the hood
        // actually sat when this enable started -- not a fixed number, and not carried over
        // from the last session. Before any datum exists the fixed MIN_ANGLE soft limit stands
        // in, since there is nothing measured to floor against yet.
        //
        // This replaces the old base - HOOD_TRAVEL_WINDOW_DEG lower edge, which let the DPAD
        // walk the hood a whole window BELOW the enable position and into the bottom stop.
        // CONSEQUENCE, on purpose: enable the robot with the hood already raised and it cannot
        // be lowered past that point for the rest of the enable -- by the DPAD or by a
        // commanded shot angle. Enable with the hood resting (~3.2 deg), which is the normal
        // case, and nothing changes. The one exception is a hood enabled above MAX_ANGLE --
        // see the cap below. Used by BOTH clampDesiredAngle and the hard travel guard in
        // periodic(), so the open-loop DPAD drive is bounded by it too.
        // Package-private + static for HoodTrackingTest.
        static double hoodFloorAngle(double baseAngleDeg, boolean datumValid){
            // Capped at MAX_ANGLE: a hood enabled ABOVE the safe ceiling (already in the
            // belt-skip zone) must still be able to come DOWN to it. That is the one case
            // where descending below the enable position is required, not forbidden.
            return datumValid ? Math.min(baseAngleDeg, ShooterSubsystemConstants.MAX_ANGLE)
                              : ShooterSubsystemConstants.MIN_ANGLE;
        }

        // The datum angle for a fresh capture: the absolute map, CLAMPED into the physical
        // stops. The clamp is the defence against capturing a flipped reading -- the -4.3 deg
        // and +52 deg the fixed split produces either side of raw 103.5 both land back on a
        // real stop instead of seeding the tracker with a fiction.
        // Package-private + static for HoodTrackingTest.
        static double hoodDatumAngle(double raw){
            return MathUtil.clamp(hoodDegreesFromRaw(raw),
                ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN,
                ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP);
        }

        // Live hood angle: the datum plus everything travelled since. Falls back to the raw
        // absolute map only before a datum exists (first loops after boot, robot never enabled).
        private double getShooterAngleDegrees(){
            if (hoodDatumValid) {
                return hoodBaseAngleDeg + hoodRelativeDeg;
            }
            return hoodDatumAngle(shooterAngleEncoder.getPosition());
        }

        // Capture a fresh datum HERE: this position becomes the reference every later reading
        // is measured against. Called on each disable -> enable transition, so a session's
        // accumulated encoder drift is discarded rather than carried into the next enable.
        // Accumulate one loop's travel, holding the tracked angle inside the PHYSICAL stops.
        // The hood cannot pass its stops, so an accumulation that would take the angle outside
        // them is slip or noise, not motion -- letting it run is what produced a reported
        // -32.96 deg on a hood whose travel is 3.2 to 44.5. Saturating instead of accumulating
        // also means the travel guard still knows which way is blocked.
        // Package-private + static for HoodTrackingTest.
        static double accumulateHoodTravel(double relativeDeg, double baseAngleDeg, double deltaDeg){
            double proposed = relativeDeg + deltaDeg;
            double angle = baseAngleDeg + proposed;
            if (angle > ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP) {
                return ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP - baseAngleDeg;
            }
            if (angle < ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN) {
                return ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN - baseAngleDeg;
            }
            return proposed;
        }

        private void captureHoodDatum(double raw){
            hoodBaseAngleDeg = hoodDatumAngle(raw);
            hoodRelativeDeg = 0;
            hoodLastRaw = raw;
            hoodDatumValid = true;
            // SEED THE SETPOINT TO WHERE THE HOOD ACTUALLY IS (2026-08-22). Without this the
            // setpoint keeps whatever stale value it had -- measured on the robot: the hood sat
            // at 34.84 deg with desired_Angle still 17 from an earlier session, so eight +2 deg
            // clicks only walked the target from 17 to 33, still BELOW the hood. Every click
            // registered and the hood never moved up, because the loop was trying to go DOWN
            // the whole time. Seeding here means a click is always 2 deg from where the hood is.
            setDesired_Angle(hoodBaseAngleDeg);
        }

        // Own alliance for the tag partition. DriverStation may not know yet (no FMS / DS
        // pick): default BLUE and warn ONCE instead of spamming every 20 ms loop.
        private Alliance ownAlliance(){
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                if (!allianceWarned) {
                    DriverStation.reportWarning(
                        "ShooterSubsystem: DriverStation alliance unknown -- defaulting to BLUE tag partition", false);
                    allianceWarned = true;
                }
                return Alliance.Blue;
            }
            return alliance.get();
        }

        // Team tag partition (FieldConstants, spec 2026-07-16): SCORE only on OUR alliance's
        // scoring faces (never the opponent hub), FEED on the neutral-facing feed set (either
        // alliance), anything else = NONE (no flywheels, no belts, no kicker).
        private TargetClass classifyTag(int tagId){
            if (FieldConstants.ownScoreTags(ownAlliance()).contains(tagId)) return TargetClass.SCORE;
            if (FieldConstants.FEED_TAGS.contains(tagId)) return TargetClass.FEED;
            return TargetClass.NONE;
        }
    //Subsystem Methods
        public void enableSubsystem(){
            enableSubsystem = true;
        }

        public void disableSubsystem(){
            enableSubsystem = false;
        }

        public void setDesiredFlywheelVelocity(double velocity){
            desired_Velocity = velocity;
        }

        public double getDesiredFlywheelVelocity(){
            return desired_Velocity;
        }

        public void setDesired_Angle(double angle){
            desired_Angle = clampDesiredAngle(angle, hoodBaseAngleDeg, hoodDatumValid);
        }
        public double  getDesiredAngle(){
            return desired_Angle;
        }
        // CCW-positive heading correction (deg) to face the primary SCORE tag of OUR alliance
        // (add it to the current heading), refreshed every loop in periodic(); 0 when no such
        // tag is visible (align commands just hold heading).
        public double getDegreesToAlignToTarget(){
            return degreesToAlignToTarget;
        }

        // True while the primary in-view tag is a SCORE tag of OUR alliance (search-align gate).
        public boolean seesScoringTag(){
            return targetClass == TargetClass.SCORE;
        }

        // True while the active vision shot has a live, legal, in-envelope firing solution.
        public boolean hasShotTarget(){
            return hasShotTarget;
        }

        // Blue-origin field heading (deg) the robot must face to take the current RT shot;
        // empty when no vision aim target exists (no target, or RB blind feed). Consumed by
        // the drivetrain's aim-lock drive layer, re-sampled every loop.
        public OptionalDouble getAimHeadingDegrees(){
            return aimHeadingValid ? OptionalDouble.of(aimHeadingDeg) : OptionalDouble.empty();
        }

        // Aim gate: robot heading within HEADING_TOLERANCE_DEG of the firing bearing.
        // True when no vision aim target exists -- the RB blind feed (driver-aimed) must
        // still be able to open the kicker. Rotation2d.minus wraps, so 179 vs -179 = 2 deg.
        public boolean isAimedAtTarget(){
            if (!aimHeadingValid) {
                return true;
            }
            double errDeg = drivetrain.getState().Pose.getRotation()
                .minus(Rotation2d.fromDegrees(aimHeadingDeg)).getDegrees();
            return Math.abs(errDeg) <= FieldConstants.HEADING_TOLERANCE_DEG;
        }

        // Kicker gate: at-speed AND at-angle AND aimed (when a vision aim target exists) AND a
        // shot actually commanded. A refused or absent target commands velocity 0, which can
        // never read "at speed", so this one predicate is safe for both the RT model shot and
        // the RB fixed feed without extra flags. The aim term physically prevents feeding a
        // shot pointed the wrong way (team request 2026-07-17).
        public boolean isReadyToShoot(){
            return velReached && angleReached && isAimedAtTarget();
        }

        // Kicker gate, SPEED ONLY (team request 2026-07-18: "kicker only starts after the
        // flywheels are up to speed"). True once the measured flywheel surface speed is within
        // SPEED_TOLERANCE of a NONZERO commanded speed -- a refused/absent shot commands 0 and
        // can never read at-speed, so this also keeps the kicker off stopped wheels. Deliberately
        // ignores hood angle + aim (unlike isReadyToShoot) so it still opens while the hood
        // encoder is down; tighten to isReadyToShoot once the hood feedback is trusted.
        // NOTE: ball contact briefly slows the wheels below tolerance, pausing the kicker until
        // they recover -- that paces fuel so every ball leaves at speed. If the kicker never
        // opens at all, SPEED_TOLERANCE (0.2 m/s) is likely too tight for the untuned velocity
        // loop -- widen it before blaming the gate.
        public boolean isFlywheelAtSpeed(){
            return velReached;
        }

        // Shot model core (SHOT MODEL block in ShooterSubsystemConstants has the derivation).
        // No-drag closed form for exit speed through (distance, deltaH) at the FIXED max hood
        // angle (38 deg; fits + envelope refit at 38 on 2026-07-21 -- docs/shot-model/
        // refit_38.py), times the distance-fitted drag multiplier, converted to flywheel
        // SURFACE speed (m/s). Returns 0 (= refuse: CoastOut path, at-speed gate never opens,
        // kicker never feeds) when the ball cannot cross the target height DESCENDING or the
        // motor velocity ceiling is exceeded.
        // Package-private for ShotModelTest (pins this against the drag integrator's truth).
        static double modelSurfaceSpeed(double distance, double deltaH, double dragMult){
            double theta = Math.toRadians(ShooterSubsystemConstants.MAX_ANGLE);
            double reach = distance * Math.tan(theta);
            // Must cross the target height DESCENDING (past apex): d*tan(theta) > 2*deltaH.
            if (distance <= 0 || reach <= 2 * deltaH) {
                return 0;
            }
            double cos = Math.cos(theta);
            double vBall = Math.sqrt(9.81 * distance * distance / (2 * cos * cos * (reach - deltaH)))
                    * dragMult;
            double vSurface = vBall / ShooterSubsystemConstants.SHOT_EFFICIENCY;
            double motorRps = vSurface / (2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
                    * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION);
            if (motorRps > ShooterSubsystemConstants.SHOT_MAX_MOTOR_RPS) {
                return 0; // out of range -- refuse rather than lob short into the field
            }
            return vSurface;
        }

        // Time-of-flight linear fit (s) for the given target class (moving-shot comp, spec 5).
        private static double timeOfFlightSec(double distance, TargetClass cls){
            return cls == TargetClass.SCORE
                ? ShooterSubsystemConstants.SCORE_TOF_BASE_SEC
                    + ShooterSubsystemConstants.SCORE_TOF_SEC_PER_METER * distance
                : ShooterSubsystemConstants.FEED_TOF_BASE_SEC
                    + ShooterSubsystemConstants.FEED_TOF_SEC_PER_METER * distance;
        }

        // No firing solution: wheels to 0 (CoastOut path -- the at-speed gate can never open,
        // so belts/kicker never feed). THE HOOD IS DELIBERATELY NOT TOUCHED (2026-08-26, team
        // request: no auto-stow anywhere). This used to take a hood argument and every caller
        // passed MIN_ANGLE, so a refused or released shot yanked the hood down and threw away
        // wherever the driver had jogged it.
        private void refuseShot(){
            hasShotTarget = false;
            movingCompMeters = 0;
            setDesiredFlywheelVelocity(0);
        }

        // RT targeting core -- one pass per loop while visionShotCommand() is scheduled.
        // Classification (SCORE own alliance / FEED / NONE) was already refreshed by periodic()
        // this loop (the scheduler runs subsystem periodic() before command execute()).
        private void runVisionTargeting(){
            // Effective class for THIS loop: a fresh tag wins; a momentary dropout within
            // TARGET_HOLD_SEC of the last real tag keeps the previous class alive (ride-through,
            // team request 2026-07-17) so the flywheels don't restart on a one-frame flicker.
            // Initial acquisition still requires a real tag: heldClass is cleared when the RT
            // command starts (beforeStarting) and when it ends.
            TargetClass effClass = targetClass;
            boolean holdover = false;
            if (effClass == TargetClass.NONE && heldClass != TargetClass.NONE
                    && !targetHoldTimer.hasElapsed(ShooterSubsystemConstants.TARGET_HOLD_SEC)) {
                effClass = heldClass;
                holdover = true;
            }
            if (effClass == TargetClass.NONE) {
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot();
                return;
            }

            // Field-pose source: a fresh Limelight botpose while a tag is in view (wpiBlue
            // frame; getBotPoseEstimate returns null on empty NT data); during a holdover the
            // drivetrain's FUSED pose instead -- MegaTag2-corrected odometry carries the last
            // vision fix forward across the dropout, so strafing keeps both distance and aim
            // accurate with no tag in frame. NOTE: botpose is only as good as the camera mount
            // pose set in the Limelight web UI -- TODO on-robot: verify the mount config; until
            // then the camera-space fallback below is effectively the primary path.
            var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Vision.CAM_LIMELIGHT);
            boolean botposeValid = !holdover && poseEstimate != null && poseEstimate.tagCount > 0;
            Pose2d fieldPose = botposeValid ? poseEstimate.pose
                             : holdover ? drivetrain.getState().Pose
                             : null;

            double distance;   // horizontal m, robot -> target point
            double vRadial;    // m/s along the target line, positive = moving AWAY from target
            if (fieldPose != null) {
                Alliance alliance = ownAlliance();
                Translation2d robot = fieldPose.getTranslation();
                Translation2d target = effClass == TargetClass.SCORE
                    ? FieldConstants.ownHubCenter(alliance)
                    : FieldConstants.ownFeedTarget(alliance);
                distance = robot.getDistance(target);
                // AUTO-AIM bearing (team request 2026-07-17): blue-origin field heading from the
                // SAME pose used for distance to the target point. Stays valid through the
                // legality/envelope refusals below, so the drivetrain holds aim while the driver
                // strafes back into range.
                if (distance > 0.01) {
                    aimHeadingDeg = Math.toDegrees(Math.atan2(
                        target.getY() - robot.getY(), target.getX() - robot.getX()));
                    aimHeadingValid = true;
                } else {
                    aimHeadingValid = false;
                }
                if (effClass == TargetClass.SCORE
                        && !FieldConstants.isLegalScoringX(robot.getX(), alliance)) {
                    // Rule G407: never spin up a scoring shot from outside our alliance zone.
                    // Hood untouched -- this used to stage it to MAX, which is the robot moving
                    // the hood on its own (no auto motion, 2026-08-26).
                    target_distance = 0;
                    refuseShot();
                    return;
                }
                // Radial speed: rotate the robot-relative chassis speeds into the field frame
                // with the SAME pose's rotation (one consistent frame), then project onto the
                // robot->target unit vector. Positive = distance growing = moving away.
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    drivetrain.getState().Speeds, fieldPose.getRotation());
                double ux = (target.getX() - robot.getX()) / distance;
                double uy = (target.getY() - robot.getY()) / distance;
                vRadial = -(fieldSpeeds.vxMetersPerSecond * ux + fieldSpeeds.vyMetersPerSecond * uy);
            } else if (effClass == TargetClass.SCORE) {
                // Distance source 2 (fallback, fresh SCORE tag only): camera-space tag
                // translation + the fixed tag-face -> hub-center geometry (every hub tag plane
                // sits 0.6035 m in front of the hub center). Camera space X = right, Z =
                // forward; treated as the ROBOT frame until the camera mount pose is measured
                // (TODO on-robot -- a mount offset shifts every fallback distance).
                double x = tagCameraPose.getX() + FieldConstants.tagLateralOffsetMeters(visibleTagId);
                double z = tagCameraPose.getZ() + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS;
                distance = Math.hypot(x, z);
                // Radial speed in the robot frame (x forward, y LEFT; the camera bearing is
                // positive to the RIGHT, so the target direction is (cos b, -sin b)).
                ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
                double bearing = Math.atan2(x, z);
                // AUTO-AIM bearing, fallback path: current heading MINUS the right-positive
                // camera bearing = CCW-positive field heading to the HUB CENTER (the lateral
                // term steers to center, not the tag face, once TAG_LATERAL_OFFSET_SIGN is set).
                aimHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees()
                    - Math.toDegrees(bearing);
                aimHeadingValid = true;
                vRadial = -(robotSpeeds.vxMetersPerSecond * Math.cos(bearing)
                          - robotSpeeds.vyMetersPerSecond * Math.sin(bearing));
            } else {
                // FEED with no usable botpose and no holdover pose: a camera-space tag cannot
                // locate a field aim point, so REFUSE (no shooting operations). The driver
                // still has RB for a deliberate blind lob -- NOT auto-substituted here
                // (deviation from shot-math spec 3.2 per team task spec: refusing is the
                // conservative branch).
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot();
                return;
            }
            // Degenerate-pose guard: a garbage botpose sitting ON the target point would divide
            // by ~0 above and float a NaN through every envelope gate (NaN comparisons are all
            // false) straight into VelocityVoltage. Refuse instead.
            if (!(distance > 0.01)) {   // also catches NaN
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot();
                return;
            }
            target_distance = distance;

            // Moving-shot radial compensation (spec 5): virtual-target fixed point in effective
            // distance, exactly two iterations, clamped to +-1.5 m. Lateral (tangential) motion
            // is NOT compensated -- documented limitation; the aim lock now keeps the nose on
            // target while strafing, but the ball still inherits tangential velocity: shoot
            // roughly still or radial. A fast approach into the score dead zone correctly
            // REFUSES via the envelope gate below.
            double gain = ShooterSubsystemConstants.MOVING_COMP_GAIN;
            double d1 = distance + gain * vRadial * timeOfFlightSec(distance, effClass);
            movingCompMeters = MathUtil.clamp(gain * vRadial * timeOfFlightSec(d1, effClass),
                -ShooterSubsystemConstants.MOVING_COMP_MAX_METERS,
                ShooterSubsystemConstants.MOVING_COMP_MAX_METERS);
            double dEff = distance + movingCompMeters;

            double vSurface;
            if (effClass == TargetClass.SCORE) {
                // Outside [3.0, 6.2] m the shot is a guaranteed miss or illegal (spec 2.3) --
                // refuse. The hood used to be staged to MAX here; it is now left alone, since
                // nothing may move the hood without a driver command (2026-08-26).
                if (dEff < ShooterSubsystemConstants.MIN_SCORE_DISTANCE_METERS
                        || dEff > ShooterSubsystemConstants.MAX_SCORE_DISTANCE_METERS) {
                    refuseShot();
                    return;
                }
                vSurface = modelSurfaceSpeed(dEff,
                    FieldConstants.HUB_OPENING_HEIGHT_METERS
                        - ShooterSubsystemConstants.SHOT_RELEASE_HEIGHT_METERS,
                    ShooterSubsystemConstants.SCORE_DRAG_MULT_BASE
                        + ShooterSubsystemConstants.SCORE_DRAG_MULT_PER_METER * dEff);
            } else {
                // FEED lob lands on the carpet (deltaH = -release height). Clamp, never refuse
                // for range: a slightly-short lob still lands in friendly territory (spec 3.2).
                dEff = MathUtil.clamp(dEff,
                    ShooterSubsystemConstants.MIN_FEED_DISTANCE_METERS,
                    ShooterSubsystemConstants.MAX_FEED_DISTANCE_METERS);
                vSurface = modelSurfaceSpeed(dEff,
                    -ShooterSubsystemConstants.SHOT_RELEASE_HEIGHT_METERS,
                    ShooterSubsystemConstants.FEED_DRAG_MULT_BASE
                        + ShooterSubsystemConstants.FEED_DRAG_MULT_PER_METER * dEff);
            }
            setDesired_Angle(ShooterSubsystemConstants.MAX_ANGLE);
            setDesiredFlywheelVelocity(vSurface);
            hasShotTarget = vSurface > 0;
        }

    //Command Based Methods
        public Command enableSubsystemCommand(){
            return Commands.runOnce(()->{
                        this.enableSubsystem();
                    });
        }
        
        public Command disableSubsystemCommand(){
            return Commands.runOnce(()->{
                        this.disableSubsystem();
                    });
        }

        public Command setAngleAndVelocityCommand(double angle, double velocity){
            return runOnce(()->{
                        this.setDesired_Angle(angle);
                        this.setDesiredFlywheelVelocity(velocity);
                    });
        }

        public Command enableLiveData(boolean isEnabled){
           return Commands.runOnce(
            ()->{
                this.enableComp = isEnabled;
                // Leaving live-data mode: zero the VELOCITY ONCE here (this used to happen
                // every loop in periodic(), which also stomped preset shot commands). The hood
                // is left alone -- this line also drove it to MIN_ANGLE, and it wrote the field
                // directly, so it bypassed setDesired_Angle's clamp entirely (2026-08-26).
                if (!isEnabled) {
                    desired_Velocity = 0;
                }
            }
            );
        }

        // Coast the flywheels down. The HOOD IS LEFT WHERE IT IS (2026-08-26, team request: no
        // auto-stow anywhere) -- this used to drop it to MIN_ANGLE, throwing away whatever angle
        // the driver had jogged it to every time a shot was released.
        public Command stopShooterCommand(){
            return runOnce(() -> setDesiredFlywheelVelocity(0));
        }

        // RT (hold) = precision vision shot. Every loop: classify the primary tag (SCORE own
        // alliance / FEED / NONE), pick the distance source (botpose, else camera-space for
        // SCORE only), moving-comp, envelope gates, then hood MAX + model velocity. NONE or any
        // refusal keeps velocity 0 so belts/kicker never feed. Release: flywheels coast
        // (0 -> CoastOut), hood LEFT WHERE IT IS (no auto-stow, 2026-08-26), target flags clear.
        // (The old testSpinCommand/hoodJogCommand and their RT/DPAD bindings are superseded and
        // deleted per team spec 2026-07-16.)
        public Command visionShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();   // shot commands re-enable per team directive 2026-07-16
                    runVisionTargeting();
                },
                () -> {
                    refuseShot();
                    target_distance = 0;
                    aimHeadingValid = false;
                    heldClass = TargetClass.NONE; // next press must acquire a fresh tag
                })
                // Clear any pre-press residue so the spec holds exactly: pressing RT with no
                // tag found does nothing, even if one was seen moments before the press.
                .beforeStarting(() -> heldClass = TargetClass.NONE);
        }

        // RT (hold) = FLYWHEELS ONLY (team request 2026-08-22): spin to a fixed surface speed
        // and NEVER touch the hood -- the angle is whatever the DPAD jog left it at, which is
        // the whole point (manual range control). No vision, no model, no aim. The RT binding
        // runs belts always and gates the kicker on isFlywheelAtSpeed(). Release: velocity 0
        // (CoastOut), hood deliberately left where it is. (stopShooterCommand no longer moves
        // the hood either, so nothing can undo the driver's setting any more.)
        public Command flywheelOnlyShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();
                    // Every loop, so a DPAD trim mid-hold takes effect immediately.
                    setDesiredFlywheelVelocity(rtSurfaceSpeed);
                },
                () -> {
                    hasShotTarget = false;
                    setDesiredFlywheelVelocity(0);
                });
        }

        // RB (hold) = fixed blind feed: fixed hood angle + fixed tunable feed surface speed.
        // The RB binding runs belts always and gates the kicker on isFlywheelAtSpeed() (team
        // request 2026-07-18) -- fuel feeds only once the wheels actually reach the commanded
        // speed, never into wheels still spinning up. Release: flywheels coast, hood unchanged.
        public Command feedAngleShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();
                    setDesired_Angle(ShooterSubsystemConstants.RB_FEED_ANGLE);
                    setDesiredFlywheelVelocity(ShooterSubsystemConstants.RB_FEED_SURFACE_SPEED);
                },
                () -> refuseShot());
        }

    @Override
    public void periodic(){
        // Vision sensing -- read-only, EVERY loop (even while disabled): classify the primary
        // tag and keep the align bearing fresh for the drivetrain's A / DPAD-UP search-align.
        // Cheap NetworkTables reads for the PRIMARY target only -- the old getLatestResults()
        // call deserialized the Limelight's full JSON dump with Jackson every 20 ms loop,
        // which stalls the whole robot loop.
        targetClass = TargetClass.NONE;
        visibleTagId = -1;
        if (LimelightHelpers.getTV(Vision.CAM_LIMELIGHT)) {
            visibleTagId = (int) LimelightHelpers.getFiducialID(Vision.CAM_LIMELIGHT);
            targetClass = classifyTag(visibleTagId);
        }
        // Record the tag-flicker ride-through state (see runVisionTargeting): any real
        // classification refreshes the hold window.
        if (targetClass != TargetClass.NONE) {
            heldClass = targetClass;
            targetHoldTimer.restart();
        }
        if (targetClass == TargetClass.SCORE) {
            // Limelight camera space: X = right, Y = DOWN, Z = forward (depth). Cached here for
            // the fallback distance path. TODO verify signs/axes on the robot with a real tag.
            tagCameraPose = LimelightHelpers.getTargetPose3d_CameraSpace(Vision.CAM_LIMELIGHT);
            // Camera X is RIGHT-positive but WPILib field heading is CCW-positive, so the
            // bearing is NEGATED: a tag to the robot's right needs a clockwise (negative)
            // heading delta. Consumers ADD this value to the current heading to face the tag.
            degreesToAlignToTarget = Math.toDegrees(Math.atan2(-tagCameraPose.getX(), tagCameraPose.getZ()));
        } else {
            degreesToAlignToTarget = 0; // no OWN score tag -> align commands just hold heading
        }

        if(enableSubsystem){
            //Shooter Speed
                m_velocity.Slot = 0;
                double motorRps = desired_Velocity /(2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION);
                if (motorRps == 0) {
                    // "Off" = coast down freely, never VelocityVoltage(0) (see m_flywheelCoast note).
                    shooterA.setControl(m_flywheelCoast);
                } else {
                    shooterA.setControl(m_velocity.withVelocity(motorRps));
                }

            //Shooter Angle
                // NO AUTOMATIC HOOD MOTION (team request 2026-08-26). The hood moves only when
                // something explicitly commands an angle -- in the match bindings that is the
                // DPAD jog and nothing else. Enabling re-datums to wherever the hood sits and
                // seeds the setpoint there, so the hood holds; the old hood-lower interlock and
                // the coast-down auto-recess it gated are both gone.
                double rawHood = shooterAngleEncoder.getPosition();
                // CONTINUOUS TRACKING. Datum on the disable -> enable edge (a session's drift is
                // discarded, never carried across enables), then accumulate shortest-path deltas.
                // Noise and glitches are filtered out in hoodDeltaDegrees, so a still hood cannot
                // drift its own angle reading.
                boolean nowEnabled = DriverStation.isEnabled();
                if (isHoodFeedbackValid(rawHood) && (!hoodDatumValid || (nowEnabled && !hoodWasEnabled))) {
                    captureHoodDatum(rawHood);
                } else if (hoodDatumValid && isHoodFeedbackValid(rawHood)) {
                    hoodRelativeDeg = accumulateHoodTravel(hoodRelativeDeg, hoodBaseAngleDeg,
                        hoodDeltaDegrees(shortestRawDelta(hoodLastRaw, rawHood)));
                    hoodLastRaw = rawHood;
                }
                hoodWasEnabled = nowEnabled;
                double currentHoodAngle = getShooterAngleDegrees();
                // SAME band as setDesired_Angle -- one definition, both callers. A raw
                // clamp(desired_Angle, MIN_ANGLE, MAX_ANGLE) here would re-introduce the
                // enable-time lurch even with the setter fixed, by pulling a hood parked below
                // the 5.0 deg floor back up to it every loop.
                double hoodTarget = clampDesiredAngle(desired_Angle, hoodBaseAngleDeg, hoodDatumValid);
                // The stow interlock that used to sit here (hold the hood up while the flywheels
                // coast down) is GONE with auto-stow itself (2026-08-26): nothing lowers the hood
                // on its own any more, so the only thing it could still block was a DPAD-down
                // press while the wheels spun -- which is exactly the one motion the driver is
                // supposed to always get.
                // FEEDBACK SANITY GATE (verify-review 2026-07-18): only close the loop on a raw
                // reading inside the calibrated band. An unplugged/silent absolute encoder reads 0,
                // which the two-point map turns into ~105 deg -- the PID would then drive DOWN at
                // 6 V into the bottom hard stop forever (the MIN guard never fires because the
                // mapped angle reads high, not low) and stall the fragile NEO 550 at its 20 A
                // limit. Out-of-band or NaN (comparisons fail) -> 0 V; brake idle holds the hood.
                boolean hoodFeedbackValid = isHoodFeedbackValid(rawHood);
                double hoodVolts = 0; // kill switch / invalid feedback -> 0 V (brake idle holds)
                if (HOOD_CLOSED_LOOP_ENABLED && hoodFeedbackValid && hoodJogVolts != 0) {
                    // HELD DPAD JOG: drive open loop at the requested voltage and keep the
                    // setpoint pinned to where the hood actually is, so releasing resumes the
                    // position loop at zero error (it latches into hold and the brake parks it)
                    // instead of snapping back to a stale setpoint. The MIN/MAX guards after
                    // the branches still apply to this voltage.
                    hoodVolts = hoodJogVolts;
                    setDesired_Angle(currentHoodAngle);
                    hoodHolding = true;
                    shooterAnglePID.reset();
                } else if (HOOD_CLOSED_LOOP_ENABLED && hoodFeedbackValid) {
                    // SETTLE BAND (2026-08-22, team requirement: a DPAD click moves the hood
                    // 2 deg and it STAYS there). Once inside ANGLE_TOLERANCE the hood is driven
                    // with 0 V and the brake idle mode holds it; without this it HUNTS -- it
                    // breaks free at ~7.5 V, carries past the target, and the loop drives it
                    // back down with up to 6 V plus gravity, a limit cycle around the setpoint.
                    // Latching with a wider re-engage threshold (hysteresis) so the loop cannot
                    // chatter on and off at the band edge -- see hoodShouldHold().
                    hoodHolding = hoodShouldHold(hoodTarget - currentHoodAngle, hoodHolding);
                    double anglePID = hoodHolding
                        ? 0
                        : shooterAnglePID.calculate(currentHoodAngle, hoodTarget);
                    // Gravity LIFT feedforward: the pure-P loop under-drives against gravity at modest
                    // errors (a 20 deg error only asks 5.5 V, below the ~7 V breakaway), so add an
                    // up-bias while the hood is still BELOW its target. FADED IN across
                    // HOOD_FF_FADE_DEG rather than stepped at ANGLE_TOLERANCE (2026-08-22) -- the step
                    // made the DPAD jog lurch. Still 0 V at/inside the target, so the hood can never be
                    // driven PAST the target into the MAX_ANGLE belt-skip zone by this term, and still
                    // off while lowering (error <= 0), so the gravity-assisted descent stays gentle.
                    double liftFF = hoodHolding ? 0 : hoodLiftFeedforward(hoodTarget - currentHoodAngle);
                    // ASYMMETRIC speed cap (2026-07-18): raising fights gravity, lowering is gravity-
                    // assisted, so the UP stroke gets more voltage (HOOD_MAX_UP_VOLTAGE) than the DOWN
                    // stroke (HOOD_MAX_DOWN_VOLTAGE). A symmetric 6 V could not lift the hood at all
                    // (it sat fully down during the feed shot). Positive output raises the hood, so it
                    // is capped at +UP; negative lowers, capped at -DOWN.
                    hoodVolts = MathUtil.clamp(anglePID + liftFF, -ShooterSubsystemConstants.HOOD_MAX_DOWN_VOLTAGE, ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE);
                }
                // HARD travel guard (team request 2026-07-18: "by no means exceed the max angle"
                // -- the hood was over-extending and skipping the belt). Positive volts raise the
                // hood: once at/above MAX_ANGLE never drive UP, once at/below the enable-time floor
                // (hoodFloorAngle -- the datum, re-read from the raw encoder every enable) never
                // drive DOWN, whatever the PID or the DPAD jog asks. MAX_ANGLE sits ~6.5 deg below the
                // physical 44.5 stop, so a little coast after the cut never reaches the stop /
                // skips the belt. Outside the branches above so it bounds BOTH the closed loop
                // and the open-loop jog.
                if (currentHoodAngle >= ShooterSubsystemConstants.MAX_ANGLE && hoodVolts > 0) {
                    hoodVolts = 0;
                }
                if (currentHoodAngle <= hoodFloorAngle(hoodBaseAngleDeg, hoodDatumValid) && hoodVolts < 0) {
                    hoodVolts = 0;
                }
                shooterAngle.setVoltage(hoodVolts);
                lastHoodVolts = hoodVolts;
            //At-speed / at-angle kicker gates -- instance state (the mutable statics in
            // ShooterSubsystemConstants are DELETED; the hopper polls isReadyToShoot()).
                velReached =
                    desired_Velocity != 0 ? Math.abs(desired_Velocity-getShooterFlywheelVelocity()) < ShooterSubsystemConstants.SPEED_TOLERANCE : false;
                angleReached =
                     desired_Angle != 0 ? Math.abs(desired_Angle-getShooterAngleDegrees()) < ShooterSubsystemConstants.ANGLE_TOLERANCE : false;
            //Data. The six gain entries are INPUTS ONLY -- they are seeded once at construction
            // from the constants and never written again. They used to be echoed here every
            // loop, a few lines above the block that READS them back, so a typed value survived
            // only if it landed in the gap between the echo and the read in the same loop:
            // dashboard tuning appeared to work, then silently reverted. Same fix the Desired
            // Velocity/Angle entries already got. The gains still revert to the constants on
            // reboot or redeploy -- write a keeper into SubsystemConstants.
                debugEntry.setBoolean(enableComp);

            //Physics Lab: live-data mode reads setpoints straight off Shuffleboard. No else --
            // zeroing on exit happens ONCE in enableLiveData(false); an every-loop else here
            // would stomp the shot commands (visionShotCommand/feedAngleShotCommand) every loop.
                if(enableComp){
                    desired_Velocity = desiredVelEntry.getDouble(0);
                    desired_Angle = desiredAngleEntry.getDouble(0);
                }

            //PID + FF Tuning -- gains come from the driver-companion PID panel over NT (frc.robot.util.Tunable).
            // No-ops entirely when SubsystemConstants.TUNING_MODE is false.
                //Speed
                    // Every hasChanged() is evaluated into a local FIRST: || short-circuits, so
                    // OR-ing the calls inline would leave later gains unpolled and their change
                    // consumed-but-unapplied on the next pass.
                    boolean kPChanged = tuneSpeedKp.hasChanged();
                    boolean kIChanged = tuneSpeedKi.hasChanged();
                    boolean kDChanged = tuneSpeedKd.hasChanged();
                    boolean kSChanged = tuneSpeedKs.hasChanged();
                    boolean kVChanged = tuneSpeedKv.hasChanged();
                    boolean kAChanged = tuneSpeedKa.hasChanged();
                    if(kPChanged || kIChanged || kDChanged || kSChanged || kVChanged || kAChanged){
                        shooterVelConfigs.kP = tuneSpeedKp.get();
                        shooterVelConfigs.kI = tuneSpeedKi.get();
                        shooterVelConfigs.kD = tuneSpeedKd.get();
                        shooterVelConfigs.kS = tuneSpeedKs.get();
                        shooterVelConfigs.kV = tuneSpeedKv.get();
                        shooterVelConfigs.kA = tuneSpeedKa.get();
                        // Slot0Configs ONLY -- applying a whole TalonFXConfiguration here would
                        // reset the stator/supply limits and Coast neutral mode set in the ctor.
                        shooterA.getConfigurator().apply(shooterVelConfigs);
                    }
                //Angle
                    boolean angleKPChanged = tuneAngleKp.hasChanged();
                    boolean angleKIChanged = tuneAngleKi.hasChanged();
                    boolean angleKDChanged = tuneAngleKd.hasChanged();
                    if(angleKPChanged || angleKIChanged || angleKDChanged){
                        shooterAnglePID.setPID(tuneAngleKp.get(), tuneAngleKi.get(), tuneAngleKd.get());
                    }
            } else {
                // Subsystem disabled while the robot is still enabled: VelocityVoltage LATCHES on
                // the TalonFX, so just skipping the body would leave the flywheels spinning at the
                // last setpoint forever. Explicitly coast the flywheels and stop the hood (its
                // Brake idle mode holds the angle).
                shooterA.setControl(m_flywheelCoast);
                shooterAngle.setVoltage(0);
                lastHoodVolts = 0;
                desired_Velocity = 0;
                // Drop the kicker gates too: these flags latch the last enabled-loop value,
                // and the hopper's kicker fires on isReadyToShoot() -- a disable mid-shot must
                // never leave them true while the flywheels coast down.
                velReached = false;
                angleReached = false;
                hasShotTarget = false;
                aimHeadingValid = false;
            }

        //Data -- read-only status, published EVERY loop (enabled or not) so dashboards
        // never show stale state (the old enabled-only write froze "Subsystem State" /
        // "Desired Velocity Reached" at their boot values while disabled).
            currentVelEntry.setDouble(getShooterFlywheelVelocity());
            currentAngleEntry.setDouble(getShooterAngleDegrees());
            currentRpmEntry.setDouble(getShooterFlywheelVelocity() / surfaceSpeedForMotorRpm(1.0));
            targetDistanceEntry.setDouble(target_distance);
            targetClassEntry.setString(targetClass.name());
            shotSetpointEntry.setDouble(desired_Velocity);
            readyToShootEntry.setBoolean(isReadyToShoot());
            aimedEntry.setBoolean(isAimedAtTarget());
            movingCompEntry.setDouble(movingCompMeters);
            subsystemStateEntry.setBoolean(enableSubsystem);
            degreedToAlignToTargEntry.setDouble(degreesToAlignToTarget);
            desiredVelReachedEntry.setBoolean(velReached);
            desiredAngleReachedEntry.setBoolean(angleReached);
            rawHoodEncoderEntry.setDouble(shooterAngleEncoder.getPosition());
            hoodVoltsEntry.setDouble(lastHoodVolts);
            hoodCurrentEntry.setDouble(shooterAngle.getOutputCurrent());
            hoodBaseAngleEntry.setDouble(hoodBaseAngleDeg);
            hoodRelativeEntry.setDouble(hoodRelativeDeg);
            rtSpeedEntry.setDouble(rtSurfaceSpeed);
            rtSpeedRpmEntry.setDouble(rtSurfaceSpeed / surfaceSpeedForMotorRpm(1.0));
            // Desired Velocity/Angle double as INPUTS in live-data mode (enableComp reads
            // them back above) -- only echo the real setpoints when NOT in that mode, so a
            // dashboard edit is never stomped mid-tune.
            if (!enableComp) {
                desiredVelEntry.setDouble(desired_Velocity);
                desiredAngleEntry.setDouble(desired_Angle);
            }
    }

    // Read-only motor access for PowerTelemetry (no control).
    public TalonFX[] getFlywheelMotors(){
        return new TalonFX[] { shooterA, shooterB, shooterC, shooterD };
    }

    // Read-only motor access for PowerTelemetry (no control).
    public SparkMax getHoodMotor(){
        return shooterAngle;
    }
}
