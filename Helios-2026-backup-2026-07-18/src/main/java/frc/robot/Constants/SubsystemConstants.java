package frc.robot.Constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class SubsystemConstants {

        public static class LightSensor{
            public static final int INDEXER_SENSOR_ID_A = 8;
            public static final int INDEXER_SENSOR_ID_B = 9;
        }

        public static class Vision{
            public static final String CAM_LIMELIGHT = "limelight-knight";
                public static final double LL_X_MULTIPLIER = 1;
                public static final double LL_Y__MULTIPLIER = 1;
                public static final double LL_Z_MULTIPLIER = 1;

            // AprilTag pose correction (CommandSwerveDrivetrain.updateVisionPose), enabled by
            // team request 2026-07-17: MegaTag2 corrects X/Y continuously (yaw stays
            // Pigeon-owned) and MegaTag1 auto-seeds the heading/pose ("auto re-zero") behind
            // the quality gates below. PREREQUISITE -- the Limelight's camera mount pose MUST
            // be set in its web UI first: botpose is systematically wrong without it, and the
            // gates below cannot catch a consistent mount-offset error. TODO verify on robot
            // before first match; MENU re-zero is the manual recovery either way.
            public static final boolean ENABLE_MEGATAG2_POSE = true;

            // Auto-seed quality gates. Disabled robot (pre-match, sitting still): adopt the
            // full MT1 tag pose whenever it disagrees with odometry. Enabled (in-match):
            // yaw-only re-seed, and only from a multi-tag solve while nearly stationary --
            // a moving single-tag MT1 yaw is exactly the noise the Pigeon exists to reject.
            public static final double VISION_SEED_MAX_TAG_DIST_METERS = 5.0;  // ignore solves with avg tag distance beyond this [assumed] TODO tune
            public static final double VISION_SEED_POS_TOL_METERS = 0.25;      // disabled-mode reseed when position is off by more than this
            public static final double VISION_SEED_YAW_TOL_DEG = 3.0;          // reseed when heading is off by more than this
            public static final int    VISION_SEED_ENABLED_MIN_TAGS = 2;       // in-match yaw reseed requires a multi-tag solve
            public static final double VISION_SEED_MAX_SPEED_MPS = 0.3;        // in-match reseed only while nearly stationary
            public static final double VISION_SEED_MAX_YAW_RATE_DPS = 10.0;    // and not rotating

            // PathPlanner pathfinding constraints (driveToPose). Conservative starts -- the old
            // zeros meant pathfinding could not move at all. TODO tune on robot.
            public static final double PP_MAX_VELOCITY = 2.5;                       // m/s (robot max ~4.93)
            public static final double PP_MAX_ACCELERATION = 2.5;                   // m/s^2
            public static final double PP_MAX_ANGULAR_VELOCITY = Math.PI;           // rad/s (180 deg/s)
            public static final double PP_MAX_ANGULAR_ACCELERATION = 2 * Math.PI;   // rad/s^2

            // ---- Limelight thermal / power management (LimelightThermalManager) ----
            // HARDWARE REALITY, measured on THIS camera 2026-07-18 via its /status endpoint:
            // the Limelight 4's heat is dominated by its on-board Hailo AI accelerator -- a
            // FIXED ~3.75 W / ~74 C load that is powered whenever the camera runs and does NOT
            // respond to pipeline resolution, framerate, LED state, or CPU load (verified by
            // sweeping all of them: hailoTemp/hailoPower never moved off 74 C / 3.75 W, and it
            // is already 74 C ~2 s after a cold boot). The AprilTag pipeline does not even use
            // the Hailo (its neural runtimes are CPU), yet it stays hot.
            //   => Robot code CANNOT throttle the actual hot chip. This manager does what it
            //      CAN: (1) force LEDs OFF (the only camera power the RIO controls; passive
            //      AprilTags never need them); (2) drop to a lower-res "idle" AprilTag pipeline
            //      when not shooting -- shaves only the CPU/board heat and keeps low-res
            //      MegaTag2 pose fusion alive; (3) read the temps and WARN the driver.
            //   => The real fix for genuine overheating is PHYSICAL: confirm the camera's fan
            //      spins and improve enclosure airflow/heatsinking. Software only warns.
            //
            // Pipeline indices. NORMAL = full-res AprilTag (best shot ranging). IDLE = lower-
            // res AprilTag (still feeds pose fusion, a bit less CPU/board heat). IDLE MUST be an
            // AprilTag pipeline that EXISTS on the camera -- a non-fiducial or missing idle
            // pipeline blinds MegaTag2 when the shooter is idle. Set LL_PIPELINE_IDLE = 0 (same
            // as NORMAL) to DISABLE idle switching until pipeline 1 is created on the Limelight
            // (clone 0, LEDs off, lower resolution). TODO create/confirm pipeline 1 on robot.
            public static final int LL_PIPELINE_NORMAL = 0;
            // Set to 0 (== NORMAL) 2026-07-18 to DISABLE idle-pipeline switching. If pipeline 1
            // isn't a working AprilTag pipeline on this camera, the thermal manager was switching to
            // it whenever the shooter idled and BLINDING vision -- which is exactly the RT auto-aim
            // bootstrap state (flywheels stay off until a target is found), so auto-aim could never
            // acquire a tag (e.g. tag 18). Idle switching only sheds board/CPU heat (never the
            // dominant Hailo load), so disabling it costs ~nothing. Create pipeline 1 (clone 0,
            // lower res, LEDs off), confirm it exists, then set this back to 1 to restore idle
            // power-saving. TODO create/confirm pipeline 1 on robot.
            public static final int LL_PIPELINE_IDLE = 0;

            // The camera reports hailoTemp/temp only over HTTP (verified: the NT 'hw' array does
            // not carry the Hailo temp). Polled OFF the main loop by a Notifier; robotPeriodic
            // only reads cached values, so a slow/at camera never stalls the 20 ms loop.
            public static final int    LL_STATUS_PORT = 5807;
            public static final double LL_STATUS_POLL_SEC = 1.0;    // thermal changes are slow; 1 Hz is plenty
            public static final double LL_STATUS_TIMEOUT_SEC = 0.3; // per-request connect+read cap

            // Thermal thresholds on hailoTemp (deg C), hysteresis: engage idle at HOT, release
            // at COOL. WARN just alerts the driver. PLACEHOLDERS -- the LL4 Hailo-8 shutdown
            // point is not published and it idles ~74 C in open air (climbs in an enclosure);
            // tune these against the real temperature the camera reaches on the robot. TODO tune.
            public static final double LL_TEMP_WARN_C = 80.0;   // TODO tune: driver warning
            public static final double LL_TEMP_HOT_C  = 83.0;   // TODO tune: engage thermal idle
            public static final double LL_TEMP_COOL_C = 78.0;   // TODO tune: release thermal idle

            // Plausibility gate: only act on a reading inside a sane range, so a bad/missing
            // read can never force the idle pipeline (and degrade vision) on garbage data.
            public static final double LL_TEMP_SANE_MIN_C = 10.0;
            public static final double LL_TEMP_SANE_MAX_C = 120.0;

            // Hold the full pipeline this long after the shooter last went inactive, so rapid
            // spin-up/down does not churn the pipeline (each switch costs a few frames of no
            // targets) and the camera is already full-res the instant the next shot starts.
            public static final double LL_IDLE_HOLD_FULL_SEC = 3.0;  // TODO tune

            // Rate-limit the over-temp warning so a sustained event does not spam the DS console.
            public static final double LL_WARN_INTERVAL_SEC = 5.0;
        }

        public static class ClimbSubsystemConstants{
            public static final int CLIMBMOTOR_A = 0;
            public static final int CLIMBMOTOR_B = 0;

            //SPEED CONSTANTS
                public static final double CLIMB_MANUAL_SPEED = 0;
                public static final double CLIMB_MAX_VELOCITY = 0;
                public static final double CLIMB_MAX_ACCELERATION = 0;
            //PID - Climb
                public static double CLIMB_kP = 0;
                public static double CLIMB_kI = 0;
                public static double CLIMB_kD = 0;
            //FEEDFORWARD - Climb
                public static double CLIMB_kS = 0;
                public static double CLIMB_kG = 0;
                public static double CLIMB_kV = 0;
                public static double Climb_kA = 0;

            //HARDWARE CONSTANTS
                public static double CLIMB_WHEEL_RADIUS_INCHES = 0;
            
            //CLIMB HEIGHT CONSTRAINTS
                public static double MAX_INCHES = 0;
                public static double MIN_INCHES = 0;
        
        }

        public static class IntakeSubsystemConstants{
            public static final int INTAKE_MOTOR_ID = 18;
            public static final int INTAKE_PIVOT_MOTOR_ID = 22;

            //HARDWARE CONSTANTS

                // Slider remade 2026-07-14, dead simple: fixed SLOW duty, tiered smart limit
                // (Hardware-Data-Sheet sec.7 conservative start -- too weak to grind anything
                // at a hard stop), and TWO stop rules (either -> cut output, brake holds):
                //   a) encoder stopped turning = hard stop or jam;
                //   b) current pegged at the smart limit = stalled even though the motor is
                //      still spinning (belt/gear slip at full extension -- observed 2026-07-15:
                //      encoder-only detection missed it and the motor kept pushing).
                public static final double SLIDER_DUTY = 0.45;               // travel duty (+50% from 0.3 per team request 2026-07-16); TODO verify hard-stop impact on robot
                public static final double SLIDER_STALL_MOTOR_RPM = 300;    // motor rpm below this = "not moving" (raised from 100 so slow slip-creep at the hard stop also reads as stalled; free travel at 0.45 duty is ~2500 motor rpm); TODO tune on robot
                public static final double SLIDER_STALL_CURRENT_AMPS = 12;  // measured amps at/above this = stalled (smart limit clamps ~15 A at 0 rpm; free travel draws only a few amps) [assumed] TODO tune on robot
                public static final double SLIDER_CURRENT_STALL_DEBOUNCE_SEC = 0.05; // fast path: current needs no spin-up grace; ~2-3 loop cycles is the floor for a real signal
                public static final double SLIDER_STALL_DEBOUNCE_SEC = 0.15; // velocity path (was 0.25; tightened for faster cutoff 2026-07-16)
                public static final double SLIDER_GRACE_SEC = 0.3;          // covers spin-up from rest (encoder reads 0 rpm at move start; must outlast the 0.25 s ramp)
                public static final double SLIDER_MOVE_TIMEOUT_SEC = 2.5;   // backstop only (was 3.0; travel is ~33% shorter at 0.45 duty); TODO measure real travel time

            //SPEED CONSTANTS
                // Roller duty cycles (driver-tuned 2026-07-10: 35% intake was too fast;
                // outtake halved 2026-07-14 per team request). Worst-case steady supply draw
                // = duty x stator limit (40 A, set in IntakeSubsystem): intake 0.25 x 40 =
                // 10 A, outtake 0.3 x 40 = 12 A, both well under the 25 A supply limit.
                // Inrush softened by the duty-cycle ramp.
                public static final double INTAKE_SPEED = 0.25;
                public static final double OUTTAKE_SPEED = 0.3;

        }

        public static class HopperSubsystemConstants{
            public static final int HOPPER_ID_A = 20;
            public static final int HOPPER_ID_B = 21;
            // Kicker Victor SPX at CAN 17 -- VERIFIED LIVE on the bus 2026-07-16 via the
            // Phoenix diagnostics server (Victor SPX, fw 22.1, "Running Application").
            // Earlier "kicker not working" was NOT the ID: the kicker only fires when the
            // flywheels are at speed (ShooterSubsystem.isReadyToShoot), and the shooter was disabled.
            public static final int KICKER_MOTOR_ID = 17;
            
            //SPEED CONSTANTS
                public static final double INDEXER_SPEED = 0.75;
                // Belt duty. Conservative start (old code ran 0.8, never verified on robot);
                // direction test 2026-07-14 confirmed both motors agree, positive = tested
                // direction. TODO raise toward 0.8 once feed direction + throughput verified.
                public static final double HOPPER_SPEED = 0.5;
                // Reverse duty for the unjam button (belts + kicker together, held).
                public static final double UNJAM_SPEED = 0.3;
                // Slow duty for the (unbound) kicker-only test (bypasses the at-speed gate).
                public static final double KICKER_TEST_SPEED = 0.3;
                // Kicker duty while INTAKING (LT): the flywheels are STOPPED during intake,
                // so fuel pressed against them can stall the kicker CIM -- and the Victor
                // SPX has no current sensing, so this deliberately LOW duty is the only
                // thing keeping a momentary stall breaker-survivable (CIM stall at 12 V is
                // ~131 A; at 0.3 duty a stall draws roughly a third of that -- brief events
                // ride the breaker curve). [assumed] TODO tune on robot; keep it low.
                public static double INTAKE_KICKER_SPEED = 0.3;
        }

        public static class ShooterSubsystemConstants{
            // AprilTag sets moved to FieldConstants (FEED_TAGS / SCORE_TAGS_RED / SCORE_TAGS_BLUE,
            // team partition 2026-07-16) -- one source so the shooter and drivetrain can't drift.
            public static final int SHOOTER_ANGLE_ID = 19;
            public static final int SHOOTER_ID_A = 13;
            public static final int SHOOTER_ID_B = 14;
            public static final int SHOOTER_ID_C = 15;
            public static final int SHOOTER_ID_D = 16;
            //PID - Angle
                public static double SHOOTER_ANGLE_kP = 0.275;
                public static double SHOOTER_ANGLE_kI = 0.0;
                public static double SHOOTER_ANGLE_kD = 0.0;
            //PID - Speed
                public static double SHOOTER_SPEED_kP = 0.4;
                public static double SHOOTER_SPEED_kI = 0;
                public static double SHOOTER_SPEED_kD = 0.01;
            //FEEDFORWARD - Speed
                public static double SHOOTER_SPEED_kS = 0;
                public static double SHOOTER_SPEED_kV = 0.12625;
                public static double SHOOTER_SPEED_kA = 0;
            //HARDWARE CONSTANTS
                public static final double FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION = 1.5;
                    // Flywheel rotations per motor rotation.
                    // Gear ratio is 3:2 (3 flywheel rotations for every 2 Kraken rotations).
                // Hood ABSOLUTE-encoder calibration: raw getPosition() units <-> PHYSICAL hood
                // degrees, measured on-robot 2026-07-18 by hand-moving the hood to each HARD STOP
                // and reading "Hood Encoder Raw (rot)" on the Shooter tab:
                //   FULL-UP   hard stop = 113.458 raw = 44.5  deg physical
                //   FULL-DOWN hard stop = 191.040 raw = 3.224 deg physical
                // Raw DECREASES as the hood raises. getShooterAngleDegrees() linearly interpolates
                // between these two anchors. These describe the PHYSICAL STOPS and are INDEPENDENT of
                // the soft travel limits (MIN_ANGLE/MAX_ANGLE) -- so insetting those limits for
                // clearance never shifts this measurement scale. Replaced the old motor-rotation math
                // (offset 63, divisor 187.5*0.0317428): the absolute encoder reads ~113-191, NOT
                // [0,1) motor rotations, so that formula returned a near-constant ~-10.5 deg and
                // railed the hood. TODO re-verify the four anchors on robot.
                public static double HOOD_RAW_AT_FULL_UP   = 113.458;
                public static double HOOD_DEG_AT_FULL_UP   = 44.5;
                public static double HOOD_RAW_AT_FULL_DOWN = 191.040;
                public static double HOOD_DEG_AT_FULL_DOWN = 3.224;
            //SHOT MODEL (distance -> hood angle AND flywheel speed; reworked 2026-07-17 against
            // the OFFICIAL hub drawings -- GE-26300 sheet 4/4: level hex mouth, front lip
            // 72.000+-0.5 in, hex outside across-flats 41.932 in, NET back-board 58.41 in wide
            // rising to 120.36 in; manual 5.4/5.10.1: opening 41.7 in, FUEL 5.91 in / 0.203-0.227 kg.
            // The old policy (hood FIXED at 44.5 deg everywhere) was optimal only for a swish
            // through the mouth: its speed-error window is ~+-3%. The hub's back NET is a ~4 ft
            // backboard, so the robust shot FLATTENS with distance and banks in: clear the front
            // lip, meet the net 0.4-0.8 m above the mouth. Speed-error window opens to +-10-18%,
            // which is what actually absorbs this untuned robot's calibration error.
            // Tables seeded by quadratic-drag integration (Cd 0.5, m 0.215 kg, release 0.686 m,
            // ball/surface efficiency 0.50) -- generator + derivation: docs/shot-model/.
                // Ball release height above carpet. User-measured 2026-07-10: bottom of swerve
                // to the UNEXTENDED hood = 27 in. TODO re-measure to the actual ball exit point.
                public static final double SHOT_RELEASE_HEIGHT_METERS = Units.inchesToMeters(27);
                // One-knob speed trim for BOTH tables: real efficiency below the assumed 0.50
                // makes every shot short by the same factor -> raise this, don't re-type tables.
                // (RB blind-feed history suggests it may need ~1.1-1.2.) TODO tune FIRST on robot.
                public static double SHOT_SPEED_SCALE = 1.0;
                // SCORE: distance to own hub center (m) -> hood angle (deg) / flywheel SURFACE
                // speed (m/s). Angle falls 44.5 -> 31.5 deg as range grows (bank-shot policy).
                // Integrator seeds; every point is a TODO tune-on-robot knob (edit pairs freely,
                // InterpolatingDoubleTreeMap interpolates linearly and clamps at the ends).
                public static final InterpolatingDoubleTreeMap SCORE_HOOD_DEG_BY_DIST = new InterpolatingDoubleTreeMap();
                public static final InterpolatingDoubleTreeMap SCORE_SURFACE_MPS_BY_DIST = new InterpolatingDoubleTreeMap();
                // FEED: distance to the feed aim point (m) -> hood angle (deg) / surface speed
                // (m/s). Flatter than score (40 -> 33 deg): min-energy ground lob is ~40-42 deg,
                // flatter cuts hang time and speed sensitivity (team saw 44.5-deg feeds arc too
                // high, 2026-07-17). Integrator seeds; TODO tune on robot.
                public static final InterpolatingDoubleTreeMap FEED_HOOD_DEG_BY_DIST = new InterpolatingDoubleTreeMap();
                public static final InterpolatingDoubleTreeMap FEED_SURFACE_MPS_BY_DIST = new InterpolatingDoubleTreeMap();
                static {
                    SCORE_HOOD_DEG_BY_DIST.put(3.0, 44.5);   SCORE_SURFACE_MPS_BY_DIST.put(3.0, 16.6);
                    SCORE_HOOD_DEG_BY_DIST.put(3.5, 44.5);   SCORE_SURFACE_MPS_BY_DIST.put(3.5, 17.2);
                    SCORE_HOOD_DEG_BY_DIST.put(4.0, 41.5);   SCORE_SURFACE_MPS_BY_DIST.put(4.0, 18.4);
                    SCORE_HOOD_DEG_BY_DIST.put(4.5, 38.5);   SCORE_SURFACE_MPS_BY_DIST.put(4.5, 19.9);
                    SCORE_HOOD_DEG_BY_DIST.put(5.0, 36.0);   SCORE_SURFACE_MPS_BY_DIST.put(5.0, 21.3);
                    SCORE_HOOD_DEG_BY_DIST.put(5.5, 34.0);   SCORE_SURFACE_MPS_BY_DIST.put(5.5, 22.8);
                    SCORE_HOOD_DEG_BY_DIST.put(6.2, 31.5);   SCORE_SURFACE_MPS_BY_DIST.put(6.2, 24.9);

                    FEED_HOOD_DEG_BY_DIST.put(4.0, 40.0);    FEED_SURFACE_MPS_BY_DIST.put(4.0, 12.0);
                    FEED_HOOD_DEG_BY_DIST.put(5.0, 38.0);    FEED_SURFACE_MPS_BY_DIST.put(5.0, 13.8);
                    FEED_HOOD_DEG_BY_DIST.put(6.0, 36.5);    FEED_SURFACE_MPS_BY_DIST.put(6.0, 15.5);
                    FEED_HOOD_DEG_BY_DIST.put(7.0, 35.0);    FEED_SURFACE_MPS_BY_DIST.put(7.0, 17.1);
                    FEED_HOOD_DEG_BY_DIST.put(8.0, 34.0);    FEED_SURFACE_MPS_BY_DIST.put(8.0, 18.7);
                    FEED_HOOD_DEG_BY_DIST.put(9.0, 33.0);    FEED_SURFACE_MPS_BY_DIST.put(9.0, 20.3);
                }
                // SCORE envelope to the own hub center (m). Below 3.0 the ball can't fit through
                // the opening within the speed/angle tolerance budget (physical floor 2.63 m;
                // dead zone 1.0-3.0 m is real -- drivers must back off ~2 m from the hub face).
                // Above 6.2 the robot cannot legally be in its zone (G407 geometric limit 6.14 m).
                // Outside the envelope the model REFUSES (velocity 0 -> no belts, no kicker).
                public static double MIN_SCORE_DISTANCE_METERS = 3.0;   // [derived] TODO tune on robot
                public static final double MAX_SCORE_DISTANCE_METERS = 6.2;
                // FEED distance clamp (m): outside 4-9 command the clamped endpoint instead of
                // refusing -- a slightly-short lob still lands in friendly territory. [assumed]
                public static double MIN_FEED_DISTANCE_METERS = 4.0;
                public static double MAX_FEED_DISTANCE_METERS = 9.0;
                // RB blind feed: flywheel SURFACE speed for a ~6.4 m lob (typical blind-feed
                // positions span 5.5-7 m -> +-0.8 m landing scatter, fine for a bulk feed).
                // Raised 16 -> 22 m/s 2026-07-17 (practice-match: the shot "barely left the robot").
                // 22 m/s = ~46 motor rps = 48% of the 95 rps ceiling (5.8 V FF of 12) -- ample motor
                // headroom, and paired with the flywheel STATOR raise (ShooterSubsystem) so the wheel
                // can actually hold it under ball load. STARTING POINT -- retest and dial in: if it now
                // overshoots the feed zone, drop it; if still weak, nudge up AND check flywheel stator
                // amps in PowerTelemetry (pegged at 80 = still torque-limited, not speed-limited).
                // [assumed] TODO tune vs actual feed positions on robot.
                // Cut 22 -> 19.8 m/s (-10%) 2026-07-17: shot ran too fast / arced too high.
                public static double RB_FEED_SURFACE_SPEED = 19.8;   // m/s surface
                // RB blind feed hood angle (deg) -- a FIXED angle (team request). Set to 25 deg
                // 2026-07-18: the hood sat FULLY DOWN (~5 deg stow) during the feed shot; team asked
                // for it to "extend up 20 degrees", i.e. ~20 deg above the stow -> 25 deg. This is a
                // clean, low target well clear of both the MAX_ANGLE (38) hard-travel guard and the
                // 30 deg stow interlock floor, so the hood servos straight up to it with no gating.
                // NOTE: reaching it also needed more UP torque -- see HOOD_MAX_UP_VOLTAGE (6 V could
                // not lift the hood against gravity). MUST stay < MAX_ANGLE. TODO retune
                // RB_FEED_SURFACE_SPEED for this flatter 25 deg lob on robot.
                public static double RB_FEED_ANGLE = 25.0;           // deg (must be < MAX_ANGLE)
                // Time-of-flight linear fits (s) for moving-shot compensation, refit 2026-07-17
                // to the VARYING-angle tables (flatter far shots barely raise ToF: score stays
                // ~0.55-0.65 s across the whole envelope, which also shrinks moving-comp error).
                public static double SCORE_TOF_BASE_SEC = 0.493;
                public static double SCORE_TOF_SEC_PER_METER = 0.025;
                public static double FEED_TOF_BASE_SEC = 0.714;
                public static double FEED_TOF_SEC_PER_METER = 0.055;
                // Moving-shot radial compensation (spec 5): d_eff = d + gain * v_radial * ToF,
                // two fixed-point iterations, clamped. Gain 0 disables while tuning everything
                // else. Beyond +-1.5 m the robot moves >~1.9 m/s and the shot shouldn't be
                // trusted; the clamp also keeps d_eff from jumping the envelope gate erratically.
                public static double MOVING_COMP_GAIN = 1.0;             // [assumed, tunable; 0 disables]
                public static double MOVING_COMP_MAX_METERS = 1.5;
                // RT tag-flicker ride-through (auto-aim, 2026-07-17): after the last real tag
                // classification, the firing solution stays live this long on the drivetrain's
                // fused pose before RT refuses. Initial acquisition still needs a real tag.
                public static double TARGET_HOLD_SEC = 0.5;              // s [assumed] TODO tune on robot
                // VelocityVoltage ceiling: ~12 V / 0.12625 kV = 95 motor rps sustainable.
                public static final double SHOT_MAX_MOTOR_RPS = 95.0;
            //SHOOTER CONSTRAINTS
                // SOFT hood travel limits (deg) -- the commandable range, deliberately INSET from
                // the physical hard stops (44.5 / 3.224, see HOOD_*_AT_FULL_* calibration) so the
                // hood settles well OFF each stop instead of jamming it / skipping the drive belt
                // (team request 2026-07-18). MAX dropped 44.5 -> 43 -> 38 (a further -5 deg of top
                // leeway after the hood was seen over-extending and skipping the belt); that leaves
                // ~6.5 deg clearance below the top stop. A HARD guard in periodic() also cuts upward
                // drive at MAX_ANGLE so it is never exceeded. Lower MAX / raise MIN for more room;
                // never exceed the stops. NOTE: the SCORE table asks for up to 44.5 deg at close
                // range, so close shots now clamp hard to MAX_ANGLE (38) and will land flatter until
                // MAX is raised -- raise it toward the stop only once overshoot is controlled and
                // ballistics are tuned. TODO tune the margin + close-shot angle on robot.
                public static double MAX_ANGLE = 38.0;   // was 44.5 (hard stop), then 43; -5 for belt-skip leeway
                public static double MIN_ANGLE = 5.0;    // was 3.224 (hard stop)
                // Hood drive voltage caps -- ASYMMETRIC (2026-07-18). Gravity always pulls the hood
                // toward the DOWN stop, so raising fights gravity and lowering is gravity-assisted.
                // The old symmetric 6 V cap was gentle enough for the DOWN stroke but too weak to
                // LIFT the hood at all (it sat fully down during the feed shot). So the angle PID
                // output is now clamped to [-DOWN, +UP]:
                //   UP  (positive volts raise): 10 V -- enough to overcome gravity + friction and
                //       servo up. Still 2 V below the original 12 V that over-extended, and the hard
                //       MAX_ANGLE guard in periodic() (plus the ~6.5 deg inset below the 44.5 stop
                //       and Brake idle) still catch any coast, so belt-skip risk stays bounded.
                //   DOWN (negative volts lower): 6 V -- kept gentle; gravity assists, so this is
                //       plenty for a controlled auto-descend and keeps the "50% slower" feel on the
                //       stroke where over-travel into the bottom would be roughest.
                // TODO tune on robot: raise UP toward 12 if the lift is still sluggish; LOWER it if
                // high-angle SCORE shots over-extend past MAX_ANGLE and skip the belt.
                // (If overshoot ever forces UP back down, a gravity feedforward kG is the cleaner
                // long-term fix -- lets a low cap still lift. Not added: kG needs on-robot tuning.)
                public static double HOOD_MAX_UP_VOLTAGE = 10.0;   // raising (fights gravity)
                public static double HOOD_MAX_DOWN_VOLTAGE = 6.0;  // lowering (gravity-assisted, gentle)
                // Hood gravity LIFT feedforward (volts), added to the angle-PID output ONLY while the
                // hood is still below its target (raising). The hood loop is pure-P (kP 0.275), so at a
                // modest error it under-drives: e.g. a 20 deg error asks only 0.275*20 = 5.5 V, below
                // the ~7 V breakaway needed to lift the hood against gravity -- which is exactly why it
                // "sat fully down" at the 25 deg feed target. This constant bias breaks it away and
                // cancels the steady-state gravity droop. It is GATED off once the hood is within
                // ANGLE_TOLERANCE of the target (see ShooterSubsystem periodic), so it can NEVER push
                // the hood past its target into the MAX_ANGLE belt-skip zone -- the hard MAX guard
                // still backstops the top either way. Only applies when raising; during the gravity-
                // assisted descent it is off, so auto-descend stays gentle.
                // STARTING VALUE -- breakaway measured ~7 V, moving-sustain is less; 5 V + the PID term
                // clears breakaway from the stow (5.5+5 = 10.5 -> capped 10) and sustains the creep in
                // near the target. TODO tune on robot: raise toward 7 if the hood won't start from
                // small errors; LOWER it if high-angle SCORE shots reach MAX_ANGLE too hard.
                public static double HOOD_RAISE_FF_VOLTS = 5.0;
                // Surface-speed gate for the kicker, m/s. TIGHTENED 0.3 -> 0.2 (spec 6): at the
                // 3.0 m minimum, 0.3 m/s maps to 0.30 m of along-track error -- more than the
                // 0.226 m half-window through the opening; 0.2 closes the budget exactly.
                public static double SPEED_TOLERANCE = 0.2;
                public static double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);
                // Hood-lower interlock (team request 2026-07-17): the hood may only be driven DOWN
                // when the flywheel SURFACE speed is at/below this -- above it the hood holds its
                // current angle and waits for the flywheels to coast down (raising is never gated).
                // Low value = "nearly stopped". [assumed] TODO tune on robot: raise if the hood
                // should start dropping sooner, lower it toward 0 to wait for a fuller stop.
                public static double HOOD_LOWER_MAX_SURFACE_SPEED = 2.0;   // m/s surface
                // Interlock SCOPE (added with the varying-angle tables 2026-07-17): the gate above
                // now only blocks drops BELOW this angle (the stow region). Tracking WITHIN the
                // shot band (31.5-44.5 deg, both fully deployed positions) stays live while the
                // wheels spin -- otherwise a far shot staged steep could never lower to its firing
                // angle (spin-up outruns the hood and the old whole-range gate deadlocked it).
                public static double HOOD_STOW_INTERLOCK_FLOOR_DEG = 30.0;
                // AprilTag QUALITY GATES for the shot's botpose distance source (2026-07-17):
                // reject solves with the average tag farther than this (pose noise grows ~d^2;
                // max legal shot puts the hub tag ~5.7 m out) or taken while spinning faster
                // than this (rolling-shutter smear; matches the drivetrain's MegaTag2 gate).
                public static double SHOT_MAX_TAG_DIST_METERS = 7.0;      // [assumed] TODO tune
                public static double SHOT_MAX_YAW_RATE_DPS = 360.0;      // [assumed] TODO tune
                // Hood gate, deg: worst contribution 0.078 m at 3 m, shrinking with distance.
                public static double ANGLE_TOLERANCE = 0.5;
        }
}