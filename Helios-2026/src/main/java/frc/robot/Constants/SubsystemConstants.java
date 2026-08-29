package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class SubsystemConstants {

        /**
         * Master gate for live PID tuning from the driver-companion app.
         *
         * <p>MUST BE FALSE FOR COMPETITION. When false, frc.robot.util.Tunable publishes nothing,
         * subscribes to nothing, and every gain is the compiled constant below -- there is no path
         * for a dashboard value to reach a motor controller. Flip to true and redeploy once at the
         * start of a tuning session, then flip it back.
         */
        public static final boolean TUNING_MODE = true;

        public static class DriveConstants{
            // Heading servo (SwerveRequest.FieldCentricFacingAngle) gains, shared by every
            // aim/align command -- rotateToAngle, searchAndAlignCommand, aimUntilAligned. These
            // were three duplicated 5.0 literals in CommandSwerveDrivetrain; one copy so there is
            // a single thing to tune. Radians in, rad/s out. TODO tune on robot.
            public static double HEADING_kP = 5.0;
            public static double HEADING_kI = 0.0;
            public static double HEADING_kD = 0.0;
        }

        public static class LightSensor{
            // Indexer beam-breaks -- DECLARED, NOT YET WIRED IN CODE (data sheet sec.11):
            // physical wiring + active-high/low polarity unconfirmed. Construct
            // DigitalInput(8/9) only after the team verifies both on the robot; the old
            // Sensors stub that faked "always detected" was deleted 2026-07-21.
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
            // Pigeon-owned) and MegaTag1 seeds the full pose ONLY until the first enable
            // after boot. The in-match auto re-seed was REMOVED 2026-07-21 by team request:
            // once the match starts, field centering happens ONLY via the driver's MENU
            // press. PREREQUISITE -- the Limelight's camera mount pose MUST be set in its
            // web UI first: botpose is systematically wrong without it, and the gates below
            // cannot catch a consistent mount-offset error. TODO verify on robot before
            // first match.
            public static final boolean ENABLE_MEGATAG2_POSE = true;

            // Boot-seed quality gates (robot sitting still pre-match, never yet enabled):
            // adopt the full MT1 tag pose whenever it disagrees with odometry.
            public static final double VISION_SEED_MAX_TAG_DIST_METERS = 5.0;  // ignore solves with avg tag distance beyond this [assumed] TODO tune
            public static final double VISION_SEED_POS_TOL_METERS = 0.25;      // reseed when position is off by more than this
            public static final double VISION_SEED_YAW_TOL_DEG = 3.0;          // reseed when heading is off by more than this

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
                // = duty x stator limit (40 A, set in IntakeSubsystem): intake 0.2875 x 40 =
                // 11.5 A, outtake 0.345 x 40 = 13.8 A, both well under the 25 A supply limit.
                // Inrush softened by the duty-cycle ramp.
                public static final double INTAKE_SPEED = 0.2875;   // +15% team request 2026-08-24
                public static final double OUTTAKE_SPEED = 0.345;   // +15% team request 2026-08-24

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
                //                                                                                                                                                                          Belt duty. Conservative start (old code ran 0.8, never verified on robot);
                // direction test 2026-07-14 confirmed both motors agree, positive = tested
                // direction. TODO raise toward 0.8 once feed direction + throughput verified.
                public static final double HOPPER_SPEED = 0.6;   // +20% team request 2026-08-24
                // Reverse duty for the unjam button (belts + kicker together, held).
                public static final double UNJAM_SPEED = 0.3;
                // Slow duty for the (unbound) kicker-only test (bypasses the at-speed gate).
                public static final double KICKER_TEST_SPEED = 0.3;
                // Slow duty for the single-belt direction tests (VIEW / DPAD-DOWN): slow enough
                // that even a full stall sits well under the 20 A smart limit.
                public static final double DIRECTION_TEST_SPEED = 0.05;
                // (INTAKE_KICKER_SPEED removed 2026-07-18: team spec "during intake only the
                // hopper runs, not the kicker" -- intakeFeedCommand no longer touches the kicker.
                // The stall analysis it carried still applies to ANY kicker use against stopped
                // flywheels: CIM stall at 12 V is ~131 A and the Victor SPX has no current
                // sensing, so duty is the only software knob -- see manualRunCommand.)
                
                // Some variable that AI made, change value to make it work.
                // Back to 2.0 s (team request 2026-08-24): this is now BOTH the delay before the
                // kicker feeds forward AND the window it spends running in REVERSE while the
                // flywheels wind up.
                public static final double KICKER_SPINUP_DELAY_SEC = 2.0;
                public static final int HOPPER_CURRENT_LIMIT_A = 5;
                public static final int HOPPER_SHOT_CURRENT_LIMIT_A = (int)(HOPPER_CURRENT_LIMIT_A * 1.2);
                // Belt limit while B (manual hopper run) is held -- team request 2026-08-27
                // (30 -> 60), lowered to 40 on 2026-08-28. 40 A sits at/below the branch
                // breaker, so the software limit now acts before the breaker does, and stays
                // under the data-sheet 60 A cap for a hopper NEO 2.0.
                public static final int HOPPER_MANUAL_CURRENT_LIMIT_A = 40;
        }

        public static class ShooterSubsystemConstants{
            // AprilTag sets moved to FieldConstants (FEED_TAGS / SCORE_TAGS_RED / SCORE_TAGS_BLUE,
            // team partition 2026-07-16) -- one source so the shooter and drivetrain can't drift.
            public static final int SHOOTER_ANGLE_ID = 19;
            public static final int SHOOTER_ID_A = 13;
            public static final int SHOOTER_ID_B = 14;
            public static final int SHOOTER_ID_C = 15;
            public static final int SHOOTER_ID_D = 16;
            // HOOD ANGLE UNITS (2026-08-27, team request: "make the hood angle the exact same as
            // the raw encoder angle"). The tracked hood angle is now the RAW ENCODER READING --
            // 1 hood unit = 1 raw encoder unit, and the enable-time datum is the raw value itself
            // (see ShooterSubsystem.captureHoodDatum). Every hood band and gain below is therefore
            // in RAW UNITS, not physical degrees; this factor converts the on-robot tuning
            // (which was measured in physical degrees) into them, so the physical behaviour of
            // the loop is UNCHANGED -- only the numbers it counts in are.
            // 309.6 raw units / 41.276 deg of travel = 7.5 units per physical degree.
            // Declared HERE, ahead of the gains, because Java static initializers run in textual
            // order and the gains divide by it. It mirrors
            //   HOOD_RAW_UNITS_PER_FULL_TRAVEL / (HOOD_DEG_AT_FULL_UP - HOOD_DEG_AT_FULL_DOWN)
            // declared further down -- HoodTrackingTest pins the two together, so an edit to one
            // that misses the other fails the build.
            // NOTE: MIN_ANGLE / MAX_ANGLE stay PHYSICAL degrees -- they belong to the shot model
            // (modelSurfaceSpeed fits at 38 deg), not to the hood tracker. The hood's own travel
            // limits are the enable-time floor and HOOD_TRAVEL_WINDOW_DEG above it.
            public static double HOOD_UNITS_PER_DEG = 309.6 / (44.5 - 3.224);
            //PID - Angle (RAW UNITS, see HOOD_UNITS_PER_DEG). THE HOOD IS NOW POSITION-CONTROLLED
            // ONLY (2026-08-28, team direction: "the DPAD should be based on PID, voltage
            // throttle removed"): these gains, plus hoodLiftFeedforward, are the ONLY thing that
            // moves the hood. There is no open-loop jog path left.
            //
            // BASE GAINS, chosen to minimise oscillation given a hood that needs ~7.5 V just to
            // break away from stiction:
            //  kP 0.18 -- the feedforward ramp supplies ~5.83 V at a one-step (10 unit) error,
            //    and the hood does not move below ~7.5 V, so P must find the remaining ~1.7 V:
            //    0.18 x 10 = 1.8 V, total 7.63 V, just over breakaway and just under the 8 V cap.
            //    Below ~0.167 a single DPAD press cannot break the hood away at all; above ~0.25
            //    the approach carries enough surplus to bounce out the far side of the band.
            //    (HoodSettleTest pins the one-step sum against breakaway and the cap.)
            //  kI 0 -- DELIBERATE, do not add. An integrator sitting against stiction winds up
            //    for as long as the hood refuses to move, then dumps it all at once; that is a
            //    lurch, not a correction. The gravity feedforward already does the job kI would.
            //  kD 0.004 -- THE ANTI-OSCILLATION TERM, and the one to tune first. At the observed
            //    approach rate it subtracts ~1.5-2 V near the target, dropping the total below
            //    breakaway a few units EARLY so the hood coasts the last bit instead of arriving
            //    with surplus and bouncing out the far side of the settle band. TODO on robot:
            //    if the hood still hunts after a press, raise kD in 0.002 steps before touching
            //    kP; if presses become sluggish or the hood stops short, lower it.
            // Every value is live-tunable from the Shooter tab (see the tune* entries) and
            // reverts to these on redeploy.
                public static double SHOOTER_ANGLE_kP = 0.6;
                public static double SHOOTER_ANGLE_kI = 0.0;
                public static double SHOOTER_ANGLE_kD = 0.000;
            //PID - Speed
                public static double SHOOTER_SPEED_kP = 0.2;
                public static double SHOOTER_SPEED_kI = 0.0;
                public static double SHOOTER_SPEED_kD = 0.01;
            //FEEDFORWARD - Speed
                public static double SHOOTER_SPEED_kS = 0.0;
                public static double SHOOTER_SPEED_kV = 0.12625;
                public static double SHOOTER_SPEED_kA = 0.0;
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
                // RE-ANCHORED 2026-08-24 from two NT-logged hand sweeps at 50 Hz (robot disabled;
                // see the 2026-08-24 hood-calibration note). The 08-22 anchors (415.3 / 151.7)
                // are DEAD, and they were the bug: they put HOOD_RAW_WRAP_SPLIT at 103.5, which
                // sits INSIDE live travel. Mid-stroke the map then jumped ~41 deg across one
                // count, so the tracked angle saturated at HOOD_DEG_AT_FULL_UP barely half-way
                // up; the travel guard (currentHoodAngle >= MAX_ANGLE) then killed ALL upward
                // voltage, and the hood moved once on a DPAD click and never again.
                //
                // MEASURED (raw values here are UNWRAPPED -- see HOOD_RAW_WRAP_SPLIT):
                //   FULL-DOWN hard stop = 360.0 unwrapped raw = 3.224 deg physical
                //     (sweep 1: 360.34 over 1615 samples, spread 0.15; sweep 2: 359.70 over 767.
                //      The bottom rest straddles the encoder's 0/360 rollover and varies ~0.7
                //      units between settles; 360.0 splits the two, 0.05 deg from either.)
                //   FULL-UP   hard stop = 650.4 unwrapped raw = 44.5  deg physical
                //     (two independent PUSHES agree: 650.82 peak, then 650.08 held 1.2 s. On
                //      release the hood falls back to ~639.5 -- that is backlash off the stop,
                //      not the stop. Anchors are the PUSHED stops, matching the degree values.)
                // Scale: 290.4 raw units / 41.276 deg = 7.036 units per hood degree.
                // The DEGREE values are unchanged -- they describe the physical stops, which did
                // not move. TODO on-robot: confirm the hood really reaches 3.224 / 44.5 at these
                // stops; if the physical travel was ever re-shimmed these degrees are stale too.
                //
                // TODO (root cause of the repeat re-anchoring): three measurements have now given
                // three different spans for the SAME travel -- 77.6, then 263.6, then 290.4 raw
                // units. The absolute encoder's positionConversionFactor is never set in code and
                // ShooterSubsystem configures the hood with kNoResetSafeParameters specifically to
                // keep whatever was flashed, so anyone opening the REV Hardware Client silently
                // re-scales the hood and these anchors go stale again. Pin it in code to stop this.
                // The RAW ANCHORS ARE GONE (2026-08-27). They were absolute POSITIONS, and the
                // encoder's zero offset moves: live capture found the hood resting at raw 311.169,
                // inside the arc the anchors said travel never occupies, so the map read the bottom
                // rest as the TOP stop (base 44.5 deg) and the travel guard killed every upward
                // volt -- the DPAD was dead. Nothing maps a raw reading to an angle any more; raw is
                // used ONLY for frame-to-frame DELTAS, which no offset can affect. What survives is
                // the SCALE (how many raw units the hood's whole travel spans) and the two PHYSICAL
                // stop angles, neither of which moves when the encoder re-zeroes.
                public static double HOOD_DEG_AT_FULL_UP   = 44.5;
                public static double HOOD_DEG_AT_FULL_DOWN = 3.224;
                // Raw units spanned by the hood's ENTIRE travel, bottom stop to top stop. A
                // DISTANCE, not a position -- an encoder that re-zeroes does not change it.
                //
                // DERIVED FROM GEOMETRY, not from a hand sweep (2026-08-27): the encoder is belted
                // 1:1 to the 24T pinion (team-confirmed pulley), and the 24T -> 180T sector is
                // 7.5:1 to the hood, so the whole 41.276 deg stroke is 41.276 x 7.5 = 309.57 raw
                // units. Geometry wins here because hand sweeps cannot agree: four stop-to-stop
                // sweeps have given 77.6, 263.6, 290.4, and (two on 08-27) 328 and 347. Pressing a
                // hood against its hard stop by hand adds compliance and backlash, which inflates
                // the span -- the 08-27 pair bracket the geometric value from above, and the two
                // top-of-stroke readings differed by 15 units (~2 deg) on their own.
                //
                // The old 290.4 made the code over-report travel by ~6.6%, so the hood stopped
                // short of every commanded angle (a commanded 30 deg moved ~28).
                //
                // TODO on robot: this is the one hood number a REV Hardware Client edit can still
                // invalidate (positionConversionFactor is never pinned in code). If the hood tracks
                // at the wrong RATE, re-measure; the encoder's offset no longer matters at all.
                public static double HOOD_RAW_UNITS_PER_FULL_TRAVEL = 309.6;
                // CONTINUOUS TRACKING (2026-08-22). The absolute reading cannot be trusted on its
                // own: the encoder drifts against the hood over a session (the hood physically
                // cannot pass its up stop at raw ~55-71, yet readings wander toward the split),
                // and interpreting one reading through a FIXED split is catastrophic near it --
                // raw 103.4 reads +52.0 deg, raw 103.6 reads -4.3 deg, a 56 deg flip across one
                // count. At -4.3 the travel guard thinks the hood is at the bottom, so it drives
                // UP at full feedforward: the "hood shoots up and cannot be controlled" symptom.
                //
                // So the angle is now tracked RELATIVELY: capture a datum when the robot enables,
                // then accumulate shortest-path deltas. A 359 -> 0 step is a small delta, never a
                // flip, and slow drift cannot move the hood because only CHANGES are counted.
                //
                // Hood units per raw unit -- 1.0 (2026-08-27, team request: the hood angle IS the
                // raw encoder angle). It used to convert raw units into physical degrees
                // (~0.1421); the physical scale now lives in HOOD_UNITS_PER_DEG at the top of this
                // class, where it converts the degree-tuned bands and gains INTO these units.
                // Set this back to (HOOD_DEG_AT_FULL_UP - HOOD_DEG_AT_FULL_DOWN) /
                // HOOD_RAW_UNITS_PER_FULL_TRAVEL to return the tracker to physical degrees.
                public static double HOOD_DEG_PER_RAW_UNIT = 1.0;
                // Ignore deltas smaller than this (raw units): measured encoder noise is +-0.08,
                // and noise must never accumulate into phantom travel. 0.2 clears it with margin
                // and costs 0.03 deg of resolution -- far below the 0.5 deg angle tolerance.
                public static double HOOD_RAW_NOISE_DEADBAND = 0.2;
                // Reject deltas bigger than this (raw units) as glitches, NOT motion.
                //
                // RAISED 20 -> 120 (2026-08-27) after it caused a belt grind. Under POWER the
                // encoder moves 25-55 raw units per 20 ms loop (measured), so a 20-unit ceiling
                // discarded EVERY delta of a real move: the tracked angle froze at 11.84 deg while
                // the hood kept climbing, the MAX_ANGLE travel guard compared against that frozen
                // value and never fired, and the hood drove into the top stop until the driver
                // disabled. The old 20 was sized off HAND sweeps (~1.6 units/loop) -- hand motion
                // is an order of magnitude slower than powered motion, so it was never a valid
                // ceiling for the thing it had to pass.
                //
                // 120 is the largest sane value, not a tuned one: above 180 a delta is
                // indistinguishable from the same motion the other way round the circle, so the
                // wrap logic (shortestRawDelta) stops being able to tell direction. 120 keeps
                // that margin while clearing the measured 55.
                //
                // TODO: re-derive from the real scale once step 3 of the hood bring-up measures
                // units-per-hood-degree against an angle finder. A filter this wide also means a
                // SLIPPING drive accumulates tracked angle fast -- which trips the travel guard
                // and cuts drive, the fail-safe direction, but it is not a substitute for the
                // motor-vs-hood skip detector (step 4).
                public static double HOOD_RAW_MAX_STEP = 120.0;
                // How far ABOVE its enable-time position the hood may be commanded, in RAW UNITS
                // (2026-08-27). This is the ONLY upper travel limit now: MAX_ANGLE is a physical
                // degree belonging to the shot model and cannot be compared against a raw reading.
                // The hood's whole stroke is HOOD_RAW_UNITS_PER_FULL_TRAVEL, so this permits
                // exactly the full travel above the datum -- same assumption as before, that the
                // hood is enabled resting on its bottom stop. Enable it already raised and the
                // ceiling sits that much too high; see captureHoodDatum's note.
                public static double HOOD_TRAVEL_WINDOW_DEG = HOOD_RAW_UNITS_PER_FULL_TRAVEL; // 309.6
            //SHOT MODEL (distance -> velocity; quadratic-drag ballistics for the OFFICIAL FUEL
            // ball -- 5.91 in / 0.203-0.227 kg foam, Cd~0.5, manual sec.5.10.1 -- validated
            // against the no-drag closed form. Angle rule: FIXED at MAX_ANGLE (38 deg since the
            // belt-skip inset -- see SHOOTER CONSTRAINTS).
            // REFIT 2026-07-21 AT 38 DEG (docs/shot-model/refit_38.py): every drag/ToF fit and
            // MIN_SCORE_DISTANCE_METERS below is now derived AT 38 (the old 44.5-deg fits
            // commanded ~6% too little ball speed everywhere and a 3.0 m minimum whose entry
            // was a grazing -2.5 deg). ShotModelTest pins the model to the integrator truth --
            // if MAX_ANGLE ever changes again, re-run refit_38.py at the new angle and update
            // the fits AND the test pins together.
                // Ball release height above carpet. User-measured 2026-07-10: bottom of swerve
                // to the UNEXTENDED hood = 27 in. TODO re-measure to the actual ball exit point.
                public static final double SHOT_RELEASE_HEIGHT_METERS = Units.inchesToMeters(27);
                // Ball exit speed / flywheel surface speed (energy lost to backspin ~half).
                // Biggest single unknown -- every model velocity scales with it; tune FIRST,
                // drag slopes LAST. TODO tune on robot (0.45-0.55).
                public static double SHOT_EFFICIENCY = 0.50;
                // Distance-dependent drag multipliers (ball-speed ratio true/no-drag), linear
                // fits to the integrator AT 38 DEG (refit_38.py 2026-07-21):
                //   score: mult(d) = 0.9988 + 0.01027*d, fitted over 3.5-6.2 m (residual 0.01%)
                //   feed:  mult(d) = 0.9954 + 0.01063*d, fitted over 4-9 m   (residual 0.05%)
                // Efficiency-independent (it divides out afterward). Endpoints tunable on robot
                // within ~2-3% -- beyond that re-run refit_38.py (ShotModelTest pins at 3%).
                public static double SCORE_DRAG_MULT_BASE = 0.9988;
                public static double SCORE_DRAG_MULT_PER_METER = 0.01027;  // 1/m
                public static double FEED_DRAG_MULT_BASE = 0.9954;
                public static double FEED_DRAG_MULT_PER_METER = 0.01063;   // 1/m
                // SCORE envelope to the own hub center (m). RAISED 3.0 -> 3.5 in the 38-deg
                // refit (2026-07-21): at 38 deg the descending-entry floor is 2.93 m, and the
                // window analysis (refit_38.py) shows 3.0 m enters at a grazing -2.5 deg with a
                // +-0.9% speed window -- the +-0.2 m/s kicker gate alone eats that, so 3.0-3.4 m
                // shots were near-guaranteed misses. 3.5 m enters at -9 deg with +-2.2%.
                // DRIVER-FACING: the close dead zone grew -- back off ~3 m from the hub FACE.
                // Above 6.2 the robot cannot legally be in its zone (G407 geometric limit 6.14 m).
                // Outside the envelope the model REFUSES (velocity 0 -> no belts, no kicker).
                public static double MIN_SCORE_DISTANCE_METERS = 3.5;   // [derived at 38 deg] TODO tune on robot
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
                // clean, low target well clear of the MAX_ANGLE (38) hard-travel guard. Raising to
                // it is never gated; lowering to it from a staged SCORE angle works too, because the
                // stow interlock only gates the coast-down auto-recess (desired velocity 0), never a
                // live shot's own commanded angle (see ShooterSubsystem periodic).
                // NOTE: reaching it also needed more UP torque -- see HOOD_MAX_UP_VOLTAGE (6 V could
                // not lift the hood against gravity). MUST stay < MAX_ANGLE. TODO retune
                // RB_FEED_SURFACE_SPEED for this flatter 25 deg lob on robot.
                // STALE UNITS since 2026-08-27: this is a PHYSICAL degree and the hood setpoint
                // is now in raw encoder units, so it clamps to the enable-time floor and
                // commands no hood motion. Harmless -- the RB binding itself is disabled (see
                // RobotContainer) -- but multiply by HOOD_UNITS_PER_DEG and re-anchor it to the
                // enable-time base before RB is ever restored.
                public static double RB_FEED_ANGLE = 25.0;           // deg (must be < MAX_ANGLE)
                // RT flywheel-only shot surface speed (m/s) -- team request 2026-08-22: RT now
                // just spins the wheels to this speed and the hood angle is set by hand on the
                // DPAD. Seeded from RB_FEED_SURFACE_SPEED because that is the one speed actually
                // run on the robot; it is NOT tuned for a scoring shot at an arbitrary hood
                // angle. TODO tune on robot (and note the whole point of this mode is that range
                // now comes from the manual hood angle, not from the model).
                // Cut 20% (19.8 -> 15.84) 2026-08-22 by team request.
                public static double RT_FLYWHEEL_SURFACE_SPEED = 15.84;  // m/s surface
                // Time-of-flight linear fits (s) for moving-shot compensation, refit AT 38 DEG
                // (refit_38.py 2026-07-21, residual <= 0.011 s): score 0.188 + 0.115*d,
                // feed 0.583 + 0.081*d. (The stale 44.5-deg fits over-read ToF ~14% at range,
                // inflating the moving-shot lead.)
                public static double SCORE_TOF_BASE_SEC = 0.188;
                public static double SCORE_TOF_SEC_PER_METER = 0.115;
                public static double FEED_TOF_BASE_SEC = 0.583;
                public static double FEED_TOF_SEC_PER_METER = 0.081;
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
                // never exceed the stops. NOTE: the shot model asks for MAX_ANGLE at close range,
                // so close shots land flatter until MAX is raised -- raise it toward the stop only
                // once overshoot is controlled and ballistics are tuned. TODO tune the margin on robot.
                public static double MAX_ANGLE = 38.0;   // was 44.5 (hard stop), then 43; -5 for belt-skip leeway
                public static double MIN_ANGLE = 5.0;    // was 3.224 (hard stop)
                // Hood drive voltage caps -- ASYMMETRIC (2026-07-18). Gravity always pulls the hood
                // toward the DOWN stop, so raising fights gravity and lowering is gravity-assisted.
                // The old symmetric cap was gentle enough for the DOWN stroke but too weak to
                // LIFT the hood at all (it sat fully down during the feed shot). So the angle PID
                // output is now clamped to [-DOWN, +UP]:
                //   UP  (positive volts raise): 10 V -- enough to overcome gravity + friction and
                //       servo up. Still 2 V below the original 12 V that over-extended, and the hard
                //       MAX_ANGLE guard in periodic() (plus the ~6.5 deg inset below the 44.5 stop
                //       and Brake idle) still catch any coast, so belt-skip risk stays bounded.
                //   DOWN (negative volts lower): 6 V -- kept gentle; gravity assists, so this is
                //       plenty for a controlled auto-descend and keeps it gentle on the
                //       stroke where over-travel into the bottom would be roughest.
                // TODO tune on robot: raise UP toward 12 if the lift is still sluggish; LOWER it if
                // high-angle shots over-extend past MAX_ANGLE and skip the belt.
                // LOWERED 10.0 -> 8.0 on 2026-08-22 (team request: the steps were too big and too
                // fast). Breakaway is 7.5 V, so a 10 V ceiling meant every slip had up to 2.5 V of
                // surplus behind it -- the hood lunged. At 8.0 the surplus is 0.5 V, so it creeps
                // through each step instead. This is the knob for step VIOLENCE; the FF is the knob
                // for step FREQUENCY. TODO on robot: if the hood stalls near the TOP of its travel
                // (gravity load is worst there) raise toward 8.5-9.0 -- 8.0 is only 0.5 V over
                // breakaway, which is not much margin.
                public static double HOOD_MAX_UP_VOLTAGE = 8.0;   // raising (fights gravity)
                public static double HOOD_MAX_DOWN_VOLTAGE = 6.0;  // lowering (gravity-assisted, gentle)
                // Hood gravity LIFT feedforward (volts), added to the angle-PID output ONLY while the
                // hood is still below its target (raising). The hood loop is P+D (kP 0.18, kD 0.004,
                // no integrator -- see SHOOTER_ANGLE_kP), so at a
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
                // small errors; LOWER it if shots reach MAX_ANGLE too hard.
                // RAISED 5.0 -> 7.0 on 2026-08-22 (team request: smaller, more frequent steps),
                // using the breakaway measured that day -- the hood does NOT move at 7.0 V and
                // does at 7.5 V. At 5.0 the loop peaked near 5.7 V (FF + 0.275 V/deg of P), under
                // breakaway, so error had to reach ~9 deg before anything moved: big, rare lurches.
                // At 7.0 the FF plus a few tenths of P crosses breakaway at ~2 deg instead, so the
                // hood steps roughly 4x as often and 4x as small. Deliberately kept just BELOW the
                // 7.5 V standalone breakaway so the FF alone can never move the hood -- P still has
                // to ask for it. TODO tune on robot: lower if shots overshoot toward MAX_ANGLE.
                public static double HOOD_RAISE_FF_VOLTS = 7.0;
                // FF FADE BAND (deg), 2026-08-22: the lift feedforward used to STEP from 0 to the
                // full 5 V the instant error crossed ANGLE_TOLERANCE, which made the DPAD jog move
                // in lurches -- nothing (PID alone is ~0.14 V at small error, far under breakaway),
                // then a 5 V slam, then error collapses and the FF cuts out, repeat. The FF now
                // ramps linearly from 0 at ANGLE_TOLERANCE to full at ANGLE_TOLERANCE + this band,
                // so there is no discontinuity anywhere. Endpoints are unchanged: still 0 V at the
                // target, still 5 V at a large error. Bigger = smoother but slower to break away;
                // smaller = back toward the old lurch. TODO tune on robot.
                // Narrowed 2.0 -> 1.0 on 2026-08-22 alongside the FF raise: the FF reaches full a
                // degree sooner, so breakaway comes sooner and the steps shorten further. The fade
                // is what keeps this from being the step function that caused the original lurch.
                // RAW UNITS (2026-08-27): the tuned 1.0 physical degree, converted.
                public static double HOOD_FF_FADE_DEG = 1.0 * HOOD_UNITS_PER_DEG;   // 7.5 raw units
                // Surface-speed gate for the kicker, m/s. TIGHTENED 0.3 -> 0.2 (spec 6): at the
                // 3.0 m minimum, 0.3 m/s maps to 0.30 m of along-track error -- more than the
                // 0.226 m half-window through the opening; 0.2 closes the budget exactly.
                public static double SPEED_TOLERANCE = 0.2;
                public static double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);
                // HOOD_LOWER_MAX_SURFACE_SPEED / HOOD_STOW_INTERLOCK_FLOOR_DEG were DELETED
                // 2026-08-26 along with auto-stow. They gated the coast-down auto-recess -- with
                // nothing lowering the hood on its own, the only motion left for them to block
                // was a driver DPAD-down while the flywheels spun, which must never be blocked.
                // Hood gate, deg: worst contribution 0.078 m at 3 m, shrinking with distance.
                // RAW UNITS (2026-08-27): the tuned 0.5 physical degree, converted.
                public static double ANGLE_TOLERANCE = 0.5 * HOOD_UNITS_PER_DEG;   // 3.75 raw units

                // Measured breakaway: the hood does not move UP below ~7.5 V (on-robot
                // 2026-08-22). P + hoodLiftFeedforward only reaches that at ~10 units of error
                // -- one whole HOOD_UP_STEP_UNITS -- so between the re-engage threshold and a
                // full step the loop commands 2-5 V, which energises the motor without moving
                // the hood. That is the sag: brake idle mode is a DAMPER (shorted terminals,
                // torque only while moving), not a static brake, so a gravity-loaded hood
                // creeps down and the loop cannot catch it until it has lost the entire step.
                // hoodBreakawayFloor lifts an active UP command to this value so a commanded
                // stroke finishes instead of dying short. TODO on robot: if a press stalls short,
                // raise this (never above HOOD_MAX_UP_VOLTAGE); if it snaps up too hard, lower.
                public static double HOOD_BREAKAWAY_VOLTS = 7.5;
                
                // HOOD_JOG_UP_VOLTS / HOOD_JOG_DOWN_VOLTS are DELETED (2026-08-28, team
                // direction: "voltage throttle should be entirely removed, the only throttle
                // available should be for rpm on the shooter"). The hood has no open-loop drive
                // path any more, in teleop or in test mode -- every hood volt now comes out of
                // the position loop (SHOOTER_ANGLE_k* + hoodLiftFeedforward), bounded by
                // HOOD_MAX_UP_VOLTAGE / HOOD_MAX_DOWN_VOLTAGE and the travel guards.
                // CONSEQUENCE, accepted by the team: parking the hood ON a hard stop to re-read
                // the encoder anchors is no longer possible from the controller -- every setpoint
                // is clamped to [enable datum, datum + travel]. Move it by hand, or re-add a
                // test-mode-only jog if that calibration is needed again.
                // DPAD UP STEP (2026-08-27, team direction: "in teleop the hood should go up
                // 5 degrees every press", raised to 10 the same day). EVERY press of DPAD RIGHT
                // raises the hood by this much, measured from where the hood IS at the moment of
                // the press. DPAD LEFT returns it to the base (the enable-time datum, re-read
                // from the raw encoder on every enable -- see captureHoodDatum).
                //
                // RAW UNITS, deliberately NOT converted: the team asked for a number on the same
                // scale as the raw encoder readout, so this is 10 raw units = 1.33 PHYSICAL
                // degrees. Multiply by HOOD_UNITS_PER_DEG (-> 75) if 10 physical degrees was meant.
                //
                // WHAT ACTUALLY HAPPENS ON THE ROBOT, accepted knowingly by the team (re-confirmed
                // 2026-08-28): a press moves 25-55 raw units (3-7 physical deg), NOT 10. Closing
                // the loop does not change that number -- it is set by the hardware. The hood
                // cannot move at all below ~7.5 V, and one 20 ms loop AT that voltage already
                // carries it further than this step, so the smallest move the hood can make is
                // several times the step being asked for. kD (see SHOOTER_ANGLE_kD) shortens the
                // arrival, and the settle band absorbs what is left, but a 10-unit step will
                // still land long. The step only becomes literal past ~60 units.
                public static double HOOD_UP_STEP_UNITS = 10.0;
                // HOOD_REENGAGE_DEG (the drift that used to restart the drive) is DELETED
                // 2026-08-28: re-engaging on droop WAS the up/down limit cycle -- one catch
                // stroke is 25-55 units against a 3.75-unit band, so it always overshot and
                // re-seeded higher. The hood now parks on arrival and only a new setpoint moves
                // it. See ShooterSubsystem.hoodHolding.
                // RAW UNITS (2026-08-27): the tuned 0.75 physical degree, converted.
                // HOOD_LOWER_FF_VOLTS is self explanatory, based off raise volts
                public static double HOOD_LOWER_FF_VOLTS = 6.0;
        }
}