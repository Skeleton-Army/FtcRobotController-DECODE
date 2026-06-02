package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA;

@Config
public class ShooterConfig {

    public static String FLYWHEEL1_NAME = "flywheel1";
    public static String FLYWHEEL2_NAME = "flywheel2";
    public static GoBILDA FLYWHEEL_MOTOR = GoBILDA.BARE;
    public static boolean FLYWHEEL1_INVERTED = true;
    public static boolean FLYWHEEL2_INVERTED = false;
    public static int RPM_REACHED_THRESHOLD = 200;
    public static double FLYWHEEL_KP = 0.05;
    public static double FLYWHEEL_KP_DOWN = 0.05;
    public static double FLYWHEEL_KI = 0;
    public static double FLYWHEEL_KD = 0;
    public static double FLYWHEEL_KS = 1.5;
    public static double FLYWHEEL_KA = 0.01;
    public static double FLYWHEEL_KA_DOWN = 0.01;
    public static double FLYWHEEL_KV = 0.0052;
    public static double BRAKE_ENTRY_THRESHOLD = -100; // Error must be THIS negative to start braking
    public static double BRAKE_EXIT_THRESHOLD = -50;   // Error must get back up to THIS to stop braking
    public static double FLYWHEEL_DELAY_SEC = 0.02; // ~2 control loops @ 20ms
    public static double FLYWHEEL_SHOOTING_DIFFRENCE = 0.02;
    public static int RPM_WINDOW_SIZE = 5;
    public static double INITIAL_RAMP_DURATION = 0.3;

    public static String TURRET_NAME = "turret";
    public static GoBILDA TURRET_MOTOR = GoBILDA.RPM_1150;
    public static boolean TURRET_INVERTED = true;
    public static double TURRET_POSITION_TOLERANCE = Math.toRadians(0.3);
    public static double TURRET_VELOCITY_TOLERANCE = Math.toRadians(30);
    public static double TURRET_IZONE = Math.toRadians(30);
    public static double ACCELERATION_SMOOTHING_GAIN = 0.7;
    public static double TURRET_DERIVATIVE_GAIN = 0.8;
    public static double TURRET_SECOND_DERIVATIVE_GAIN = 0.8;
    public static double TURRET_DELAY = 0.02;
    public static double TURRET_MIN_VOLTAGE = 0.4;
    public static double TURRET_KP = 5;
    public static double TURRET_KI = 0;
    public static double TURRET_KD = 0.1;
    public static double TURRET_KS = 1;
    public static double TURRET_KS_CW_0    = 0;  public static double TURRET_KS_CCW_0    = 0;  // 0–1000 RPM
    public static double TURRET_KS_CW_1000 = 0;  public static double TURRET_KS_CCW_1000 = 0;  // 1000–2000 RPM
    public static double TURRET_KS_CW_2000 = 0;  public static double TURRET_KS_CCW_2000 = 0;  // 2000–3000 RPM
    public static double TURRET_KS_CW_3000 = 0;  public static double TURRET_KS_CCW_3000 = 0;  // 3000–4000 RPM


    // clockwise PID - Turret
    public static double TURRET_KP_CW = 1;
    public static double TURRET_KI_CW = 0;
    public static double TURRET_KD_CW = 0;

    //Count clockwise PID - Turret
    public static double TURRET_KP_CCW = 0.7;
    public static double TURRET_KI_CCW = 0.02;
    public static double TURRET_KD_CCW = 0.01;
//    public static double TURRET_KV = 1.8;
//    public static double TURRET_KA = 0;
    public static double TURRET_MIN = Math.toRadians(-184); // Clockwise
    public static double TURRET_MAX = Math.toRadians(184); // Counter-clockwise
//    public static double GEAR_RATIO = (double) 286 / 20;
    public static double TURRET_OFFSET_X = 0; // Positive = Front
    public static double TURRET_OFFSET_Y = 0; // Positive = Left
    public static boolean TURRET_WRAP = true;

    public static double DIST_CLOSE = 30; // Inches
    public static double DIST_FAR = 160; // Inches
    public static double WINDOW_CLOSE = Math.toRadians(6.0); // Max error allowed when close
    public static double WINDOW_FAR = Math.toRadians(1.0); // Max error allowed when far
    public static double TURRET_VELOCITY_WINDOW = Math.toRadians(40); // Max velocity error threshold
    public static double VELOCITY_WINDOW_GAIN = Math.toRadians(0.05); // Window growth per inch/sec of speed
    public static double MAX_WINDOW_SIZE = Math.toRadians(6.0); // Absolute limit for the window

    public static String HOOD_NAME = "hood";
    public static boolean HOOD_INVERTED = false;
    public static double HOOD_POSSIBLE_MIN = 0;
    public static double HOOD_POSSIBLE_MAX = 0.96;
    public static double HOOD_MIN = Math.toRadians(33.44);
    public static double HOOD_MAX = Math.toRadians(64);

    public static double STALL_TIMEOUT = 1;
    public static double CURRENT_THRESHOLD = 8;
// ============================================================
//  ADD THESE CONSTANTS TO YOUR EXISTING ShooterConfig.java
// ============================================================

    // ── Flywheel motor physics ───────────────────────────────────────────────────
    public static final double FLYWHEEL_STALL_CURRENT_A    = 9.2;    // A
    public static final double FLYWHEEL_NO_LOAD_RPM         = 5800;   // RPM
    public static final double FLYWHEEL_STALL_TORQUE_NM     = 0.144;  // N·m
    public static final double FLYWHEEL_NO_LOAD_CURRENT_A   = 0.25;   // A

    // Driving limit: maximum current during spin-up / hold.
// Start at 70% of stall current. Raise if spin-up is too sluggish.
    public static final double FLYWHEEL_MAX_DRIVING_CURRENT_A = 6.5;  // A  (~70% of 9.2)

    // Regen limit: maximum current flowing BACK into the bus during deceleration.
// This is the new safety added for regen spikes. Keep it tight (1–2 A).
// After a shot the flywheel decelerates quickly; without this limit the
// regenerated current causes a voltage spike on the 12 V bus that can crash
// the Control Hub or corrupt encoder reads mid-match.
    public static final double FLYWHEEL_MAX_REGEN_CURRENT_A   = 2.0;  // A


    // ── Turret motor physics ─────────────────────────────────────────────────────
    // ── Turret: GoBilda 5203 1150 RPM, 286:20 gear (14.3:1) ──────────────────────

    // Gear ratio: motor turns per one output-shaft turn
    public static final double GEAR_RATIO                    = 14.3;

    // Motor datasheet values (motor shaft — do NOT adjust for gear ratio)
    public static final double TURRET_STALL_CURRENT_A        = 9.2;
    public static final double TURRET_NO_LOAD_RPM            = 1150;
    public static final double TURRET_STALL_TORQUE_NM        = 0.192;
    public static final double TURRET_NO_LOAD_CURRENT_A      = 0.25;

// Calculated motor constants (for reference only — setMotorPhysicsConstants derives these)
//   R  = 1.3043 Ω
//   Ke = 0.096937 V/(rad/s)   [motor shaft]
//   Kt = 0.020870 N·m/A

    // Output-shaft feedforward gains (gear ratio already baked in)
    public static final double TURRET_KV = 1.3862; // V / (output rad/s)

    //   TURRET_KA = (J_output_kg_m2 / 0.020870) / 14.3
//   Pick the row closest to your turret plate:
//     0.3 kg, r=0.10 m  →  J=0.00150 kg·m²  →  TURRET_KA = 0.005026
//     0.5 kg, r=0.12 m  →  J=0.00360 kg·m²  →  TURRET_KA = 0.012063
//     0.8 kg, r=0.15 m  →  J=0.00900 kg·m²  →  TURRET_KA = 0.030157
    public static final double TURRET_KA = 0.0; // ← fill in from table above

    // Current limits
//   At 3.45 A driving: max output torque ≈ 1.03 N·m  (37% of the 2.75 N·m theoretical stall)
//   More than enough for tracking; tight enough to protect the 286:20 gear mesh.
    public static final double TURRET_MAX_DRIVING_CURRENT_A  = 3.45;
    public static final double TURRET_MAX_REGEN_CURRENT_A    = 1.21;


// ============================================================
//  GAIN UNIT REFERENCE
// ============================================================
//
//  All flywheel gains are in volts/(rad/s)-based units.
//  The controller computes a voltage; applyVoltage() handles the rest.
//
//  FLYWHEEL_KP          V / (rad/s)    proportional
//  FLYWHEEL_KP_DOWN     V / (rad/s)    proportional, braking phase
//  FLYWHEEL_KI          V / rad        integral
//  FLYWHEEL_KD          V / (rad/s²)   derivative
//  FLYWHEEL_KS          V              static friction offset
//  FLYWHEEL_KV          V / (rad/s)    velocity FF (fallback only, when physics OFF)
//  FLYWHEEL_KA          V / (rad/s²)   acceleration FF, spin-up phase
//  FLYWHEEL_KA_DOWN     V / (rad/s²)   acceleration FF, deceleration phase
//
//  TURRET_KP            V / rad
//  TURRET_KI            V / (rad·s)
//  TURRET_KD            V / (rad/s)
//  TURRET_KV            V / (rad/s)    motion compensation velocity FF
//  TURRET_KA            V / (rad/s²)   motion compensation acceleration FF
//  TURRET_KS            V              static friction offset


// ============================================================
//  HOW applyVoltage() USES THESE CONSTANTS
// ============================================================
//
//  Every call to motor.applyVoltage(desiredVolts, batteryVolts):
//
//    backEMF  = encoder_velocity_rad_per_sec × Ke
//
//    vMax     = backEMF + FLYWHEEL_MAX_DRIVING_CURRENT_A × R   ← driving ceiling
//    vMin     = backEMF − FLYWHEEL_MAX_REGEN_CURRENT_A   × R   ← regen floor
//
//    clamped  = clamp(desiredVolts, vMin, vMax)
//    clamped  = clamp(clamped, −batteryVolts, +batteryVolts)
//    power    = clamped / batteryVolts
//
//  This means:
//    - A PID that asks for 13 V during spin-up gets silently capped to vMax.
//    - A flywheel spinning down after a shot that would generate 3 A of regen
//      current gets silently capped to vMin instead of spiking the bus.
//    - No subsystem code needs to think about current. Just call applyVoltage().


// ============================================================
//  TUNING GUIDE
// ============================================================
//
//  1. Validate physics constants
//     Add telemetry for motor.getEstimatedCurrent(lastAppliedVoltage).
//     During steady-state spin at target RPM, estimated current should be
//     close to the hardware sensor reading (within ~0.5 A).
//     If estimated reads high: increase FLYWHEEL_NO_LOAD_CURRENT_A.
//     If estimated reads low:  decrease it.
//
//  2. Tune FLYWHEEL_MAX_DRIVING_CURRENT_A
//     Watch peak current during a cold spin-up from 0 to full speed.
//     Set this just below the observed peak. Spin-up time should be
//     nearly unchanged; if it feels noticeably slower, raise the limit.
//
//  3. Tune FLYWHEEL_MAX_REGEN_CURRENT_A
//     Shoot a game element and watch the RPM drop. Look for voltage spikes
//     on the Driver Hub or jitter in encoder reads right after the shot.
//     Start at 1.5–2 A. Raise if post-shot RPM recovery is too slow.
//     Lower if you still see bus voltage spikes.
//
//  4. Turret limits
//     Spin the turret at max speed and command a hard stop.
//     TURRET_MAX_REGEN_CURRENT_A should prevent any perceptible voltage dip
//     on the hub. TURRET_MAX_DRIVING_CURRENT_A should allow fast slews but
//     stall safely when the turret hits a physical endstop.


// ============================================================
//  TURRET GEAR RATIO & DIFFERENT MOTOR — WHAT TO CHANGE
// ============================================================
//
//  If your turret motor is different from the flywheel motor and/or has
//  a non-1:1 gear ratio, update these values:
//
//  ── Step 1: Motor datasheet constants (raw motor shaft values) ──────────────
//  Replace TURRET_* physics constants with your new motor's spec sheet values.
//  These describe what happens electrically AT THE MOTOR SHAFT — gear ratio
//  does NOT factor into these four constants.
//
//    TURRET_STALL_CURRENT_A  = <new motor stall current, A>
//    TURRET_NO_LOAD_RPM      = <new motor free-run RPM at 12V>
//    TURRET_STALL_TORQUE_NM  = <new motor stall torque, N·m>  ← motor shaft, NOT output
//    TURRET_NO_LOAD_CURRENT_A = <new motor no-load current, A>
//
//  ── Step 2: GEAR_RATIO ───────────────────────────────────────────────────────
//  Already used in setDistancePerPulse() to convert encoder ticks → output radians.
//  Definition: how many times the motor shaft turns per ONE output shaft turn.
//
//    5:1 gearbox  → GEAR_RATIO = 5.0
//    20:1 gearbox → GEAR_RATIO = 20.0
//    1:1 direct   → GEAR_RATIO = 1.0
//
//  public static final double GEAR_RATIO = 5.0;  // ← set this to your ratio
//
//  ── Step 3: Feedforward gains at the OUTPUT shaft ────────────────────────────
//  TURRET_KV and TURRET_KA are used for motion compensation feedforward in
//  updateTurretPID(). Because the PID tracks output-shaft position in radians,
//  the FF velocity and acceleration are also in OUTPUT-shaft rad/s and rad/s².
//  The motor Ke is measured at the motor shaft, so you must scale by gear ratio:
//
//    Ke_motor = (ratedV - noLoadCurrent * R) / (noLoadRPM * 2π / 60)   [V/(rad/s), motor shaft]
//    R        = ratedV / stallCurrent                                    [Ω]
//
//    TURRET_KV = Ke_motor * GEAR_RATIO
//      Why: output shaft turns GEAR_RATIO times slower than motor shaft.
//           To maintain the same back-EMF-equivalent FF voltage per unit of
//           output angular velocity, you scale Ke up by the ratio.
//
//    TURRET_KA = (J_output_kgm2 / Kt_motor) / GEAR_RATIO
//      Why: gear ratio reduces required motor torque by 1/ratio for the same
//           output acceleration, but the reflected inertia grows by ratio².
//           Net effect on the voltage-per-output-accel term: divide by GEAR_RATIO.
//      J_output_kgm2 = moment of inertia of everything on the output shaft
//                      (turret plate + game elements + any attached mechanism).
//      Kt_motor = TURRET_STALL_TORQUE_NM / TURRET_STALL_CURRENT_A
//
//  ── Example: GoBilda 5203 312 RPM (≈ 13.7:1 internal) with an extra 3:1 belt ─
//    Total GEAR_RATIO = 13.7 × 3 = 41.1
//    Motor: stall=9.2A, no-load=312RPM, stall torque=0.144N·m, no-load current=0.25A
//    R    = 12 / 9.2  = 1.304 Ω
//    Ke_motor = (12 - 0.25*1.304) / (312 * 2π/60) = 11.674 / 32.67 = 0.357 V/(rad/s)
//    TURRET_KV = 0.357 * 41.1 = 14.68  V/(output rad/s)
//
//    If J_output ≈ 0.002 kg·m², Kt = 0.144/9.2 = 0.01565 N·m/A:
//    TURRET_KA = (0.002 / 0.01565) / 41.1 = 0.128 / 41.1 = 0.00311  V/(output rad/s²)
//
//  ── Step 4: PID gains ────────────────────────────────────────────────────────
//  TURRET_KP, KI, KD operate on output-shaft radians (position error) and
//  output rad/s (velocity error). They are independent of gear ratio — tune
//  them empirically or via standard Ziegler–Nichols / relay tuning on the
//  actual assembled mechanism.
//
//  ── Step 5: TURRET_MAX_DRIVING_CURRENT_A ────────────────────────────────────
//  With a higher gear ratio, the motor needs LESS current to produce the same
//  output torque (torque multiplies by the ratio). You can set a tighter driving
//  current limit, which reduces heat and extends gear life. A good starting
//  point: stallCurrent * (max_desired_output_torque / (stallTorque * GEAR_RATIO))
}
