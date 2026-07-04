package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA;

@Config
public class ShooterConfig {

    public static String FLYWHEEL1_NAME = "flywheel1";
    public static String FLYWHEEL2_NAME = "flywheel2";
    public static GoBILDA FLYWHEEL_MOTOR = GoBILDA.BARE;
    public static boolean FLYWHEEL1_INVERTED = false;
    public static boolean FLYWHEEL2_INVERTED = true;
    public static double FLYWHEEL_GEAR_RATIO = 51.0 / 52.0; // Flywheel teeth / Motor teeth
    public static int RPM_REACHED_THRESHOLD = 200;
    public static double FLYWHEEL_KP = 0.04;
    public static double FLYWHEEL_KP_DOWN = 0.05;
    public static double FLYWHEEL_KI = 0;
    public static double FLYWHEEL_KD = 0;
    public static double FLYWHEEL_KS = 1;
    public static double FLYWHEEL_KA = 0.004;
    public static double FLYWHEEL_KA_DOWN = 0.008;
    public static double FLYWHEEL_KV = 0.0047;
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
    public static double TURRET_MIN_VOLTAGE = 0.2;
    public static double TURRET_KP = 15;
    public static double TURRET_KI = 1;
    public static double TURRET_KD = 0.7;
    public static double TURRET_KS = 0.5;
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
    public static double TURRET_KV = 3;
    public static double TURRET_KA = 0;
    public static double TURRET_MIN = Math.toRadians(-184); // Clockwise
    public static double TURRET_MAX = Math.toRadians(184); // Counter-clockwise
    public static double GEAR_RATIO = (double) 286 / 20;
    public static double TURRET_OFFSET_X = -0.0065; // In meters, Positive = Front
    public static double TURRET_OFFSET_Y = 0; // In meters, Positive = Left
    public static boolean TURRET_WRAP = true;
    public static double TURRET_WRAP_JUMP_THRESHOLD = Math.toRadians(90);

    public static double DIST_CLOSE = 30; // Inches
    public static double DIST_FAR = 160; // Inches
    public static double WINDOW_CLOSE = Math.toRadians(3.0); // Max error allowed when close
    public static double WINDOW_FAR = Math.toRadians(1.5); // Max error allowed when far
    public static double TURRET_VELOCITY_WINDOW = Math.toRadians(200); // Max velocity error threshold
    public static double VELOCITY_WINDOW_GAIN = Math.toRadians(0.01); // Window growth per inch/sec of speed
    public static double MAX_WINDOW_SIZE = Math.toRadians(3.0); // Absolute limit for the window

    public static String HOOD_NAME = "hood";
    public static boolean HOOD_INVERTED = false;
    public static double HOOD_POSSIBLE_MIN = 0;
    public static double HOOD_POSSIBLE_MAX = 1;
    public static double HOOD_MIN = Math.toRadians(31.17);
    public static double HOOD_MAX = Math.toRadians(64);
    public static double HOOD_USABLE_MIN = Math.toRadians(35.6);
    public static double HOOD_USABLE_MAX = Math.toRadians(64);

    public static double STALL_TIMEOUT = 1;
    public static double CURRENT_THRESHOLD = 8;
}
