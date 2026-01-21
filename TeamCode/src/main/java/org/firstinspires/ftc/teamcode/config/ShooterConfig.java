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
    public static int RPM_REACHED_THRESHOLD = 50;
    public static double FLYWHEEL_KP = 5;
    public static double FLYWHEEL_KI = 0;
    public static double FLYWHEEL_KD = 0;
    public static double FLYWHEEL_KS = 190;
    public static double FLYWHEEL_KA = 0;
    public static double FLYWHEEL_KV = 1.7;
    public static double FLYWHEEL_DELAY_SEC = 0.06; // ~2 control loops @ 20ms
    public static double RPM_SMOOTHING_FACTOR = 0.05; // 0.1 (very smooth) - 0.8 (responsive)

    public static String TURRET_NAME = "turret";
    public static GoBILDA TURRET_MOTOR = GoBILDA.RPM_1150;
    public static double TURRET_POSITION_TOLERANCE = Math.toRadians(1);
    public static double TURRET_VELOCITY_TOLERANCE = Math.toRadians(30);
    public static double TURRET_IZONE = Math.toRadians(10);
    public static double ACCELERATION_SMOOTHING_GAIN = 1;
    public static double TURRET_KP = 0.5;
    public static double TURRET_KI = 0.2;
    public static double TURRET_KD = 0;
    public static double TURRET_KS = 0.165;
    public static double TURRET_KV = 0.09;
    public static double TURRET_KA = 0.012;
    public static double TURRET_MIN = Math.toRadians(-180); // Clockwise
    public static double TURRET_MAX = Math.toRadians(180); // Counter-clockwise
    public static double GEAR_RATIO = (double) 111 / 11;
    public static double TURRET_OFFSET_X = -0.591; // Positive = Front
    public static double TURRET_OFFSET_Y = 0; // Positive = Left
    public static boolean TURRET_WRAP = true;

    public static String HOOD_NAME = "hood";
    public static boolean HOOD_INVERTED = false;
    public static double HOOD_POSSIBLE_MIN = 0;
    public static double HOOD_POSSIBLE_MAX = 0.96;
    public static double HOOD_MIN = Math.toRadians(33.44);
    public static double HOOD_MAX = Math.toRadians(64);

    public static double STALL_TIMEOUT = 1;
    public static double CURRENT_THRESHOLD = 8;
}
