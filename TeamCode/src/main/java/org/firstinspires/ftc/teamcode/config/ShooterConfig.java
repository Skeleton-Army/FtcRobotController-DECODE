package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA;

@Config
public class ShooterConfig {
    static double metersToInches = 39.37;

    public static double SHOOT_HEIGHT = 0.4 * metersToInches;
    public static String FLYWHEEL1_NAME = "flywheel1";
    public static String FLYWHEEL2_NAME = "flywheel2";
    public static GoBILDA FLYWHEEL_MOTOR = GoBILDA.BARE;
    public static boolean FLYWHEEL1_INVERTED = true;
    public static boolean FLYWHEEL2_INVERTED = false;
    public static int RPM_REACHED_THRESHOLD = 50;
    public static int SHOT_RPM_DROP = 400;
    public static double FLYWHEEL_KP = 2.4;
    public static double FLYWHEEL_KI = 0.002;
    public static double FLYWHEEL_KD = 0.1;
    public static double FLYWHEEL_KS = 0;
    public static double FLYWHEEL_KA = 0.001; //  or 0.01
    public static double FLYWHEEL_KV = 1.66;
    public static double FLYWHEEL_DELAY_SEC = 0.06; // ~2 control loops @ 20ms

    public static String TURRET_NAME = "turret";
    public static GoBILDA TURRET_MOTOR = GoBILDA.RPM_1150;
    public static double ANGLE_REACHED_THRESHOLD = Math.toRadians(3);
    public static double TURRET_KP = 2.3;
    public static double TURRET_KI = 0;
    public static double TURRET_KD = 0;
    public static double TURRET_KS = 0;
    public static double TURRET_KV = 0;
    public static double TURRET_KA = 0;
    public static double TURRET_MIN = Math.toRadians(-70); // Clockwise
    public static double TURRET_MAX = Math.toRadians(70); // Counter-clockwise
    public static double GEAR_RATIO = (double) 111 / 11;
    public static double TURRET_OFFSET_X = -0.591; // Positive = Front
    public static double TURRET_OFFSET_Y = 0; // Positive = Left
    public static boolean TURRET_WRAP = false;

    public static String HOOD_NAME = "hood";
    public static boolean HOOD_INVERTED = true;
    public static double HOOD_POSSIBLE_MIN = 0;
    public static double HOOD_POSSIBLE_MAX = 0.96;
    public static double HOOD_MIN = Math.toRadians(64);
    public static double HOOD_MAX = Math.toRadians(30.04);
    public static double  HOOD_COMPENSATION = 0.01;
}
