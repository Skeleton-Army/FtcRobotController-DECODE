package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA;

@Config
public class ShooterConfig {
    static double metersToInches = 39.37;

    public static double SHOOT_HEIGHT = 0.4 * metersToInches;
    public static String FLYWHEEL_NAME = "flywheel";
    public static GoBILDA FLYWHEEL_MOTOR = GoBILDA.BARE;
    public static boolean FLYWHEEL_INVERTED = true;
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
    public static GoBILDA TURRET_MOTOR = GoBILDA.RPM_435;
    public static double ANGLE_REACHED_THRESHOLD = Math.toRadians(10);
    public static double TURRET_KP = 1.5;
    public static double TURRET_KI = 0.01;
    public static double TURRET_KD = 0.04;
    public static double TURRET_KS = 0.025;
    public static double TURRET_KV = 0.22;
    public static double TURRET_KA = 0.01;
    public static double TURRET_MIN = Math.toRadians(-210); // Clockwise
    public static double TURRET_MAX = Math.toRadians(165); // Counter-clockwise
    public static double GEAR_RATIO = (double) 200 / 30;
    public static double TURRET_OFFSET_X = -0.591; // 0.591 inch
    public static double TURRET_OFFSET_Y = 0;
    public static boolean TURRET_WRAP = true;

    public static String HOOD_NAME = "hood";
    public static double HOOD_POSSIBLE_MIN = 0.03;
    public static double HOOD_POSSIBLE_MAX = 0.9;
    public static double HOOD_MIN = Math.toRadians(27.8);
    public static double HOOD_MAX = Math.toRadians(62.5);
}
