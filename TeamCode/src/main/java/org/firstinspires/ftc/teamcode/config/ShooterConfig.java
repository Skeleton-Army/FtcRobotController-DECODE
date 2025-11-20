package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA;

@Config
public class ShooterConfig {
    public static String FLYWHEEL_NAME = "flywheel";
    public static GoBILDA FLYWHEEL_MOTOR = GoBILDA.BARE;
    public static boolean FLYWHEEL_INVERTED = true;
    public static double FLYWHEEL_TARGET = 4000;
    public static double FLYWHEEL_KP = 2.321;
    public static double FLYWHEEL_KI = 0;
    public static double FLYWHEEL_KD = 0;
    public static double FLYWHEEL_KS = 0;
    public static double FLYWHEEL_KV = 1.756;

    public static String TURRET_NAME = "turret";
    public static GoBILDA TURRET_MOTOR = GoBILDA.RPM_435;
    public static double TURRET_KP = 2;
    public static double TURRET_MIN = -Math.PI;
    public static double TURRET_MAX = Math.PI;
    public static double GEAR_RATIO = (double) 200 / 30;
    public static double TURRET_OFFSET_X = 0;
    public static double TURRET_OFFSET_Y = 0;

    public static String HOOD_NAME = "hood";
    public static double HOOD_POSSIBLE_MIN = 0.03;
    public static double HOOD_MIN = 0;
    public static double HOOD_MAX = 260;
    public static double HOOD_START_ANGLE = 180 - 62.5; // according to hardware team

    public static String TRANSFER_NAME = "transfer";
    public static double TRANSFER_POWER = 1;

    public static String KICKER_NAME = "kicker";
    public static double KICKER_MIN = 0;
    public static double KICKER_MAX = 0.4;
    public static long KICK_TIME = 400;

    public static double DISTANCE_TO_BOT_CENTER = 0;
}
