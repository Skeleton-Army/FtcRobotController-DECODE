package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TransferConfig {
    public static String KICKER_NAME = "kicker";
    public static double KICKER_MIN = 0.29;
    public static double KICKER_MAX = 0.55;
    public static long KICK_TIME = 500;

    public static String STOPPER_NAME = "stopper";
    public static double STOPPER_MIN = 0.12;
    public static double STOPPER_MAX = 0.3;

    public static String SENSOR_NAME = "transferSensor";
    public static double DISTANCE_THRESHOLD_CM = 4;

    public  static  String DISTANCE_SENSOR_NAME = "intakeSensor";
    public static double DISTANCE_INTAKE_CM = 18;


}
