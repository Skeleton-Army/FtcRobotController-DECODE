package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TransferConfig {
    public static String TRANSFER_NAME = "transfer";
    public static double TRANSFER_POWER = 1;

    public static String KICKER_NAME = "kicker";
    public static double KICKER_MIN = 0;
    public static double KICKER_MAX = 0.4;
    public static long KICK_TIME = 500;

    public static String SENSOR_NAME = "transferSensor";
    public static double DISTANCE_THRESHOLD_CM = 4;
}
