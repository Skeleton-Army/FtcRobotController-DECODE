package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConsts {
    public static double SHOT_LATENCY = 0.1;
    public static double SHOOTER_MIN_VELOCITY = 4.4;
    public static double SHOOTER_MAX_VELOCITY = 8;
    public static double MIN_DISTANCE = 0.7;
    public static double MIN_DISTANCE_TABLE = 1;
    public static double MID_DISTANCE_TABlE = 3;
    public static double MAX_DISTANCE_TABLE = 4.3;
    public static double VELOCITY_BIAS = 0.8; // 0 = lowest possible velocity, 1 = highest possible velocity

    public static double MAX_DISTANCE = 4.3;
    public static double CLOSE_MIN_DISTANCE = 0.7;
    public static double CLOSE_MAX_DISTANCE = 2.5;
    public static double FAR_MAX_DISTANCE = 4.3;
    public static double CLOSE_SHOOTER_MIN_VELOCITY = 4.73;
    public static double CLOSE_SHOOTER_MAX_VELOCITY = 6.4;
    public static double FAR_SHOOTER_MIN_VELOCITY = 8;
    public static double FAR_SHOOTER_MAX_VELOCITY = 8;
    public static double MOVEMENT_COMPENSATION = 0.0;
}
