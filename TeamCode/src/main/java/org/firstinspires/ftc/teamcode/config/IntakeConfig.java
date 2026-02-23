package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA;

@Config
public class IntakeConfig {
    public static String INTAKE_NAME = "intake";
    public static GoBILDA INTAKE_MOTOR = GoBILDA.RPM_1150;
    public static double INTAKE_POWER = 1;
    public static double SHOOTING_POWER = 1;
    public static double SLOW_SHOOTING_POWER = 1;
    public static double STALL_THRESHOLD = 200;
    public static double STALL_STARTUP_DELAY = 0.5;
}
