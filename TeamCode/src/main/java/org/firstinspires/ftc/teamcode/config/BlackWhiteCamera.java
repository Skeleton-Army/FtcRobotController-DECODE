package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class BlackWhiteCamera {

    public static double offsetX_turret = 0; // 174.5mm
    public static double offsetY_turret = 6.87007874 + ShooterConfig.TURRET_OFFSET_X;
    public static double offsetZ = 12.38189; //314.5 mm
    public static double pitchAngle = 20;
    public static Position relativePos = new Position(DistanceUnit.MM, -50.306, 222.1702, 239.321, 0);

    public static int WIDTH = 1280;
    public static int HEIGHT = 720;

    public static final double[] cameraMatrix = {
            688.9972, 0, 613.914,  // fx, 0, cx
            0, 688.5173, 397.1161,  // 0, fy, cy
            0, 0, 1                         // 0, 0, 1
    };

    public static final double[] distCoeffs = {
            -0.3295, 0.1029, 0.0, 0.0, 0.0};

    /*public static final double[] cameraMatrix = {
            496.040455195, 0, 322.226720938, 0, 496.912794034, 179.36243685, 0, 0, 1                      // 0, 0, 1
    };

    // Distortion coefficients (k1, k2, p1, p2, k3)
    public static final double[] distCoeffs = {
            0.0145220035986483, -0.0121145654176830, 0.0, 0.0, 0.0};*/
}
