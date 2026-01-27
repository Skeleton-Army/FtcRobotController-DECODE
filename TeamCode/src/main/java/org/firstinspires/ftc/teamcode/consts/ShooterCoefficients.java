package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public static double[] VEL_COEFFS = {400, 100};
    public static double[] CLOSE_VEL_COEFFS = {360, 80};
    public static double[] FAR_VEL_COEFFS = {395, 0};
    public static final double DISTANCE_THRESHOLD_METERS = 2.5;
    public static final double[] HOOD_COEFFS = {-0.01346324, 0.19849955, -1.14432030, 3.22262259, -4.46598400, 3.23409893};
    public static final double[] FAR_HOOD_COEFFS = {0.071373, -0.90498, 4.3316, -9.199, 7.86};
    public static final double[] CLOSE_HOOD_COEFFS = {0.0185, -0.16811, 0.60909, -1.0981, 1.6589};
}