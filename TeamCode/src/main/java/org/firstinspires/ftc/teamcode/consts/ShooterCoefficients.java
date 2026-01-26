package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public static double[] CLOSE_VEL_COEFFS = {360, 80};
    public static double[] FAR_VEL_COEFFS = {395, 0};
    public static final double DISTANCE_THRESHOLD_METERS = 2.5;
    public static final double[] HOOD_COEFFS = {0.0058264, -0.074795, 0.37207, -0.93326, 1.6822};
    public static final double[] FAR_HOOD_COEFFS = {0.071373, -0.90498, 4.3316, -9.199, 7.86};
    public static final double[] CLOSE_HOOD_COEFFS = {0.0185, -0.16811, 0.60909, -1.0981, 1.6589};
}