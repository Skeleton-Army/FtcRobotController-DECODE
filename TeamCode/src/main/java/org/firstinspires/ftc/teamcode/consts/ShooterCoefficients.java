package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public static double[] RPM_INTERPOLATION = {330, 0};
    public static double[] VELOCITY_INTERPOLATION = {1.1413, 3.6006};
    public static double[] CLOSE_VEL_COEFFS = {415, 100};
    public static double[] FAR_VEL_COEFFS = {415, 100};
    public static final double DISTANCE_THRESHOLD_METERS = 2.5;
    public static final double[] HOOD_COEFFS = {-0.00724826, 0.10506345, -0.59533422, 1.65466694, -2.32887105, 2.20658018};

    public static final double[] MIN_VEL_COEFFS = {-0.02583005, 0.39161520, -2.32334103, 6.72493467, -8.42455845, 8.30635335};

    public static final double[] MAX_VEL_COEFFS = {0.01561668, -0.22149425, 1.20893427, -3.14796900, 4.87944120, 2.03087333};
}