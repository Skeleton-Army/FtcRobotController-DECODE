package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public static double[] RPM_INTERPOLATION = {420, 0};
    public static double[] VELOCITY_INTERPOLATION = {1.1253, 3.6153};
    public static double[] CLOSE_VEL_COEFFS = {415, 100};
    public static double[] FAR_VEL_COEFFS = {415, 100};
    public static final double DISTANCE_THRESHOLD_METERS = 2.5;
    public static final double[] HOOD_COEFFS = {-0.01167379, 0.16784940, -0.94105583, 2.58253214, -3.56949884, 2.79826739};

    public static final double[] MIN_VEL_COEFFS = {0.01631966, -0.19137007, 0.77019317, -1.06911109, 0.84759759, 4.39331111};

    public static final double[] MAX_VEL_COEFFS = {0.00000000, -0.00000000, 0.00000000, -0.00000000, 1.13888889, 3.60277778};

}