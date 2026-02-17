package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public static double[] RPM_INTERPOLATION = {412, 0};
    public static double[] VELOCITY_INTERPOLATION = {1.1413, 3.6006};
    public static double[] CLOSE_VEL_COEFFS = {415, 100};
    public static double[] FAR_VEL_COEFFS = {415, 100};
    public static final double DISTANCE_THRESHOLD_METERS = 2.5;
    public static final double[] HOOD_COEFFS = {-0.01019346, 0.15389212, -0.91188978, 2.65911739, -3.89362224, 3.03329116};

    public static final double[] MIN_VEL_COEFFS = {0.02770411, -0.38846154, 2.07521833, -5.17235799, 6.88146406, 1.32267170};

    public static final double[] MAX_VEL_COEFFS = {-0.01138695, 0.13700431, -0.58997567, 1.04296726, 0.59121101, 3.55756520};
}