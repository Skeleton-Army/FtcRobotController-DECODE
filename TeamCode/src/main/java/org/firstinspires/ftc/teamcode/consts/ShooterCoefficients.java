package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public static double[] RPM_INTERPOLATION = {420, 0};
    public static double[] VELOCITY_INTERPOLATION = {1.1673, 3.5767};
    public static double[] CLOSE_VEL_COEFFS = {415, 100};
    public static double[] FAR_VEL_COEFFS = {415, 100};
    public static final double DISTANCE_THRESHOLD_METERS = 2.5;
    public static final double[] HOOD_COEFFS = {-0.01019346, 0.15389212, -0.91188978, 2.65911739, -3.89362224, 3.03329116};

    public static final double[] MIN_VEL_COEFFS = {0.02563666, -0.36066151, 1.93293321, -4.83018900, 6.50218727, 1.47262155};

    public static final double[] MAX_VEL_COEFFS = {-0.00244035, -0.00276436, 0.26471416, -1.49694056, 4.20831507, 1.75071217};
}