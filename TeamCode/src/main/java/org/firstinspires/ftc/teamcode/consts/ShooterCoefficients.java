package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public double[] RPM_INTERPOLATION = {460, 0};
    public double[] VELOCITY_INTERPOLATION = {0.9722, 3.7194};
    public static final double[] MIN_VEL_COEFFS = {-0.00528470, 0.09206074, -0.61495762, 1.95117865, -1.91193896, 5.04735174};

    public static final double[] MAX_VEL_COEFFS = {0.00222497, -0.03698359, 0.23121640, -0.66246109, 1.79566580, 3.50321521};

    public static final double[] HOOD_COEFFS = {-0.00152664, 0.02829630, -0.20423260, 0.71309074, -1.22392859, 1.66816074};
}