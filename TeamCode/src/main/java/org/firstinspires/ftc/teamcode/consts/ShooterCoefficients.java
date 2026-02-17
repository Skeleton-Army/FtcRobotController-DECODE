package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public static double[] RPM_INTERPOLATION = {420, 0};
    public static double[] VELOCITY_INTERPOLATION = {1.0101, 3.6908};
    public static double[] CLOSE_VEL_COEFFS = {415, 100};
    public static double[] FAR_VEL_COEFFS = {415, 100};
    public static final double DISTANCE_THRESHOLD_METERS = 2.5;
    public static final double[] HOOD_COEFFS = {-0.01244952, 0.18496110, -1.07567572, 3.06070616, -4.29576364, 3.16990449};

    public static final double[] MIN_VEL_COEFFS = {0.03563689, -0.48579242, 2.50868955, -6.00051606, 7.34033378, 1.31432148};

    public static final double[] MAX_VEL_COEFFS = {-0.00075515, -0.02372834, 0.35646624, -1.64814659, 4.08239382, 1.92131493};
}