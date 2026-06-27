package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public double[] RPM_INTERPOLATION = {470, 0};
    public double[] VELOCITY_INTERPOLATION = {0.9722, 3.7194};
    public static final double[] MIN_VEL_COEFFS = {-0.01508226, 0.21624210, -1.20411612, 3.23962373, -3.19567987, 5.49066643};
    public static final double[] MAX_VEL_COEFFS = {0.01038346, -0.13950784, 0.71383673, -1.71093057, 2.82016923, 3.14727338};
    public static final double[] HOOD_COEFFS = {-0.00526129, 0.07453143, -0.41918836, 1.17898518, -1.68283754, 1.83040709};
}