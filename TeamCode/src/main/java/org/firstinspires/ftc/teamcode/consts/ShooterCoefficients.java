package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public double[] RPM_INTERPOLATION = {415, 0};
    public double[] VELOCITY_INTERPOLATION = {1.1413, 3.6006};
    public double[] MIN_VEL_COEFFS = {-0.00753753, 0.14158881, -1.01421641, 3.47785831, -4.57624623, 6.68692950};
    public double[] MAX_VEL_COEFFS = {0.01871128, -0.26276313, 1.42527417, -3.70961862, 5.72298206, 1.59238723};
    public double[] HOOD_COEFFS = {-0.00842119, 0.12090451, -0.67832152, 1.87398840, -2.66079694, 2.38229058};
}