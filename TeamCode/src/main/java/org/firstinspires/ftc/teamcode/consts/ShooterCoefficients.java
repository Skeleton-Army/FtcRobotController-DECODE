package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterCoefficients {
    public double[] RPM_INTERPOLATION = {520, 0};
    public double[] VELOCITY_INTERPOLATION = {1, 3.7};
    public double[] MIN_VEL_COEFFS = {0.03051737, -0.39733680, 1.92515700, -4.17389950, 4.66861781, 2.67109537};
    public double[] MAX_VEL_COEFFS = {0.00899242, -0.13875104, 0.83303624, -2.41515766, 4.33287142, 2.07668995};
    public double[] HOOD_COEFFS = {-0.01340167, 0.19520493, -1.10985582, 3.07826998, -4.20510022, 3.08414438};
}