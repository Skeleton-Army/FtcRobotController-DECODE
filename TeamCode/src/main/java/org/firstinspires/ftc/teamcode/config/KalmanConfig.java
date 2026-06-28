package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

@Config
public class KalmanConfig {


    // the stdev-squared for each asix - x,y and heading
    // TODO: These are not
    public static Pose initialCovariance = new Pose(0, 0, Math.toRadians(0)); // the initial covariance for pinpoint - how much we trust our first positioning of the robot when placing it on the field
    public static Pose processVariance = new Pose(1,1, Math.toRadians(0.05) / 120); // the drift covariance
    public static Pose measurementVariance = new Pose(2, 2, Math.toRadians(1000)); // the apriltag covariance
    public static int bufferSize = 100;

    public static boolean enableMeasurements = false;

    // Your empirical measurement: 36ms hardware transit delay
    public static double CAMERA_PHYSICAL_LATENCY_MS = 30; // tune this

    // Standard FTC I2C bulk read loop overhead: ~3ms delay
    public static double PINPOINT_I2C_LATENCY_MS = 3;

    public static double apriltagDistanceCoeff = 0;
    public static double apriltagTagSizeCoeff = 0;
    public static double apriltagTagSizeCoeffX = 0;
    public static double apriltagTagSizeCoeffY= 0;

    public static double apriltagTagSizeCoeffAngle = 0;







}
