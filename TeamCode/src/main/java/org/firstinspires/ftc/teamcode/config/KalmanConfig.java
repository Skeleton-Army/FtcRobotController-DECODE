package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

@Config
public class KalmanConfig {


    // the stdev-squared for each asix - x,y and heading
    // TODO: These are not
    public static Pose initialCovariance = new Pose(0.01, 0.01, Math.toRadians(0.0001)); // the initial covariance for pinpoint - how much we trust our first positioning of the robot when placing it on the field
    public static Pose processVariance = new Pose(Math.pow(0.02, 2),Math.pow(0.02, 2), Math.toRadians(0.05) / 60); // the drift covariance - high due to vibrations from our beloved flywheel
    public static Pose measurementVariance = new Pose(0.5, 0.5, 100); // the apriltag covariance
    public static int bufferSize = 100;

    public static boolean enableMeasurements = true;

    // Your empirical measurement: 36ms hardware transit delay
    public static double CAMERA_PHYSICAL_LATENCY_MS = 30; //TODO: tune this

    // Standard FTC I2C bulk read loop overhead: ~3ms delay
    public static double PINPOINT_I2C_LATENCY_MS = 0;

    public static double apriltagDistanceCoeff = 0;
    public static double apriltagTagSizeCoeff = 0;
    public static double apriltagTagSizeCoeffX = 0;
    public static double apriltagTagSizeCoeffY= 0;

    public static double apriltagTagSizeCoeffAngle = 0;

    public static double farVarianceThreshold = 100;
    public static double farVariance = 1.2;
}
