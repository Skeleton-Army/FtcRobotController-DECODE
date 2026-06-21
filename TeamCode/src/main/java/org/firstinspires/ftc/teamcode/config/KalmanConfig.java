package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

@Config
public class KalmanConfig {


    // the stdev-squared for each asix - x,y and heading
    public static Pose initialCovariance = new Pose(0.25, 0.25, Math.toRadians(2)); // the initial covariance for pinpoint - how much we trust our first positioning of the robot when placing it on the field
    public static Pose processVariance = new Pose(0.1,0.1, Math.toRadians(0.5) / 60); // the drift covariance
    public static Pose measurementVariance = new Pose(2,2, Math.toRadians(1)); // the apriltag covariance
    public static int bufferSize = 100;
    public static double apriltagDistanceCoeff = 0;
    public static double apriltagTagSizeCoeff = 0;
    public static double apriltagTagSizeCoeffX = 0;
    public static double apriltagTagSizeCoeffY= 0;

    public static double apriltagTagSizeCoeffAngle = 0;







}
