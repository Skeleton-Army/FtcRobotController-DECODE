package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

@Config
public class KalmanConfig {


    // the stdev-squared for each asix - x,y and heading
    public static Pose initialCovariance = new Pose(0.01, 0.01, Math.toRadians(0.5)); // the initial covariance for pinpoint - how much we trust our first positioning of the robot when placing it on the field
    public static Pose processVariance = new Pose(); // the drift covariance
    public static Pose measurementVariance = new Pose(); // the apriltag covariance
    public static int bufferSize = 64;
    public static double apriltagDistanceCoeff = 0;
    public static double apriltagTagSizeCoeff = 0;

}
