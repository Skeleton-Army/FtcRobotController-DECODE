package org.firstinspires.ftc.teamcode.opModes.FOMLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Mat;

@Config
public class FOMCalculator {

    Follower follower;
    Limelight3A limelight;

    public double odometryFOM = 0.01;
    public double apriltagFOM;
    public double robotFOM;

    public static double maxVel = 20; // TODO: see what this is

    // TODO: Tune these values
    public static double apriltagFOMVelCoeff = 1; // 1 means that if the robot travels at full speed, the FOM will increase by 1
    public static double apriltagFOMDistanceCoeff = 162; // this is simply a guess, tune it a bit (might not be linear)
    // the FOM for (0,0) will be 0.5 (I hope this is the case)

    public static double odometryFOMtravelCoeff = 750; // TODO: measure this value, by seeing how much the robot traveled in the whole game
    double xDeltaSum = 0;
    double yDeltaSum = 0;
    public FOMCalculator(Follower follower) {
        this.follower= follower;

    }

    /* calculating the apriltag FOM according to:
        - robot's current speed (high speed -> higher FOM)
        - distance from the tag
        - if it is centered

     */
    public void calcApriltagFOM(Pose limelightPose, Pose3D cameraApriltagPose) {
        if (!limelightPose.equals(new Pose(0,0,0))) {
            // the relative distace to the detected tag
            double distanceToTag = Math.sqrt(Math.pow(cameraApriltagPose.getPosition().x, 2) + Math.pow(cameraApriltagPose.getPosition().y, 2));

            apriltagFOM = distanceToTag/apriltagFOMDistanceCoeff + apriltagFOMVelCoeff*follower.getVelocity().getMagnitude()/ maxVel;

            return;
        }

        apriltagFOM = Double.POSITIVE_INFINITY; // simply ignore the apriltag bc it isn't being detected

    }

     /*
        calculating the pinpoint/odometry FOM according to:
            - the distance traveled (higher distance -> increase FOM)
            - collision/skidding (may not be the case for us)

      */

    public void calcOdometryFOM() {
        // how much we moved in x and y axes
        xDeltaSum += Math.abs(follower.poseTracker.getDeltaPose().getX());
        yDeltaSum += Math.abs(follower.poseTracker.getDeltaPose().getY());

        odometryFOM += (xDeltaSum + yDeltaSum) / odometryFOMtravelCoeff;

    }

    // called every loop
    public Pose calcPos(Pose limelightPose, Pose3D cameraApriltagPose) {

        calcOdometryFOM();
        calcApriltagFOM(limelightPose, cameraApriltagPose);
        robotFOM = Math.min(odometryFOM,apriltagFOM);

        Pose limelightFOMPose = limelightPose.times(1/Math.pow(apriltagFOM, 2));
        Pose odometryFOMPose = follower.getPose().times(1/Math.pow(odometryFOM, 2));

        return (limelightFOMPose.plus(odometryFOMPose)).div(Math.pow(1/odometryFOM, 2) + Math.pow(1/apriltagFOM, 2));

    }

}
