package org.firstinspires.ftc.teamcode.opModes.FOMLocalizer;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class FOMCalculator {

    Follower follower;
    Limelight3A limelight;

    double odometryFOM;
    double apriltagFOM;

    public double robotFOM;
    public FOMCalculator(Follower follower, Limelight3A limelight) {
        this.follower= follower;
        this.limelight = limelight;

    }

    /* calculating the apriltag FOM according to:
        - robot's current speed (high speed -> higher FOM)
        - if the tag is close
        - if it is centered

     */
    public void calcApriltagFOM() {


    }

     /*
        calculating the pinpoint/odometry FOM according to:
            - the distance traveled (higher distance -> increase FOM)
            - collision/skidding (may not be the case for us)

      */

    public void calcOdometryFOM() {


    }

    // called every loop
    public Pose calcPos(Pose limelightPose) {

        calcOdometryFOM();
        calcApriltagFOM();
        robotFOM = Math.min(odometryFOM,apriltagFOM);

        Pose limelightFOMPose = limelightPose.times(1/Math.pow(apriltagFOM, 2));
        Pose odometryFOMPose = follower.getPose().times(1/Math.pow(odometryFOM, 2));

        return (limelightPose.plus(odometryFOMPose)).div(Math.pow(odometryFOM + apriltagFOM, 2));

    }

}
