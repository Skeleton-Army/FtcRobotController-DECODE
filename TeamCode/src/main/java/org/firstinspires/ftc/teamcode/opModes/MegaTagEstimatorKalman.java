package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/*
    Implements kalman filter with MegaTag1 and pinpoint localizer
    modelCovariance and dataCovariance need to be tuned

 */
public class MegaTagEstimatorKalman extends OpMode {
    Limelight3A limelight;
    Follower follower;

    TelemetryPacket telemetryPacket;
    FtcDashboard dashboard;

    final double modelCovariance = 0.05; // TODO: tune this for odometry drift
    final double dataCovariance = 0.5; // TODO: tune this for AprilTag noise

    Pose odometryDeltaPose = new Pose(0,0,0);

    KalmanFilter xFilter;
    KalmanFilter yFilter;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        telemetryPacket = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();



        //KalmanEstimator estimator = new KalmanEstimator();
        KalmanFilterParameters params = new KalmanFilterParameters(modelCovariance, dataCovariance);
         xFilter = new KalmanFilter(params);
         yFilter = new KalmanFilter(params);
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        odometryDeltaPose = follower.poseTracker.getDeltaPose();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles orientation = result.getBotpose().getOrientation();

            xFilter.update(odometryDeltaPose.getX(), botpose.x);
            yFilter.update(odometryDeltaPose.getY(), botpose.y);

            telemetryPacket.put("Apriltag x", botpose.x);
            telemetryPacket.put("Apriltag y", botpose.y);
            telemetryPacket.put("Apriltag z", botpose.z);
            telemetryPacket.put("Apriltag heading", orientation.getYaw(AngleUnit.DEGREES));

        } else if (!result.isValid()) {
            xFilter.update(odometryDeltaPose.getX(), xFilter.getState());
            yFilter.update(odometryDeltaPose.getY(), yFilter.getState());
        }

        telemetryPacket.put("Odometry x", follower.getPose().getX());
        telemetryPacket.put("Odometry y", follower.getPose().getY());
        telemetryPacket.put("Odometry heading", follower.getHeading());

        telemetryPacket.put("Filtered x", xFilter.getState());
        telemetryPacket.put("Filtered y", yFilter.getState());
        telemetryPacket.put("Filtered heading", follower.getHeading()); // just for now, It's pretty accurate, right?

        dashboard.sendTelemetryPacket(telemetryPacket);

    }
}
