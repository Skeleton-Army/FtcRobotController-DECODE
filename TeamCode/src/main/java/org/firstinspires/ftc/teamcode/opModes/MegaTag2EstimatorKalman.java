package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
    Implements kalman filter with MegaTag2 and pinpoint localizer
    modelCovariance and dataCovariance need to be tuned

 */

public class MegaTag2EstimatorKalman extends OpMode {
    Limelight3A limelight;
    Follower follower;

    TelemetryPacket telemetryPacket;
    FtcDashboard dashboard;
    IMU imu;

    final double modelCovariance = 0.05; //TODO:  tune this for odometry drift
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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //KalmanEstimator estimator = new KalmanEstimator();
        KalmanFilterParameters params = new KalmanFilterParameters(modelCovariance, dataCovariance);
        xFilter = new KalmanFilter(params);
        yFilter = new KalmanFilter(params);
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


        LLResult result = limelight.getLatestResult();
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Position botpose_mt2 = result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH);
            if (botpose_mt2 != null) {
                double x = botpose_mt2.x;
                double y = botpose_mt2.y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                telemetryPacket.put("MT2 Location:", "(" + x + ", " + y + ")");

                xFilter.update(odometryDeltaPose.getX(), x);
                yFilter.update(odometryDeltaPose.getY(), y);

                telemetryPacket.put("Apriltag x", x);
                telemetryPacket.put("Apriltag y", y);
                telemetryPacket.put("Apriltag z", botpose_mt2.z);
                telemetryPacket.put("Apriltag heading", result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.DEGREES));
            }
        } else if (result == null || !result.isValid()) {
            xFilter.update(odometryDeltaPose.getX(), xFilter.getState());
            yFilter.update(odometryDeltaPose.getY(), yFilter.getState());
        }

        telemetryPacket.put("Odometry x", follower.getPose().getX());
        telemetry.addData("Odometry y", follower.getPose().getY());
        telemetryPacket.put("Odometry heading", follower.getHeading());


        telemetryPacket.put("Filtered x", xFilter.getState());
        telemetry.addData("Filtered y", yFilter.getState());
        telemetryPacket.put("Odometry heading", follower.getHeading()); // just for now, It's pretty accurate, right?

        telemetry.update();
        dashboard.sendTelemetryPacket(telemetryPacket);
    }
}
