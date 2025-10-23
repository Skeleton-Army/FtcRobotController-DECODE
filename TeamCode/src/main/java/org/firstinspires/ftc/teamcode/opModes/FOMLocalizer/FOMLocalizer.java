package org.firstinspires.ftc.teamcode.opModes.FOMLocalizer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class FOMLocalizer extends OpMode {
    Follower follower;
    Limelight3A limelight;

    TelemetryPacket telemetryPacket;
    FtcDashboard dashboard;


    // starting values
    double odometryFOM = 0.01;
    double apriltagFOM = 1;


    Pose MT2Pose;
    Pose poseFOM;
    IMU imu; // using the internal IMU for MT2

    FOMCalculator FOMcalculator;
    Pose3D cameraApriltagPose;

    MultipleTelemetry telemetry;
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        FOMcalculator = new FOMCalculator(follower);
    }

    @Override
    public void loop() {

        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        LLResult result = limelight.getLatestResult();
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(robotYaw);
        if (result.isValid()) {
            // Access general information

            Position botpose_mt2 = result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH);
            if (botpose_mt2 != null) {
                YawPitchRollAngles orientation = result.getBotpose_MT2().getOrientation();

                cameraApriltagPose = result.getFiducialResults().get(0).getCameraPoseTargetSpace();

                MT2Pose = new Pose(botpose_mt2.x, botpose_mt2.y, orientation.getYaw(AngleUnit.DEGREES));

                MT2Pose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, botpose_mt2.x, botpose_mt2.y, AngleUnit.DEGREES, orientation.getYaw()), MT2Pose.getCoordinateSystem());
                telemetry.addData("Apriltag x", botpose_mt2.x);
                telemetry.addData("Apriltag y", botpose_mt2.y);
                telemetry.addData("Apriltag z", botpose_mt2.z);
                telemetry.addData("Apriltag heading", orientation.getYaw(AngleUnit.DEGREES));

            }
            else {
                MT2Pose = new Pose(0,0,0);
            }
        }

        poseFOM = FOMcalculator.calcPos(MT2Pose, cameraApriltagPose);
        follower.setPose(poseFOM); // setting the calculated pose to the follower

        telemetry.addData("Robot x ", follower.getPose().getX());
        telemetry.addData("Robot y ", follower.getPose().getY());
        telemetry.addData("Robot heading ", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("mega tag2 x", MT2Pose.getX());
        telemetry.addData("mega tag2 y", MT2Pose.getY());
        telemetry.addData("mega tag2 heading", Math.toDegrees(MT2Pose.getHeading()));

        telemetry.addData("apriltag FOM", FOMcalculator.apriltagFOM);
        telemetry.addData("odometry FOM", FOMcalculator.odometryFOM);
        telemetry.addData("robot FOM ", FOMcalculator.robotFOM);

        telemetry.update();
    }
}
