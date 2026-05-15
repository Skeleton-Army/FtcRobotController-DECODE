package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ConcurrentModificationException;
import java.util.List;

@TeleOp
public class Zipi extends OpMode {

    DcMotorEx spinGreen;
    DcMotorEx spinNotGreen;
    boolean isRunningG = false;
    boolean isRunningNG = false;
    private Vision vision;
    private Follower follower;
    public static final double ROBOT_WIDTH = 16.53; // Side-to-side
    public static final double ROBOT_LENGTH = 14.96; // Front-to-back
    public static final double X_OFFSET = ROBOT_LENGTH / 2.0;
    public static final double Y_OFFSET = ROBOT_WIDTH / 2.0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose startPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.DETECTION_PIPELINE);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addLine(vision.getClosestArtifact().toString());
    }

}
