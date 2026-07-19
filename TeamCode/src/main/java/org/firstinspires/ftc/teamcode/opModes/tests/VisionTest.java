package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.utilities.Artifact.predictPose;

import android.text.method.TextKeyListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.GoToArtifactCommand;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.opModes.TeleOpApp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.firstinspires.ftc.teamcode.utilities.Kinematics;

import java.util.Arrays;

import lombok.var;

@TeleOp
public class VisionTest extends ComplexOpMode {
    Follower follower;
    Vision vision;
    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.DETECTION_PIPELINE);
        Pose stat = new Pose(0, 0, Math.toRadians(0));
        follower.setPose(stat);

        gamepadEx = new GamepadEx(gamepad1);

        gamepadEx.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new GoToArtifactCommand(follower, vision, Alliance.RED));

        gamepadEx.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new FollowPathCommand(follower,
                        follower.pathBuilder().addPath(
                                new BezierLine(follower.getPose(), new Pose(10, 10))
                        ).build()));
    }

    @Override
    public void run() {
        follower.update();

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        telemetry.addData("Robot x", -rotatedPose.getX());
        telemetry.addData("Robot y", -rotatedPose.getY());
        telemetry.addData("Robot heading", rotatedPose.getHeading() - Math.PI);

        telemetry.addData("Pedro X", follower.getPose().getX());
        telemetry.addData("Pedro Y", follower.getPose().getY());

        Vision.ArtifactList artifactList = vision.artifactList();
        if (artifactList.isArtifactDetected()) {
            Artifact biggest = artifactList.getBiggest();

            telemetry.addData("biggestX", biggest.getPose().getX());
            telemetry.addData("biggestY", biggest.getPose().getY());
            telemetry.addData("biggestVelX", biggest.getVelocityX());
            telemetry.addData("biggestVelY", biggest.getVelocityY());
        }

        telemetry.update();
    }
}