package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.opModes.TeleOpApp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;

@TeleOp
public class Zipi extends OpMode {
    private Vision vision;
    private Follower follower;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(TeleOpApp.X_OFFSET, TeleOpApp.Y_OFFSET, Math.toRadians(0)));
        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.DETECTION_PIPELINE);

        gamepadEx.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(
                        //TODO: make real command
                        new InstantCommand(() -> {
                            Artifact artifact = vision.artifactList().getClosest();
                            PathChain pathChain = buildPathFromArtifact(artifact);
                            follower.followPath(pathChain);
                        })
                );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();

        Vision.ArtifactList artifactList = vision.artifactList();
        if (artifactList != null && !artifactList.toList().isEmpty()) {
            telemetry.addData("tx ty", artifactList.getClosest().getTxtyPair().toString());
            telemetry.addData("abs pos", artifactList.getClosest());
        }
        telemetry.update();

    }

    private PathChain buildPathFromArtifact(Artifact artifact) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(135, artifact.getArtifactPose().getY())))
                .setGlobalDeceleration()
                .build();

    }

}
