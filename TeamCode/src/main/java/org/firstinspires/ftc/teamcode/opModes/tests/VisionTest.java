package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.FORWARD_SPEED;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.SHOOTING_FORWARD_SPEED;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.SHOOTING_STRAFE_SPEED;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.SHOOTING_TURN_SPEED;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.STRAFE_SPEED;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.X_OFFSET;
import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.Y_OFFSET;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.GoToArtifactCommand;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

import java.util.concurrent.atomic.AtomicReference;

import lombok.var;

@TeleOp
public class VisionTest extends ComplexOpMode {
    Follower follower;
    Vision vision;
    GamepadEx gamepadEx;
    boolean isTeleopDrive = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.DETECTION_PIPELINE);
        Pose stat = new Pose(189.5 - X_OFFSET, Y_OFFSET, Math.toRadians(180));
        follower.setPose(stat);

        gamepadEx = new GamepadEx(gamepad1);

        AtomicReference<Pose> detectPose = new AtomicReference<>(new Pose());

        gamepadEx.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    detectPose.set(follower.getPose());
                                    isTeleopDrive = false;
                                }),
                                new GoToArtifactCommand(follower, vision, Alliance.RED)
                        )
                );
    }

    @Override
    public void run() {
        follower.update();

        if (gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_x > 0.1 || gamepad1.right_stick_x > 0.1) {
            if (!isTeleopDrive) {
                follower.startTeleopDrive(true);
                isTeleopDrive = true;
            }
        }

        if (isTeleopDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }

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

            /*
            Pose predictedPose = predictInterceptPose(biggest);
            telemetry.addData("predictedX", predictedPose.getX());
            telemetry.addData("predictedY", predictedPose.getY());
            //telemetry.addData("go to pose x", Artifact.predictPose())
             */

            var prev = artifactList.prevList().get(0);

            telemetry.addData("prevX", prev.getPose().getX());
            telemetry.addData("prevY", prev.getPose().getY());
            telemetry.addData("prevVelX", prev.getVelocityX());
            telemetry.addData("prevVelY", prev.getVelocityY());

        }

        telemetry.update();
    }

}