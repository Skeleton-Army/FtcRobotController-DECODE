package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.X_OFFSET;
import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.Y_OFFSET;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

@Autonomous
public class DriftTest extends ComplexOpMode {
    private Follower follower;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0)));

        follower.followPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(X_OFFSET, Y_OFFSET),
                                        new Pose(38.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))




                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 33.5),
                                        new Pose(105.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 33.5),
                                        new Pose(105.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 107.5),
                                        new Pose(38.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 107.5),
                                        new Pose(38.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))

                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 33.5),
                                        new Pose(105.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 33.5),
                                        new Pose(105.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 107.5),
                                        new Pose(38.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 107.5),
                                        new Pose(38.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))




                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 33.5),
                                        new Pose(105.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 33.5),
                                        new Pose(105.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 107.5),
                                        new Pose(38.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 107.5),
                                        new Pose(38.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(270))

                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 33.5),
                                        new Pose(105.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 33.5),
                                        new Pose(105.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .addPath(
                                new BezierLine(
                                        new Pose(105.5, 107.5),
                                        new Pose(38.5, 107.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 107.5),
                                        new Pose(38.5, 33.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(270))




                        .addPath(
                                new BezierLine(
                                        new Pose(38.5, 33.5),
                                        new Pose(X_OFFSET, Y_OFFSET)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
        );
    }

    @Override
    public void run() {
        follower.update();

        double driftX = Math.abs(X_OFFSET - follower.getPose().getX());
        double driftY = Math.abs(Y_OFFSET - follower.getPose().getY());

        telemetry.addData("Follower X", follower.getPose().getX());
        telemetry.addData("Follower Y", follower.getPose().getY());
        telemetry.addData("Drift X", driftX);
        telemetry.addData("Drift Y", driftY);
        telemetry.addData("Drift Total", driftX + driftY);
        telemetry.update();
    }
}
