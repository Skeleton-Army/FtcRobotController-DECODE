package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

@Autonomous(name = "Straight Line Test")
public class StraightLineTest extends ComplexOpMode {

    private Follower follower;
    private PathChain path;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(38.63157894736841, 33.578947368421055, Math.toRadians(180));
        follower.setStartingPose(startPose);

        path = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                new Pose(39.89473684210527, 127.8947368421053)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public void onStart() {
        // Start following the path
        follower.followPath(path);
    }

    @Override
    public void run() {
        // MUST be called every loop
        follower.update();
    }
}
