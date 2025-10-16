package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class AutonomousApp extends ComplexOpMode {
    private Follower follower;
    private Intake intake;
    private PathChain batata;
    private PathChain potato;
    private Shooter shooter;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.604751619870406,7.464362850971918, Math.toRadians(90)));

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, follower.poseTracker);

        batata = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.605, 7.464),
                                new Pose(60.959, 29.235),
                                new Pose(17.417, 36.078)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        potato = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(17.417, 36.078),
                                new Pose(60.959, 29.235),
                                new Pose(56.294, 14.307)

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    @Override
    public void onStart() {
        schedule(
                new SequentialCommandGroup(
                        new ShootCommand(shooter),
                        new InstantCommand(() -> intake.collect()),
                        new FollowPathCommand(follower, batata),
                        new InstantCommand(() -> intake.stop()),
                        new FollowPathCommand(follower, potato),
                        new ShootCommand(shooter)
                )
        );
    }

    @Override
    public void run() {
        follower.update();
    }
}
