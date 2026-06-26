package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class CloseCycleCommand extends SequentialCommandGroup {
    private static final Pose GATE_OPEN_POSE = new Pose(13.5, 58.82221);
    private static final Pose NEAR_DRIVE_BACK = new Pose(50, 85);
    private static final double COLLECT_HEADING = Math.toRadians(154.8622);

    public CloseCycleCommand(
            Follower follower,
            Intake intake,
            Transfer transfer,
            Shooter shooter,
            Drive drive,
            Alliance alliance
    ) {
        final Pose gateOpenPose  = relative(GATE_OPEN_POSE, alliance);
        final Pose nearDriveBack = relative(NEAR_DRIVE_BACK, alliance);
        final double collectHeading = relative(COLLECT_HEADING, alliance);

        addRequirements(drive, intake);

        addCommands(
                new InstantCommand(intake::collect),

                new DeferredCommand(() -> {
                    PathChain collectPath = follower.pathBuilder()
                            .addPath(new BezierLine(
                                    follower.getPose(),
                                    gateOpenPose
                            ))
                            .setHeadingInterpolation(
                                    HeadingInterpolator.piecewise(
                                            new HeadingInterpolator.PiecewiseNode(
                                                    0,
                                                    0.8,
                                                    HeadingInterpolator.tangent
                                            ),
                                            new HeadingInterpolator.PiecewiseNode(
                                                    0.8,
                                                    1,
                                                    HeadingInterpolator.constant(collectHeading)
                                            )
                                    )
                            )
                            .build();
                    return new FollowPathCommand(follower, collectPath);
                }, null),

                new WaitUntilCommand(
                        () -> transfer.isArtifactDetected() && transfer.isArtifactInIntake()
                ).withTimeout(2000),

                new InstantCommand(intake::stop),

                // Shoot immediately when entering the launch zone and cancel path when done shooting
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(drive::isInsideLaunchZonePredictive),
                                new ShootCommand(shooter, intake, transfer, drive)
                        ),

                        new DeferredCommand(() -> {
                            PathChain returnPath = follower.pathBuilder()
                                    .addPath(new BezierLine(
                                            follower.getPose(),
                                            nearDriveBack
                                    ))
                                    .setTangentHeadingInterpolation()
                                    .setReversed()
                                    .build();
                            return new FollowPathCommand(follower, returnPath);
                        }, null)
                )
        );
    }

    private static Pose relative(Pose pose, Alliance alliance) {
        return (alliance == Alliance.RED) ? pose.mirror() : pose;
    }

    private static double relative(double headingRad, Alliance alliance) {
        return (alliance == Alliance.RED)
                ? MathFunctions.normalizeAngle(Math.PI - headingRad)
                : headingRad;
    }
}
