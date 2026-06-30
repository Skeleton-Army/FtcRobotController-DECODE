package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.function.BooleanSupplier;

public class CloseCycleCommand extends SequentialCommandGroup {
    private static final Pose GATE_OPEN_POSE = new Pose(12, 57.5);
    private static final Pose NEAR_DRIVE_BACK = new Pose(50, 85);
    private static final double COLLECT_HEADING = Math.toRadians(154.8622);

    private final Transfer transfer;

    public CloseCycleCommand(
            Follower follower,
            Intake intake,
            Transfer transfer,
            Shooter shooter,
            Drive drive,
            Alliance alliance
    ) {
        this.transfer = transfer;

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
                                                    0.5,
                                                    HeadingInterpolator.tangent
                                            ),
                                            new HeadingInterpolator.PiecewiseNode(
                                                    0.5,
                                                    1,
                                                    HeadingInterpolator.constant(collectHeading)
                                            )
                                    )
                            )
                            .build();
                    return new FollowPathCommand(follower, collectPath);
                }, null),

                new WaitUntilCommand(artifactDetected())
                        .withTimeout(2000),

                new InstantCommand(intake::stop),

                new ParallelCommandGroup(
                        new DeferredCommand(() -> {
                            PathChain returnPath = follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(), nearDriveBack))
                                    .setTangentHeadingInterpolation()
                                    .setReversed()
                                    .build();
                            return new FollowPathCommand(follower, returnPath);
                        }, null),

                        new SequentialCommandGroup(
                                new WaitUntilCommand(drive::isInsideLaunchZonePredictive),
                                new ShootCommand(shooter, intake, transfer, drive),
                                new InstantCommand(follower::breakFollowing)
                        )
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

    private BooleanSupplier artifactDetected() {
        TimerEx debounce = new TimerEx(0.2);
        return () -> {
            if (transfer.isArtifactDetected() && transfer.isArtifactInIntake()) {
                if (!debounce.isOn()) debounce.restart();
                return debounce.isDone();
            } else {
                if (debounce.isOn()) debounce.pause();
                debounce.restart();
                debounce.pause();
                return false;
            }
        };
    }
}
