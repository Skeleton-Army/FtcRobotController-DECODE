package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class CloseCycleCommand extends SequentialCommandGroup {

    // Intermediate Bezier control point shared by both path legs
    private static final Pose MID_CONTROL_POINT_BASE = new Pose(46.462, 64.985);

    // Gate collection target
    private static final Pose GATE_OPEN_POSE_BASE = new Pose(14.5721, 58.82221);

    // Shooting position to return to after collecting
    private static final Pose NEAR_DRIVE_BACK_BASE = new Pose(50, 90);

    // Heading facing the gate collection zone
    private static final double COLLECT_HEADING_BASE = Math.toRadians(154.8622);

    // Heading facing the shooting zone
    private static final double RETURN_HEADING_BASE = Math.toRadians(180);

    public CloseCycleCommand(
            Follower follower,
            Intake intake,
            Transfer transfer,
            Shooter shooter,
            Drive drive,
            Alliance alliance
    ) {
        final Pose midControl    = relative(MID_CONTROL_POINT_BASE, alliance);
        final Pose gateOpenPose  = relative(GATE_OPEN_POSE_BASE, alliance);
        final Pose nearDriveBack = relative(NEAR_DRIVE_BACK_BASE, alliance);
        final double collectHeading = relative(COLLECT_HEADING_BASE, alliance);
        final double returnHeading  = relative(RETURN_HEADING_BASE, alliance);

        addRequirements(drive, intake);

        addCommands(
                new InstantCommand(intake::collect),

                new DeferredCommand(() -> {
                    PathChain collectPath = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                    follower.getPose(),
                                    midControl,
                                    gateOpenPose
                            ))
                            .setLinearHeadingInterpolation(follower.getHeading(), collectHeading)
                            .build();
                    return new FollowPathCommand(follower, collectPath);
                }, null),

                new WaitUntilCommand(
                        () -> transfer.isArtifactDetected() && transfer.isArtifactInIntake()
                ).withTimeout(2000),

                new InstantCommand(intake::stop),

                new DeferredCommand(() -> {
                    PathChain returnPath = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                    follower.getPose(),
                                    midControl,
                                    nearDriveBack
                            ))
                            .setLinearHeadingInterpolation(follower.getHeading(), returnHeading)
                            .build();
                    return new FollowPathCommand(follower, returnPath);
                }, null),

                new InstantCommand(intake::stop),
                new ShootCommand(shooter, intake, transfer, drive)
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
