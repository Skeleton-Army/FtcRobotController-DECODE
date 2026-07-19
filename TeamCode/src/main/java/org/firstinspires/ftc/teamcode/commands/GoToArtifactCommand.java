package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;

public class GoToArtifactCommand extends SequentialCommandGroup {
    private final Follower follower;
    private final Vision vision;
    private final Alliance alliance;
    public GoToArtifactCommand(Follower follower, Vision vision, Alliance alliance) {
        this.follower = follower;
        this.vision = vision;
        this.alliance = alliance;

        Vision.ArtifactList artifactList = vision.artifactList();
        addRequirements(vision);
        addCommands(
                //TODO: fixed 🔰
                new WaitUntilCommand(() ->
                       artifactList.fetch()
                               .filterInvalidX()
                               .filterByYLevel(0, 90)
                               .isArtifactDetected()
                ),
                //new InstantCommand(() -> telemetry.addData("artifactPose", artifactList.getClosest()))
                new DeferredCommand(
                        () ->   new FollowPathCommand(follower,
                                buildPathFromArtifact(artifactList.getBiggest())),
                        null
                ).withTimeout(1500)
        );
    }

    private PathChain buildPathFromArtifact(Artifact artifact) {
        Pose correctedPose = getCorrectedPose(artifact);
        Pose artifactPose = new Pose(correctedPose.getX(), correctedPose.getY(), Math.toRadians(180));

        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), artifactPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(180)))
                                )
                        )
                )
                .setGlobalDeceleration()
                .build();
    }

    private Pose getCorrectedPose(Artifact artifact) {
        Pose pose = new Pose(10, artifact.getArtifactPose().getY(), Math.toRadians(180));

        if (alliance == Alliance.RED) {
            pose = pose.mirror(GoalPositions.FIELD_LENGTH);
        }

        return pose;
    }

    private double getRelative(double headingRad) {
        if (alliance == Alliance.RED) {
            return MathFunctions.normalizeAngle(Math.PI - headingRad);
        }

        return headingRad;
    }
}
