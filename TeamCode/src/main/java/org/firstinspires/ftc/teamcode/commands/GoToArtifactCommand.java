package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.opModes.TeleOpApp;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;

@Config
public class GoToArtifactCommand extends SequentialCommandGroup {
    private final Follower follower;
    private final Alliance alliance;
    private final Telemetry telemetry;

    public GoToArtifactCommand(Follower follower, Vision vision, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
        telemetry = FtcDashboard.getInstance().getTelemetry();

        Vision.ArtifactList artifactList = vision.artifactList();
        addRequirements(vision);
        addCommands(
                //TODO: fixed 🔰
                new WaitUntilCommand(() ->
                        artifactList.fetch()
                                .filterInvalidX()
                                .filterByYLevel(0,0) // add correct y cords
                                .isArtifactDetected()
                ),
                new DeferredCommand(
                        () ->   new FollowPathCommand(follower,
                                buildPathFromArtifact(artifactList.getBiggest())),
                        null
                )
        );
    }

    private PathChain buildPathFromArtifact(Artifact artifact) {
        Pose correctedPose = getCorrectedPose(artifact);
        Pose artifactPose = new Pose(correctedPose.getX(), correctedPose.getY());

        telemetry.addData("Artifact x", artifactPose.getX());
        telemetry.addData("Artifact y", artifactPose.getY());

        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), artifactPose))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();

    }

    private Pose getCorrectedPose(Artifact artifact) {
        Pose pose = new Pose(10, artifact.getArtifactPose().getY(), Math.toRadians(180));
        if (alliance == Alliance.RED) {
            pose = pose.mirror();
        }

        return pose;
    }
}
