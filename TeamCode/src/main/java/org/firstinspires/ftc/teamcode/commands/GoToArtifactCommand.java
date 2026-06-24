package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.OpModeManager;

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
                //TODO: fixed🔰
                new WaitUntilCommand(() ->
                       artifactList.fetch().isArtifactDetected()
                ),
                new DeferredCommand(
                        () ->   new FollowPathCommand(follower,
                                buildPathFromArtifact(artifactList.getClosest())),
                        null
                )

        );
    }



    private PathChain buildPathFromArtifact(Artifact artifact) {
        Pose artifactPose = new Pose(10, artifact.getArtifactPose().getY(), Math.toRadians(180));
        if (alliance == Alliance.RED) {
            artifactPose = artifactPose.mirror();
        }

        Log.i("VISION",artifactPose.toString());

        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), artifactPose))
                .setConstantHeadingInterpolation(
                        artifactPose.getHeading()
                )
                .setGlobalDeceleration()
                .build();

    }
}
