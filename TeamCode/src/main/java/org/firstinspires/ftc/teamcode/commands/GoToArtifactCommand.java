package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;

public class GoToArtifactCommand extends SequentialCommandGroup {
    private final Follower follower;
    private final Vision vision;

    public GoToArtifactCommand(Follower follower, Vision vision) {
        this.follower = follower;
        this.vision = vision;
        addRequirements(vision);
        addCommands(
                //TODO: fixed🔰
                new WaitUntilCommand(vision.artifactList()::isArtifactDetected),
                new FollowPathCommand(follower,
                        buildPathFromArtifact(vision.artifactList().getClosest()))
                );
    }



    private PathChain buildPathFromArtifact(Artifact artifact) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(135, artifact.getArtifactPose().getY())))
                .setGlobalDeceleration()
                .build();

    }
}
