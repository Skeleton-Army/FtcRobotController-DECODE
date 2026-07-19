package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.config.VisionConfig.AVERAGE_APPROACH_VELOCITY;
import static org.firstinspires.ftc.teamcode.config.VisionConfig.PREDICTION_ITERATIONS;
import static org.firstinspires.ftc.teamcode.utilities.Artifact.predictPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.Alliance;
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
                               .filterByYLevel(0, 90)
                               .isArtifactDetected()
                ),
                new DeferredCommand(
                        () ->   new FollowPathCommand(follower,
                                buildPathFromArtifact(artifactList.getBiggest())),
                        null
                ).withTimeout(1500)
        );
    }

    private PathChain buildPathFromArtifact(Artifact artifact) {
        Pose predictedArtifactPose = predictInterceptPose(artifact);
        Pose correctedPose = getCorrectedPose(predictedArtifactPose);
        Pose artifactPose = new Pose(correctedPose.getX(), correctedPose.getY());

        if (Settings.get("debug_mode", false)) {
            Pose rotatedPose = artifactPose.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            telemetry.addData("Artifact x", -rotatedPose.getX());
            telemetry.addData("Artifact y", -rotatedPose.getY());
            telemetry.addData("Artifact heading", artifact.getVelocityY());
            telemetry.addData("Vel", artifact.getVelocity());
            telemetry.addData("VelX", artifact.getVelocityX());
            telemetry.addData("VelY", artifact.getVelocityY());

            telemetry.addData("Artifact Pedro X", artifactPose.getX());
            telemetry.addData("Artifact Pedro Y", artifactPose.getY());

        }

        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), artifactPose))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();
    }

    private Pose predictInterceptPose(Artifact artifact) {
        Pose predictedPose = artifact.getPose();

        for (int i = 0; i < PREDICTION_ITERATIONS; i++) {
            double distance = follower.getPose().distanceFrom(predictedPose);
            double estimatedTravelTime = distance / AVERAGE_APPROACH_VELOCITY;
            predictedPose = predictPose(artifact, estimatedTravelTime);
        }

        return predictedPose;
    }

    private Pose getCorrectedPose(Pose predictedArtifactPose) {
        Pose pose = new Pose(predictedArtifactPose.getX(), predictedArtifactPose.getY());
        //Pose pose = new Pose(10, predictedArtifactPose.getY(), Math.toRadians(180));
        if (alliance == Alliance.RED) {
            //TODO: fix later for shikago //TODO: fix name for Chicago //TODO: remove commant after sheekygu //TODO: fixed 🔰
            pose = pose.mirror(GoalPositions.FIELD_LENGTH);
        }

        return pose;
    }

}
