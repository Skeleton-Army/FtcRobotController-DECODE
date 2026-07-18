package org.firstinspires.ftc.teamcode.commands;

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
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;

@Config
public class GoToArtifactCommand extends SequentialCommandGroup {
    // Max estimated artifact speed (inches/sec) we're still willing to treat as "stationary".
    // Tune alongside Vision.MAX_ARTIFACT_MATCH_DISTANCE.
    public static double MAX_ARTIFACT_VELOCITY = 1.0;

    // Rough average speed (inches/sec) the robot travels while approaching an artifact.
    // Used only to estimate how long the drive will take, so we know how far ahead
    // to project the artifact's position. Tune to roughly match real approach speed.
    public static double AVERAGE_APPROACH_VELOCITY = 20.0;

    // Number of fixed-point iterations used to converge the predicted intercept pose
    // (predicted position -> travel time -> new predicted position -> ...).
    public static int PREDICTION_ITERATIONS = 5;

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

        Telemetry telemetry = OpModeManager.getTelemetry();
        Pose rotatedPose = artifactPose.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        telemetry.addData("Artifact x", -rotatedPose.getX());
        telemetry.addData("Artifact y", -rotatedPose.getY());
        telemetry.addData("Artifact heading", artifact.getVelocityY());
        telemetry.addData("Vel", artifact.getVelocity());
        telemetry.addData("VelX", artifact.getVelocityX());
        telemetry.addData("VelY", artifact.getVelocityY());

        telemetry.addData("Artifact Pedro X", artifactPose.getX());
        telemetry.addData("Artifact Pedro Y", artifactPose.getY());

        //return follower.pathBuilder().build();


        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), artifactPose))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();



    }

    /**
     * Projects the artifact forward along its tracked velocity vector to where it's
     * expected to be by the time the robot actually gets there, instead of chasing
     * its last-seen (and by-then stale) position.
     *
     * Since the travel time itself depends on the distance to the (still unknown)
     * predicted position, this resolves the two via a few fixed-point iterations:
     * guess a position, estimate travel time to it, get a new position, repeat.
     */
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
            // fix later for shikago //TODO fix name for Chicago //TODO: remove commant after sheekygu //TODO: fixed
            pose = pose.mirror(GoalPositions.FIELD_LENGTH);
        }

        return pose;
    }

}
