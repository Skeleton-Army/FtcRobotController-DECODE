package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.config.VisionConfig.APPROACH_TIME;
import static org.firstinspires.ftc.teamcode.config.VisionConfig.MIN_DETECTION_CYCLES;
import static org.firstinspires.ftc.teamcode.utilities.Artifact.predictPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

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
    private int consecutiveDetections = 0;

    public GoToArtifactCommand(Follower follower, Vision vision, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
        telemetry = FtcDashboard.getInstance().getTelemetry();

        Vision.ArtifactList artifactList = vision.artifactList();
        addRequirements(vision);

        addCommands(
                //TODO: fixed 🔰
                new WaitUntilCommand(() -> {
                    boolean detected = artifactList.fetch()
                            .filterInvalidX()
                            .filterByYLevel(0, 90)
                            .isArtifactDetected();

                    consecutiveDetections = detected ? consecutiveDetections + 1 : 0;

                    return consecutiveDetections >= MIN_DETECTION_CYCLES;
                }),
                new DeferredCommand(
                        () ->   new FollowPathCommand(follower,
                                buildPathFromArtifact(artifactList.getBiggest())),
                        null
                ).withTimeout(1500)
        );
    }

    @Override
    public void initialize() {
        this.consecutiveDetections = 0;
        super.initialize();
    }

    private PathChain buildPathFromArtifact(Artifact artifact) {
        Pose predictedArtifactPose = predictPose(artifact, APPROACH_TIME);
        Pose correctedPose = getCorrectedPose(predictedArtifactPose);
        if (correctedPose.getY() < 8) {
            correctedPose = new Pose(correctedPose.getX(), 8);
        }
        Pose artifactPose = new Pose(correctedPose.getX(), correctedPose.getY());

        Pose rotatedPose = artifactPose.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        telemetry.addData("Artifact x", -rotatedPose.getX());
        telemetry.addData("Artifact y", -rotatedPose.getY());
        telemetry.addData("Artifact heading", artifact.getVelocityY());
        telemetry.addData("Vel", artifact.getVelocity());
        telemetry.addData("VelX", artifact.getVelocityX());
        telemetry.addData("VelY", artifact.getVelocityY());

        telemetry.addData("Artifact Pedro X", artifactPose.getX());
        telemetry.addData("Artifact Pedro Y", artifactPose.getY());

        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), artifactPose))
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
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(180)))
                                )
                        )
                )
                .setGlobalDeceleration()
                .build();
    }

    private Pose getCorrectedPose(Pose predictedArtifactPose) {
        Pose pose = new Pose(10, predictedArtifactPose.getY(), Math.toRadians(180));
        if (alliance == Alliance.RED) {
            //TODO: fix later for shikago //TODO: fix name for Chicago //TODO: remove commant after sheekygu //TODO: fixed 🔰
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