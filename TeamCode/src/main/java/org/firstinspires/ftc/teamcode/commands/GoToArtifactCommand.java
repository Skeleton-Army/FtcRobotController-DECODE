package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.config.VisionConfig.AVERAGE_APPROACH_VELOCITY;
import static org.firstinspires.ftc.teamcode.config.VisionConfig.MAX_INTERCEPT_TIME;
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
        Pose predictedArtifactPose = predictInterceptPose(artifact);
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

    private Pose predictInterceptPose(Artifact artifact) {
        Pose robotPose = follower.getPose();
        Pose artifactPose = artifact.getPose();

        double dx = artifactPose.getX() - robotPose.getX();
        double dy = artifactPose.getY() - robotPose.getY();
        double vx = artifact.getVelocityX();
        double vy = artifact.getVelocityY();

        double t = solveInterceptTime(dx, dy, vx, vy);

        return predictPose(artifact, t);
    }

    private double solveInterceptTime(double dx, double dy, double vx, double vy) {
        double S = AVERAGE_APPROACH_VELOCITY;

        double a = (vx * vx + vy * vy) - (S * S);
        double b = 2 * (dx * vx + dy * vy);
        double c = dx * dx + dy * dy;

        double t;

        if (Math.abs(a) < 1e-6) {
            // |v| ≈ S: quadratic term vanishes, solve the remaining linear equation
            t = Math.abs(b) < 1e-6 ? 0 : -c / b;
        } else {
            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0) {
                // No real solution — e.g. artifact outrunning our approach speed.
                // Don't extrapolate wildly; just aim at its current position.
                t = 0;
            } else {
                double sqrtDisc = Math.sqrt(discriminant);
                double t1 = (-b + sqrtDisc) / (2 * a);
                double t2 = (-b - sqrtDisc) / (2 * a);

                t = Double.POSITIVE_INFINITY;
                if (t1 >= 0) t = Math.min(t, t1);
                if (t2 >= 0) t = Math.min(t, t2);
                if (Double.isInfinite(t)) t = 0;
            }
        }

        // Safety clamp: a noisy velocity spike should never be able to push the
        // predicted pose an unreasonable distance ahead of where the artifact
        // actually is.
        return Math.max(0, Math.min(t, MAX_INTERCEPT_TIME));
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