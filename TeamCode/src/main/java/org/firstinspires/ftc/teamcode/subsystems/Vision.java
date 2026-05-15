package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.VisionConfig.*;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.enums.ArtifactPattern;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;

public class Vision extends SubsystemBase {
    private final double METERS_TO_INCHES = 39.37;
    private static final int GPP_TAG_ID = 21;
    private static final int PGP_TAG_ID = 22;
    private static final int PPG_TAG_ID = 23;
    private static final int FIELD_HALF_Y_LEVEL = 72;

    private final PoseTracker poseTracker;
    private final Limelight3A limelight;

    private final TimerEx relocalizeTimer = new TimerEx(RELOCALIZE_COOLDOWN);

    private final List<Consumer<Pose>> onRelocalizeListeners = new ArrayList<>();

    private boolean firstRelocalization = true;

    private final int pipeline;

    public Vision(HardwareMap hardwareMap, PoseTracker poseTracker, int pipelineIndex) {
        this.poseTracker = poseTracker;
        this.pipeline = pipelineIndex;

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(this.pipeline);
        limelight.start();

        relocalizeTimer.start();
    }

    @Override
    public void periodic() {
        double orientationDeg = Math.toDegrees(poseTracker.getPose().getHeading()) + 90;
        limelight.updateRobotOrientation(orientationDeg);

        // Check if it's the first run OR if the timer is done
        if (relocalizeTimer.isDone() || firstRelocalization) {
            boolean success = relocalize();
            if (success) {
                firstRelocalization = false;
                relocalizeTimer.restart();
            }
        }
    }

    public Pose getAprilTagPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();

            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

                Pose standardFTCPose = new Pose(x, y, heading).scale(METERS_TO_INCHES);
                Pose pedroPose = FTCCoordinates.INSTANCE.convertToPedro(standardFTCPose);

                return pedroPose;
            }
        }

        return new Pose();
    }

    public boolean relocalize() {
        if (pipeline == OBELISK_PIPELINE) return false;

        double velocity = poseTracker.getVelocity().getMagnitude();
        double angularVelocity = poseTracker.getAngularVelocity();
        if (Math.abs(velocity) > VELOCITY_THRESHOLD || Math.abs(angularVelocity) > VELOCITY_THRESHOLD) return false;

        Pose tagPose = getAprilTagPose();
        if (tagPose.roughlyEquals(new Pose(), 0.001)) return false;
        if (tagPose.getY() < FIELD_HALF_Y_LEVEL) return false;

        poseTracker.setPose(tagPose);

        for (Consumer<Pose> listener : onRelocalizeListeners) {
            listener.accept(tagPose);
        }

        return true;
    }

    public void addRelocalizationListener(Consumer<Pose> listener) {
        onRelocalizeListeners.add(listener);
    }

    /**
     * Reads the obelisk AprilTag from the latest Limelight result.
     * Returns null if no obelisk tag is currently visible.
     */
    public ArtifactPattern detectPattern() {
        if (pipeline == APRILTAG_PIPELINE) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            int id = fiducial.getFiducialId();
            if (id == GPP_TAG_ID) return ArtifactPattern.GPP;
            if (id == PGP_TAG_ID) return ArtifactPattern.PGP;
            if (id == PPG_TAG_ID) return ArtifactPattern.PPG;
        }

        return null;
    }

    public Pose getClosestArtifact() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            return new Pose(Double.NaN, Double.NaN);
        }

        Map<Double, Double> distanceMap = new TreeMap<>();
        List<LLResultTypes.DetectorResult> detectorResults = llResult.getDetectorResults();

        for (LLResultTypes.DetectorResult detectorResult : detectorResults) {
            distanceMap.put(getDistance(detectorResult.getTargetYDegrees()), detectorResult.getTargetXDegrees());
        }

        double distance = (double) distanceMap.keySet().toArray()[0];
        double tx;

        if (distanceMap.containsKey(distance)) {
            // should be safe
            tx = distanceMap.get(distance);
        } else {
            return new Pose(Double.NaN, Double.NaN);
        }

        Pose relativePose = getRelativePose(distance, tx);
        double theta = poseTracker.getPose().getHeading();

        double x = relativePose.getX() * Math.cos(theta) - relativePose.getY() * Math.sin(theta);
        double y = relativePose.getX() * Math.sin(theta) - relativePose.getY() * Math.cos(theta);

        return new Pose(poseTracker.getPose().getX() + x, poseTracker.getPose().getY() + y, 0);
    }

    private double getDistance (double ty) {
        double angleGoalRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE + ty);
        return (ARTIFACT_HEIGHT_FROM_FLOOR - LENS_HEIGHT_INCHES) / Math.tan(angleGoalRad);
    }

    private Pose getRelativePose(double distance, double tx) {
        double theta = Math.toRadians(tx);

        double deltaX = (distance + X_OFFSET_INCHES) * Math.cos(theta);
        double deltaY = (distance + Y_OFFSET_INCHES) * Math.sin(theta);

        return new Pose(deltaX, deltaY);
    }
}
