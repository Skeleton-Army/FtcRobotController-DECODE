package org.firstinspires.ftc.teamcode.subsystems;
//TODO: add max width to Pipeline
import static org.firstinspires.ftc.teamcode.config.VisionConfig.*;

import android.content.Context;
import android.content.Intent;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.enums.ArtifactColor;
import org.firstinspires.ftc.teamcode.enums.ArtifactPattern;
import org.firstinspires.ftc.teamcode.enums.ArtifactSorting;
import org.firstinspires.ftc.teamcode.utilities.Artifact;
import org.firstinspires.ftc.teamcode.utilities.Kinematics;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;

import lombok.var;

public class Vision extends SubsystemBase {
    private static final double METERS_TO_INCHES = 39.37;
    private static final int GPP_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int PPG_TAG_ID = 21;
    private static final int FIELD_HALF_Y_LEVEL = 94;

    private final PoseTracker poseTracker;
    private final Limelight3A limelight;
    private LLResult llResult;

//    private final TimerEx relocalizeTimer = new TimerEx(RELOCALIZE_COOLDOWN);
    private final List<Consumer<Pose>> onRelocalizeListeners = new ArrayList<>();

//    private boolean firstRelocalization = true;

    private final int pipeline;

    // ─── Artifact velocity tracking ─────────────────────────────────────────────
    private List<Artifact> previousArtifacts = new ArrayList<>(); //do not use outside fetch() or estimateVelocities(). needs to be part of the bigger class to preserve state
    private long previousFetchTimeNanos = -1;
    private double[] previousPythonOutput = null; //small optimization to avoid recalculating if nothing changed

    // ────────────────────────────────────────────────────────────────────────────

    public Vision(HardwareMap hardwareMap, PoseTracker poseTracker, int pipelineIndex) {
        this.poseTracker = poseTracker;
        this.pipeline = pipelineIndex;

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(this.pipeline);
        limelight.start();
        llResult = null;

//      relocalizeTimer.start();
    }

    @Override
    public void periodic() {
        llResult = limelight.getLatestResult();
        /*
        relocalization isn't in the LL anymore
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
        */
    }

    public Pose getAprilTagPose() {
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

                Pose standardFTCPose = new Pose(x, y, heading).scale(METERS_TO_INCHES);

                return FTCCoordinates.INSTANCE.convertToPedro(standardFTCPose);
            }
        }

        return new Pose();
    }

    @Deprecated
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

        llResult = limelight.getLatestResult();
        //this method runs before the loop, thus needs explicit redefinition of llResult
        if (llResult == null || !llResult.isValid()) return null;

        for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
            int id = fiducial.getFiducialId();
            if (id == GPP_TAG_ID) return ArtifactPattern.GPP;
            if (id == PGP_TAG_ID) return ArtifactPattern.PGP;
            if (id == PPG_TAG_ID) return ArtifactPattern.PPG;
        }

        return null;
    }

    /* Proud of this one so I keep it, despite being kinda useless

    pwease don't delete 👉👈🥺

    public Pose getClosestArtifact() {
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
     */

    public ArtifactList artifactList() {
        return new ArtifactList().fetch();
    }

    @SuppressWarnings("unused")
    public class ArtifactList {
        private final List<Artifact> artifacts;

        //so only build with method in Vision class
        private ArtifactList() {
            artifacts = new ArrayList<>();
        }

        // ─── Population ──────────────────────────────────────────────────────────

        /**
         * Fetches and stores artifacts from the latest Limelight Python output.
         * Always call this first before chaining any sort/filter/terminal methods.
         */

        public ArtifactList fetch() {
            artifacts.clear(); // this is called every poll (e.g. from WaitUntilCommand), so we must not accumulate stale results
            if (llResult == null) return this;

            double[] output = llResult.getPythonOutput();

            if (previousPythonOutput != null && Arrays.equals(output, previousPythonOutput)) {
                artifacts.addAll(previousArtifacts);
                return this;
            }
            previousPythonOutput = output;

            long now = System.nanoTime();
            double dt_sec = previousFetchTimeNanos < 0 ? -1 : (now - previousFetchTimeNanos) / 1e9;

            List<Artifact> freshArtifacts = new ArrayList<>();

            int count = (int) output[0];
            for (int i = 0; i < count * 3; i += 3) {
                double tx = output[1 + i];
                double ty = output[2 + i];
                double ta = output[3 + i];
                Pose absPose = getAbsolutePosition(tx, ty);
                freshArtifacts.add(new Artifact(absPose, ta));
            }

            List<Artifact> trackedArtifacts = estimateVelocities(freshArtifacts, dt_sec);

            artifacts.addAll(trackedArtifacts);
            previousArtifacts = trackedArtifacts;
            previousFetchTimeNanos = now;

            return this;
        }

        // ─── Sorting ─────────────────────────────────────────────────────────────

        /** Sorts by straight-line distance from the robot's pose (closest first). */
        public ArtifactList sortByDistance() {
            Pose robotPose = poseTracker.getPose();
            artifacts.sort(Comparator.comparingDouble(a ->
                    Math.hypot(
                            a.getPose().getX() - robotPose.getX(),
                            a.getPose().getY() - robotPose.getY()
                    )
            ));
            return this;
        }

        /** Sorts by color ordinal ascending (PURPLE → GREEN → MIXED → UNKNOWN). */
        public ArtifactList sortByColor() {
            artifacts.sort(Comparator.comparingInt(a -> a.getColor().ordinal()));
            return this;
        }

        public ArtifactList sortBySize() {
            artifacts.sort(Comparator.comparingDouble(Artifact::getSize));
            return this;
        }

        /** Dispatches to the appropriate sort method via {@link ArtifactSorting}. */
        public ArtifactList sortBy(ArtifactSorting sorting) {
            switch (sorting) {
                case DISTANCE: return sortByDistance();
                case COLOR:    return sortByColor();
                case SIZE:     return sortBySize();
                default:       return this;
            }
        }

        // ─── Filtering ───────────────────────────────────────────────────────────

        /** Keeps only artifacts whose color matches the given {@link ArtifactColor}. */
        public ArtifactList filterByColor(ArtifactColor color) {
            artifacts.removeIf(a -> a.getColor() != color);
            return this;
        }

        public ArtifactList filterStationary(double maxVelocity) {
            artifacts.removeIf(a -> !a.isMoving(maxVelocity));
            return this;
        }

        public ArtifactList filterByYLevel(double minY, double maxY) {
            artifacts.removeIf(a -> a.getPose().getY() < minY);
            artifacts.removeIf(a -> a.getPose().getY() > maxY);
            return this;
        }

        public ArtifactList filterInvalidX() {
            artifacts.removeIf(a -> a.getPose().getX() > 188 || a.getPose().getX() < 0);
            return this;
        }

        // ─── Terminals ───────────────────────────────────────────────────────────

        /** Returns the closest artifact, or {@code null} if the list is empty. */
        public Artifact getClosest() {
            if (artifacts.isEmpty()) return null;
            return sortByDistance().artifacts.get(0);
        }

        public Artifact getBiggest() {
            if (artifacts.isEmpty()) return new Artifact(new Pose(Double.NaN, Double.NaN), 0);
            return sortBySize().artifacts.get(0);
        }

        /** Returns a copy of the current artifact list. */
        public List<Artifact> toList() {
            return new ArrayList<>(artifacts);
        }
        public int count()      { return artifacts.size(); }
        public boolean isArtifactDetected() { return !artifacts.isEmpty(); }

        // ─── Geometry helpers ────────────────────────────────────────────────────

        private List<Artifact> estimateVelocities(List<Artifact> freshArtifacts, double dt) {
            if (dt <= 0 || previousArtifacts.isEmpty()) return freshArtifacts;

            List<Mat.Tuple3<Double>> candidates = new ArrayList<>();
            for (int f = 0; f < freshArtifacts.size(); f++) {
                for (int p = 0; p < previousArtifacts.size(); p++) {
                    Pose predictedPrevPose = Artifact.predictPose(previousArtifacts.get(p), dt);
                    double dist = freshArtifacts.get(f).getPose().distanceFrom(predictedPrevPose);
                    if (dist <= MAX_ARTIFACT_MATCH_DISTANCE) {
                        candidates.add(new Mat.Tuple3<>((double) f, (double) p, dist));
                    }
                }
            }

            List<Artifact> trackedArtifacts = new ArrayList<>(candidates.size());
            candidates.sort(Comparator.comparingDouble(Mat.Tuple3::get_2));
            boolean[] freshTaken = new boolean[freshArtifacts.size()];
            boolean[] prevTaken = new boolean[previousArtifacts.size()];

            for (Mat.Tuple3<Double> c : candidates) {
                int f = c.get_0().intValue();
                int p = c.get_1().intValue();
                if (freshTaken[f] || prevTaken[p]) continue;
                freshTaken[f] = true;
                prevTaken[p] = true;

                Artifact fresh = freshArtifacts.get(f);
                Artifact prev = previousArtifacts.get(p);

                double deltaXP = fresh.getPose().getX() - prev.getPose().getX();
                double deltaYP = fresh.getPose().getY() - prev.getPose().getY();

                double rawVx = (deltaXP) / dt;
                double rawVy = (deltaYP) / dt;

                double vx, vy;
                double filteredXP, filteredYP;

                if (Math.hypot(rawVx, rawVy) < VELOCITY_NOISE_FLOOR) {
                    vx = 0;
                    vy = 0;
                    filteredXP = prev.getDeltaX();
                    filteredYP = prev.getDeltaY();
                } else {
                    filteredXP = Kinematics.lowPassFilter(deltaXP, prev.getDeltaX(), VELOCITY_LOWPASS_ALPHA);
                    filteredYP = Kinematics.lowPassFilter(deltaYP, prev.getDeltaY(), VELOCITY_LOWPASS_ALPHA);
                    vx = filteredXP / dt;
                    vy = filteredYP / dt;
                }

                Artifact tracked = new Artifact(fresh.getPose(), fresh.getSize())
                        .withVelocity(vx, vy)
                        .withPositionDelta(filteredXP, filteredYP);

                trackedArtifacts.add(tracked);
            }

            return trackedArtifacts;
        }

        private double getDistance(double ty) {
            double angleGoalRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE + ty);
            return (ARTIFACT_HEIGHT_FROM_FLOOR - LENS_HEIGHT_INCHES) / Math.tan(angleGoalRad);
        }

        private Pose getRelativePose(double distance, double tx) {
            double theta = Math.toRadians(tx);

            double deltaX = distance * Math.cos(theta) + X_OFFSET_INCHES;
            double deltaY = -distance * Math.sin(theta) + Y_OFFSET_INCHES;

            return new Pose(deltaX, deltaY);
        }

        private Pose getAbsolutePosition(Pose relativePose) {
            double theta = poseTracker.getPose().getHeading();
            double x = relativePose.getX() * Math.cos(theta) - relativePose.getY() * Math.sin(theta);
            double y = relativePose.getX() * Math.sin(theta) + relativePose.getY() * Math.cos(theta);


            return new Pose(
                    poseTracker.getPose().getX() + x,
                    poseTracker.getPose().getY() + y,
                    0);
        }

        private Pose getAbsolutePosition(double tx, double ty) {
            double distance = getDistance(ty);
            return getAbsolutePosition(getRelativePose(distance, tx));
        }


        // ─── Deprecated ──────────────────────────────────────────────────────────

        /** @deprecated Neural net pipeline no longer in use. */
        @Deprecated
        public List<Artifact> getArtifactsNeuralNet() {
            if (llResult == null || !llResult.isValid()) return new ArrayList<>();

            List<Artifact> artifactList = new ArrayList<>();
            var llArtifactList = llResult.getFiducialResults();

            for (var fiducialResult : llArtifactList) {
                double distance = getDistance(fiducialResult.getTargetYDegrees());
                Pose relativePose = getRelativePose(distance, fiducialResult.getTargetXDegrees());

                double theta = poseTracker.getPose().getHeading();
                double x = relativePose.getX() * Math.cos(theta) - relativePose.getY() * Math.sin(theta);
                double y = relativePose.getX() * Math.sin(theta) - relativePose.getY() * Math.cos(theta);

                artifactList.add(new Artifact(
                        new Pose(poseTracker.getPose().getX() + x, poseTracker.getPose().getY() + y),
                        fiducialResult.getFamily()
                ));
            }
            return artifactList;
        }
    }
}