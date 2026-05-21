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

import java.util.ArrayList;
import java.util.List;
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

   private LLResult llResult;

    public Vision(HardwareMap hardwareMap, PoseTracker poseTracker, int pipelineIndex) {
        this.poseTracker = poseTracker;
        this.pipeline = pipelineIndex;

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(this.pipeline);
        limelight.start();
        llResult = null;

        relocalizeTimer.start();
    }

    @Override
    public void periodic() {
        double orientationDeg = Math.toDegrees(poseTracker.getPose().getHeading()) + 90;
        limelight.updateRobotOrientation(orientationDeg);
        llResult = limelight.getLatestResult();
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
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

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

        if (llResult == null || !llResult.isValid()) return null;

        for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
            int id = fiducial.getFiducialId();
            if (id == GPP_TAG_ID) return ArtifactPattern.GPP;
            if (id == PGP_TAG_ID) return ArtifactPattern.PGP;
            if (id == PPG_TAG_ID) return ArtifactPattern.PPG;
        }

        return null;
    }
}
