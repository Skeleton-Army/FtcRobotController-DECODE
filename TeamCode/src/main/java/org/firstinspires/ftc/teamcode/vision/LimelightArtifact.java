package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.config.LimelightConfig.*;


public class LimelightArtifact {
    private final Limelight3A limelight;
    private final Follower follower;

    public LimelightArtifact(final HardwareMap hardwareMap, final Follower follower) {
        this.follower = follower;
        this.limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        this.limelight.pipelineSwitch(PIPELINE_INDEX);
        this.limelight.start();
    }

    /*
     * This method returns the absolute position of the closest artifact
     */
    public Pose getClosestArtifact() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null) {
            return null;
        }

        double[] pyOut = llResult.getPythonOutput();

        Pose relativePose = getRelativePose(getDistance(pyOut[1]), pyOut[0]);
        double theta = follower.getHeading();

        double y = relativePose.getX() * Math.sin(theta) - relativePose.getY() * Math.sin(theta);
        double x = relativePose.getX() * Math.cos(theta) - relativePose.getY() * Math.cos(theta);

        return new Pose(follower.getPose().getX() + x, follower.getPose().getY() + y);
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

    public Limelight3A getLimelight() {
        return limelight;
    }
}
