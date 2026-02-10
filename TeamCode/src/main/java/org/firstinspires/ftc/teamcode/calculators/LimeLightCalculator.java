/*
package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.ARTIFACT_HEIGHT_FROM_FLOOR;
import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.LENS_HEIGHT_INCHES;
import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.LIMELIGHT_MOUNT_ANGLE;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig;

public class LimeLightCalculator {

    public static double getDistance(double ty) {
        double angleToGoalRadians = Math.toRadians(LIMELIGHT_MOUNT_ANGLE + ty);
        return (ARTIFACT_HEIGHT_FROM_FLOOR - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
    }

    public static Pose getRelativePos(double artifactDistance, double tx) {
        double thetaRad = Math.toRadians(tx);

        // Distance is forward (X), tx offset is sideways (Y)
        // Adjust offsets based on where your LL is mounted relative to center
        double localX = (artifactDistance + ArtifactTrackingConfig.X_OFFSET_INCHES);
        double localY = (localX * Math.tan(thetaRad)) + ArtifactTrackingConfig.Y_OFFSET_INCHES;

        return new Pose(localX, localY, 0);
    }

    public static Pose getRelativePos2(double artifactDistance, double tx) {
        double thetaRad = Math.toRadians(tx);

        // Distance is forward (X), tx offset is sideways (Y)
        // Adjust offsets based on where your LL is mounted relative to center
        double localX = (artifactDistance + ArtifactTrackingConfig.X_OFFSET_INCHES);
        double localY = (localX * Math.tan(thetaRad)) + ArtifactTrackingConfig.Y_OFFSET_INCHES;

        return new Pose(localX, localY, 0);
    }

    public static Pose calculateField(Pose robotPose, Pose artifactRelativePose) {
        double heading = robotPose.getHeading();

        // Standard 2D Rotation Matrix:
        // x_global = x_robot + (x_local * cos(θ) - y_local * sin(θ))
        // y_global = y_robot + (x_local * sin(θ) + y_local * cos(θ))
        double x = robotPose.getX() + (artifactRelativePose.getX() * Math.cos(heading) - artifactRelativePose.getY() * Math.sin(heading));
        double y = robotPose.getY() + (artifactRelativePose.getX() * Math.sin(heading) + artifactRelativePose.getY() * Math.cos(heading));

        return new Pose(x, y, 0);
    }

    public static Pose getArtifactAbsolutePos(Pose robotPose, double tx, double ty) {
        Pose artifactRelative = getRelativePos(getDistance(ty), tx);
        return calculateField(robotPose, artifactRelative);
    }

    public static Pose getArtifactAbsolutePos(Pose robotPose, double[] llPython) {
        return getArtifactAbsolutePos(robotPose, llPython[0], llPython[1]);
    }
}
*/


package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.ARTIFACT_HEIGHT_FROM_FLOOR;
import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.LENS_HEIGHT_INCHES;
import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.LIMELIGHT_MOUNT_ANGLE;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig;

public class LimeLightCalculator {
    public static double getDistance(double ty){

        double angleToGoalRadians = Math.toRadians(LIMELIGHT_MOUNT_ANGLE + ty);
        //calculate distance
        return (ARTIFACT_HEIGHT_FROM_FLOOR - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
    }

    // Relative to Sholef 2
    public static Pose getRelativePos( double artifactDistance, double tx) {
        double thetaRad = Math.toRadians(tx);

        // Calculate offsets
        double deltaX = (artifactDistance + ArtifactTrackingConfig.X_OFFSET_INCHES) * Math.cos(thetaRad);
        double deltaY = (artifactDistance + ArtifactTrackingConfig.Y_OFFSET_INCHES) * Math.sin(thetaRad);

        return new Pose(deltaX, deltaY);
    }

    public static Pose calculateField (Pose robotPose, Pose artifactRelative){
        double theta = robotPose.getHeading();
        double yOffset = artifactRelative.getX() * Math.sin(theta) - artifactRelative.getY() * Math.sin(theta);
        double xOffset = artifactRelative.getX() * Math.cos(theta) - artifactRelative.getY() * Math.cos(theta);
        return new Pose(robotPose.getX() + xOffset, robotPose.getY() + yOffset);
    }


    /*
    public static Pose calculateField(Pose robotPose, Pose artifactPose) {
        double x = robotPose.getX() + artifactPose.getY() * Math.cos(robotPose.getHeading()) - artifactPose.getX() * Math.sin(robotPose.getHeading());
        double y = robotPose.getY() + artifactPose.getY() * Math.sin(robotPose.getHeading()) + artifactPose.getX() * Math.cos(robotPose.getHeading());
        return new Pose(x, y);
    }
    */
    public static Pose getArtifactAbsolutePos(Pose robotPose, double tx, double ty) {
        Pose artifactPosition = getRelativePos(getDistance(ty),tx); //tx
        return calculateField(robotPose, artifactPosition);
    }

    public static Pose getArtifactAbsolutePos(Pose robotPose, double[] llPython) {
        double tx = llPython[0], ty = llPython[1];

        return getArtifactAbsolutePos(robotPose, tx, ty);
    }
}