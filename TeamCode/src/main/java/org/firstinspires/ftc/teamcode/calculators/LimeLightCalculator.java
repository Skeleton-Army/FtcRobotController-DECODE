package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.ARTIFACT_HEIGHT_FROM_FLOOR;
import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.LENS_HEIGHT_INCHES;
import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.LIMELIGHT_MOUNT_ANGLE;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

public class LimeLightCalculator {
    public static double getDistance(double ty){

        double angleToGoalRadians = Math.toRadians(LIMELIGHT_MOUNT_ANGLE + ty);
        //calculate distance
        return (ARTIFACT_HEIGHT_FROM_FLOOR - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
    }

    // Relative to Sholef 2
    public static Pose2d getRelativePos( double artifactDistance, double tx) {
        double thetaRad = Math.toRadians(tx);

        // Calculate offsets
        double deltaX = (artifactDistance + ArtifactTrackingConfig.X_OFFSET_INCHES) * Math.cos(thetaRad);
        double deltaY = (artifactDistance + ArtifactTrackingConfig.Y_OFFSET_INCHES)* Math.sin(thetaRad);

        return new Pose2d(deltaX, deltaY, new Rotation2d(0));
    }

    public static Pose2d calculateField(Pose robotPose, Pose2d artifactPose) {
        double x = robotPose.getX() + artifactPose.getY() * Math.cos(robotPose.getHeading()) - artifactPose.getX() * Math.sin(robotPose.getHeading());
        double y = robotPose.getY() + artifactPose.getY() * Math.sin(robotPose.getHeading()) + artifactPose.getX() * Math.cos(robotPose.getHeading());
        return new Pose2d(x, y, new Rotation2d(0));
    }

    public static Pose2d getArtifactAbsolutePos(Pose robotPose, double tx, double ty) {
        Pose2d artifactPosition = getRelativePos(getDistance(ty),tx); //tx
        return calculateField(robotPose, artifactPosition);
    }
}
