package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.enums.ArtifactColor;

import java.util.Locale;

import lombok.Getter;

@Getter
public class Artifact {
    private final Pose pose;
    private final ArtifactColor color;
    private final double size;

    private double velocity;
    private double velocityX;
    private double velocityY;

    public static Pose predictedPose;
    public Artifact(Pose pose, String color) {
        this.pose = pose;
        this.color = parseColor(color);
        size = Double.NaN;
    }

    public Artifact(Pose pose, double ta) {
        this.pose = pose;
        color = ArtifactColor.UNKNOWN;
        size = ta;
    }

    /** Sets velocity as a vector (inches/sec in field X/Y). Magnitude is derived automatically. */
    public void setVelocity(double velocityX, double velocityY) {
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.velocity = Math.hypot(velocityX, velocityY);
    }

    public boolean isMoving(double threshold) {
        return velocity > threshold;
    }

    public static ArtifactColor parseColor(String color) {
        color = color.toLowerCase(Locale.ROOT);
        switch (color) {
            case "green":
                return ArtifactColor.GREEN;
            case "purple":
                return ArtifactColor.PURPLE;
            case "mixed":
                return ArtifactColor.MIXED;
            default:
                return ArtifactColor.UNKNOWN;
        }
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ROOT,
                "pos: [%f, %f] color: %s, vel: %f",
                pose.getX(), pose.getY(), color.name(), velocity);
    }

    public static Pose predictPose(Artifact artifact, double deltaTime) {
        double predictedX = artifact.getPose().getX() + artifact.getVelocityX() * deltaTime;
        double predictedY = artifact.getPose().getY() + artifact.getVelocityY() * deltaTime;
        predictedPose = new Pose(predictedX, predictedY, artifact.getPose().getHeading());
        return predictedPose;
    }
}
