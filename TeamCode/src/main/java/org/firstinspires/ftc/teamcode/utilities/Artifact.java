package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.enums.ArtifactColor;
import org.opencv.core.Mat;

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

    private Mat.Tuple2<Double> positionDelta = new Mat.Tuple2<>(0.0, 0.0);

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

    public Artifact withVelocity(double velX, double velY) {
        this.velocityX = velX;
        this.velocityY = velY;
        this.velocity = Math.hypot(velX, velY);
        return this;
    }

    public Artifact withPositionDelta(double x, double y) {
        this.positionDelta = new Mat.Tuple2<>(x, y);
        return this;
    }

    public void setPositionDelta(double x, double y) {
        positionDelta = new Mat.Tuple2<>(x, y);
    }

    public double getDeltaX() {
        return positionDelta.get_0();
    }

    public double getDeltaY() {
        return positionDelta.get_1();
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

        return new Pose(predictedX, predictedY, artifact.getPose().getHeading());
    }
}
