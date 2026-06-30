package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.enums.ArtifactColor;

import java.util.Locale;

import lombok.Getter;

@Getter
public class Artifact {
    private final Pose artifactPose;
    private final ArtifactColor artifactColor;

    public Artifact(double x, double y, ArtifactColor color) {
        artifactPose = new Pose(x, y);
        artifactColor = color;
    }

    public Artifact(double x, double y, int color) {
        artifactPose = new Pose(x, y);
        artifactColor = parseColor(color);
    }

    public Artifact(Pose pose, String color) {
        artifactPose = pose;
        artifactColor = parseColor(color);
    }

    public Artifact(Pose pose) {
        artifactPose = pose;
        artifactColor = ArtifactColor.UNKNOWN;
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

    public static ArtifactColor parseColor(int color) {
        switch (color) {
            case 1:
                return ArtifactColor.PURPLE;
            case 2:
                return ArtifactColor.GREEN;
            case 3:
                return ArtifactColor.MIXED;
            default:
                return ArtifactColor.UNKNOWN;
        }
    }

    public static Pair<Double, Double> unpack(double packed) {
        double tx = packed / 1000.0;
        double ty = packed % 100;
        return new Pair<>(tx, ty);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ROOT,
                "pos: [%f, %f] color: %s",
                artifactPose.getX(), artifactPose.getY(), artifactColor.name());
    }

}
