package org.firstinspires.ftc.teamcode.utilities;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.enums.ArtifactColor;

import java.util.Locale;

public class Artifact {
    private final Pose artifactPose;
    private final ArtifactColor artifactColor;

    public Artifact(double x, double y, ArtifactColor color) {
        artifactPose = new Pose(x, y);
        artifactColor = color;
    }

    public Artifact(double x, double y, String color) {
        artifactPose = new Pose(x, y);
        artifactColor = parseColor(color);
    }

    public static ArtifactColor parseColor(String color) {
        color = color.toLowerCase(Locale.ROOT);
        switch (color) {
            case "green":
                return ArtifactColor.GREEN;
            case "purple":
                return ArtifactColor.PURPLE;
            default:
                return ArtifactColor.UNKNOWN;
        }
    }

    public ArtifactColor getArtifactColor() {
        return artifactColor;
    }

    public Pose getArtifactPose() {
        return artifactPose;
    }
}
