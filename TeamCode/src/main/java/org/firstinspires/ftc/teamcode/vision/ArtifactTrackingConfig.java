package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArtifactTrackingConfig {
    public static int PIPELINE_INDEX = 1;

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    public static double goalHeightInches = 60.0;

}
