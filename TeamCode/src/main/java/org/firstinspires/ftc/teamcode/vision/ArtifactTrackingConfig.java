package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArtifactTrackingConfig {
    public static int PIPELINE_INDEX = 1;

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double LIMELIGHT_MOUNT_ANGLE = -10.0;

    // distance from the center of the Limelight lens to the floor
    public static double LENS_HEIGHT_INCHES = 7.677;

    // distance from the target to the floor
    public static double ARTIFACT_HEIGHT_FROM_FLOOR = 0.0;

    // x and y offset from limelight UNGABUNGA FIX LATER
    public static double Y_OFFSET = 0.0;
    public static double X_OFFSET = -7.4;



}
