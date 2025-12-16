package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArtifactTrackingConfig {
    public static int PIPELINE_INDEX = 1;

    public static double CAMERA_HEIGHT_M = 0.35;

    // alpha: Camera mounting pitch angle relative to the floor (degrees)
    // Positive means tilted up, negative means tilted down
    public static double CAMERA_PITCH_DEG = 10.0;

    // Ht: Known height of the artifact's center from the floor (e.g., 2.0m)
    public static double TARGET_HEIGHT_M = 2.0;

    // Camera Mounting Offset (Relative to Robot Center in Meters)
    // FTC uses X=side-to-side (Right is positive), Y=forward/backward (Forward is positive)
    public static double CAM_OFFSET_X_M = 0.0;  // Center-mounted (X)
    public static double CAM_OFFSET_Y_M = 0.25; // 25 cm (0.25m) forward (Y)
    public static double CAM_YAW_DEG = -5.0;    // 5 degrees to the right (negative yaw)

    // Pre-calculate fixed angle constants
    public static double CAM_PITCH_RAD = toRadians(CAMERA_PITCH_DEG);
    public static double CAM_YAW_RAD = toRadians(CAM_YAW_DEG);

}
