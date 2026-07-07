package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConfig {
    public static String LIMELIGHT_NAME = "limelight";
    public static double VELOCITY_THRESHOLD = 0.2;
    public static double RELOCALIZE_COOLDOWN = 60;
    public static int APRILTAG_PIPELINE = 0;
    public static int OBELISK_PIPELINE = 1;
    public static int DETECTION_PIPELINE = 2;
    public static int NEURAL_DETECTION_PIPELINE = 3; //poopoo kaka
    public static double Y_CALIBRATION_OFFSET = 1;

    //Pose Detection
    public static double LIMELIGHT_MOUNT_ANGLE = -10.0;
    public static double LENS_HEIGHT_INCHES = 9.07; //8.75
    public static double ARTIFACT_HEIGHT_FROM_FLOOR = 2.5;
    public static double Y_OFFSET_INCHES = 1.85; //-7.48
    public static double X_OFFSET_INCHES = 8; //-7.67

    // Velocity Tracking
    public static final double MAX_ARTIFACT_MATCH_DISTANCE = 8.0;

}
