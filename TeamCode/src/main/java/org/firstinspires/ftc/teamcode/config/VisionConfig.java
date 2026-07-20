package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConfig {
    public final static String LIMELIGHT_NAME = "limelight";
    public static double VELOCITY_THRESHOLD = 0.2;
    public static double RELOCALIZE_COOLDOWN = 60;
    public final static int APRILTAG_PIPELINE = 0;
    public final static int OBELISK_PIPELINE = 1;
    public final static int DETECTION_PIPELINE = 2;
    public static int NEURAL_DETECTION_PIPELINE = 3; //poopoo kaka
    public static double Y_CALIBRATION_OFFSET = 1;

    //Physical offsets
    public static double LIMELIGHT_MOUNT_ANGLE = -10.0;
    public static double LENS_HEIGHT_INCHES = 9.07; //8.75
    public static double ARTIFACT_HEIGHT_FROM_FLOOR = 2.5;
    public static double Y_OFFSET_INCHES = 0; //-7.48
    public static double X_OFFSET_INCHES = 8; //-7.67

    // Velocity Tracking
    public static double MAX_ARTIFACT_MATCH_DISTANCE = 3.0;
    public static double VELOCITY_NOISE_FLOOR = 0.8; // Below this speed is sensor noise
    public static double VELOCITY_LOWPASS_ALPHA = 0.6;

    //Pose prediction
    public static double AVERAGE_APPROACH_VELOCITY = 25.0; // inch/sec
    public static int PREDICTION_ITERATIONS = 10;
    public static int MIN_DETECTION_CYCLES = 15;
}
