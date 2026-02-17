package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/*
 *
 *   === LLPYTHON LIST INDEX ===
 *   [0]=tx, [1]=ty, [2]=ta, [3]=valid, [4]=ColorID (1=P, 2=G)
 *
 */

@Config
public class LimelightConfig {
    public static String LIMELIGHT_NAME = "limelight";
    public static int PIPELINE_INDEX = 1;
    public static double LIMELIGHT_MOUNT_ANGLE = -10.0;

    // distance from the center of the Limelight lens to the floor
    public static double LENS_HEIGHT_INCHES = 9.25; //8.75
    public static double ARTIFACT_HEIGHT_FROM_FLOOR = 0;

    // x and y offset to 0;0 of the robot
    public static double Y_OFFSET_INCHES = 0; //-7.48
    public static double X_OFFSET_INCHES = 13; //-7.67

}
