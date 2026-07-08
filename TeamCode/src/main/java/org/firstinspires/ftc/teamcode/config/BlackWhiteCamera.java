package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class BlackWhiteCamera {

    public static double offsetX_turret = 0; // 174.5mm
    public static double offsetY_turret = 6.87007874 + ShooterConfig.TURRET_OFFSET_X;
    public static double offsetZ = 12.38189; //314.5 mm
    public static double pitchAngle = 15;
    public static Position relativePos = new Position(DistanceUnit.MM, 179.081, -149.126, 158.1635, 0);
    //public static Position relativePos = new Position(DistanceUnit.MM, -44.294 - 25, 222.429 - 50, 235.137, 0);

    // --- Per-Tag Localization Bias (robot-relative frame) ---
    // X: positive = shift correction to the robot's LEFT, negative = RIGHT
    // Y: positive = shift correction BACKWARD (front of robot), negative = FRONT
    public static double blueTagBiasX = -1.3; // tag id 20
    public static double blueTagBiasY = -0.2;
    public static double redTagBiasX = 1.3;  // tag id 24
    public static double redTagBiasY = 0.2;

    public static int WIDTH = 1280;
    public static int HEIGHT = 720;

    public static final double[] cameraMatrix = {
            705.3303, -0.5292, 601.1362, // fx, skew, cx
            0, 700.8256, 380.9664,       // 0, fy, cy
            0, 0, 1                      // 0, 0, 1
    };
//    public static final double[] cameraMatrix = {
//            705.3303, -0.5292, 660.1362, // fx, skew, cx
//            0, 700.8256, 400.9664,       // 0, fy, cy
//            0, 0, 1                      // 0, 0, 1
//    };

    public static final double[] distCoeffs = {
            -0.3492, 0.1757, -0.0005, 0.0010, -0.0552 // k1, k2, p1, p2, k3
    };

    // Locked (ROI) frames search a tiny crop, so we can afford fine (slow) decimation for accuracy.
    // Fallback (full-frame) frames must search a huge area, so decimation is bumped up to keep the
    // detector's pixel-sweep cost bounded and avoid the multi-hundred-ms / ~2s spikes on tag loss.
    public static float DECIMATION_LOCKED = 0f;
    public static float DECIMATION_FALLBACK = 5f;
    public static int MAX_LOST_FRAMES = 2;
}