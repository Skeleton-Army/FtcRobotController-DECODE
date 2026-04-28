package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * KalmanPinpointLocalizer (MegaTag2 Edition)
 * * Fuses Pinpoint Odometry with Limelight MegaTag2 Vision.
 * * LOGIC:
 * 1. PREDICTION: Tracks robot motion using Pinpoint (X, Y, Heading).
 * 2. SYNC: Sends Pinpoint Heading to Limelight to enable MegaTag2.
 * 3. CORRECTION: Fuses Vision X/Y with Odometry X/Y.
 * (Heading is NOT corrected by vision, as Pinpoint is more reliable).
 */
public class KalmanPinpointLocalizer extends PinpointLocalizer {

    // --- TUNING VALUES ---
    public static double ODOMETRY_SIGMA_X = 0.02; // Drift per inch
    public static double ODOMETRY_SIGMA_Y = 0.02;
    public static double ODOMETRY_SIGMA_H = 0.005;

    public static double STATIC_SIGMA_X = 0.005; // Drift per sqrt(second)
    public static double STATIC_SIGMA_Y = 0.005;
    public static double STATIC_SIGMA_H = 0.001;

    public static double VISION_VARIANCE_X = 2.0; // Vision Uncertainty (Inches)
    public static double VISION_VARIANCE_Y = 2.0;

    // --- STATE VARIABLES ---
    private double px = 0.001;
    private double py = 0.001;

    // We maintain 'ph' (heading uncertainty) only for the prediction step
    // effectively, but we won't use it for vision correction.
    private double ph = 0.001;

    private Pose fusedPose;
    private Pose lastRawPose; // Tracks raw Pinpoint output for delta calculation

    private NanoTimer timer;
    private double lastTime;

    private final Limelight3A limelight;

    /**
     * Constructor
     */
    public KalmanPinpointLocalizer(HardwareMap map, Limelight3A limelight, PinpointConstants constants, Pose setStartPose) {
        super(map, constants, setStartPose);
        this.limelight = limelight;

        // Start Limelight polling
        this.limelight.pipelineSwitch(0);
        this.limelight.start();

        // Initialize Poses
        this.fusedPose = setStartPose;

        // Initialize Baseline for Deltas
        super.update();
        this.lastRawPose = super.getPose();

        timer = new NanoTimer();
        lastTime = timer.getElapsedTimeSeconds();
    }

    /**
     * MAIN LOOP
     */
    @Override
    public void update() {
        // --- 1. PREDICTION STEP (Odometry) ---

        // Get raw data from Pinpoint
        super.update();
        Pose currentRawPose = super.getPose();

        // Calculate Delta (Movement)
        double deltaX = currentRawPose.getX() - lastRawPose.getX();
        double deltaY = currentRawPose.getY() - lastRawPose.getY();
        double deltaH = MathFunctions.getSmallestAngleDifference(currentRawPose.getHeading(), lastRawPose.getHeading());

        // Update Fused Pose (Prediction)
        double predX = fusedPose.getX() + deltaX;
        double predY = fusedPose.getY() + deltaY;
        double predH = MathFunctions.normalizeAngle(fusedPose.getHeading() + deltaH);

        // Update Uncertainty (P = P + Q)
        double distanceTraveled = Math.hypot(deltaX, deltaY);
        double currentTime = timer.getElapsedTimeSeconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // Kinematic + Static Variance
        double q_kinematic_x = Math.pow(distanceTraveled * ODOMETRY_SIGMA_X, 2);
        double q_kinematic_y = Math.pow(distanceTraveled * ODOMETRY_SIGMA_Y, 2);
        double q_static_x = Math.pow(STATIC_SIGMA_X, 2) * deltaTime;
        double q_static_y = Math.pow(STATIC_SIGMA_Y, 2) * deltaTime;

        px += q_kinematic_x + q_static_x;
        py += q_kinematic_y + q_static_y;

        // We still track Heading Uncertainty purely for debug or future use,
        // even though we don't correct it with vision.
        ph += Math.pow(Math.abs(deltaH) * ODOMETRY_SIGMA_H, 2) + (Math.pow(STATIC_SIGMA_H, 2) * deltaTime);

        // --- 2. MEGATAG2 SYNC ---

        // MegaTag2 REQUIRES accurate robot heading to calculate X/Y.
        // We provide the Pinpoint's heading (converted to Degrees).
        double robotYawDegrees = Math.toDegrees(currentRawPose.getHeading());
        limelight.updateRobotOrientation(robotYawDegrees);

        // --- 3. CORRECTION STEP (Vision) ---

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            // USE MEGATAG2 RESULT
            Pose3D botPose3D = result.getBotpose_MT2();

            if (botPose3D != null) {
                // Convert Meters (Limelight) to Inches (Pedro)
                double visionX = botPose3D.getPosition().toUnit(DistanceUnit.INCH).x;
                double visionY = botPose3D.getPosition().toUnit(DistanceUnit.INCH).y;

                // 1. Calculate Kalman Gain (K) for X and Y only
                double kx = px / (px + Math.pow(VISION_VARIANCE_X, 2));
                double ky = py / (py + Math.pow(VISION_VARIANCE_Y, 2));

                // 2. Update State (x = x + K * (z - x))
                // We fuse Vision X/Y with our Predicted X/Y
                double correctedX = predX + kx * (visionX - predX);
                double correctedY = predY + ky * (visionY - predY);

                // 3. Update Covariance (P = (1 - K) * P)
                px = (1.0 - kx) * px;
                py = (1.0 - ky) * py;

                // 4. Update Fused Pose
                // Note: We use 'predH' (Odometry Heading) directly. No vision fusion for heading.
                fusedPose = new Pose(correctedX, correctedY, predH);
            } else {
                // No valid MT2 data, keep prediction
                fusedPose = new Pose(predX, predY, predH);
            }
        } else {
            // No tag seen, keep prediction
            fusedPose = new Pose(predX, predY, predH);
        }

        // Update trackers
        lastRawPose = currentRawPose;
    }

    /**
     * Returns the Kalman-Filtered Pose (Best Estimate)
     */
    @Override
    public Pose getPose() {
        return fusedPose;
    }

    /**
     * Returns the Current Velocity (Pass-through to Pinpoint)
     */
    @Override
    public Pose getVelocity() {
        return super.getVelocity();
    }

    /**
     * Handles manual position resets
     */
    @Override
    public void setPose(Pose setPose) {
        super.setPose(setPose);      // Reset hardware
        this.fusedPose = setPose;    // Reset software
        this.lastRawPose = setPose;  // Sync trackers

        // Reset uncertainty
        px = 0.001; py = 0.001; ph = 0.001;
    }

    @Override
    public void setStartPose(Pose setStart) {
        super.setStartPose(setStart);
        this.fusedPose = setStart;
        this.lastRawPose = super.getPose();
    }

    // Debug Getters
    public double getUncertaintyX() { return px; }
    public double getUncertaintyY() { return py; }
}