package org.firstinspires.ftc.teamcode.utilities;


import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * KalmanPinpointLocalizer
 * * An advanced localizer that fuses Pinpoint Odometry with Vision (AprilTags).
 * * LOGIC:
 * 1. Prediction (Odometry):
 * - Updates position based on Pinpoint encoders/IMU.
 * - Increases uncertainty (P) based on distance driven (Kinematic Drift).
 * - Increases uncertainty (P) based on time elapsed (Static Drift/Random Walk).
 * * 2. Correction (Vision):
 * - When a tag is seen, calculates Kalman Gain (K).
 * - Fuses the Vision Pose with the Odometry Pose.
 * - Resets the Pinpoint hardware to the new "True" position.
 * * TUNING:
 * - Run 'KalmanSigmaTuner' to find 'odometrySigma' (Kinematic).
 * - Run 'KalmanStaticTuner' to find 'staticSigma' (Static).
 */
public class KalmanPinpointLocalizer extends PinpointLocalizer {

    // --- TUNING VALUES (Get these from your Tuner OpModes) ---

    // 1. Kinematic Sigma: "Drift per Inch traveled"
    // Found using KalmanSigmaTuner (Driving around)
    // Default: 0.02 (2% error)
    public static double ODOMETRY_SIGMA_X = 0.02;
    public static double ODOMETRY_SIGMA_Y = 0.02;
    public static double ODOMETRY_SIGMA_H = 0.005; // Radians per radian turned

    // 2. Static Sigma: "Drift per sqrt(second) stationary"
    // Found using KalmanStaticTuner (Stationary vibration test)
    // Default: 0.005
    public static double STATIC_SIGMA_X = 0.005;
    public static double STATIC_SIGMA_Y = 0.005;
    public static double STATIC_SIGMA_H = 0.001;

    // 3. Vision Confidence (R): Standard Deviation of your camera
    // Lower = Trust Camera more. Higher = Trust Odometry more.
    // Default: 1.0 inch
    public static double VISION_VARIANCE_X = 2.0;
    public static double VISION_VARIANCE_Y = 2.0;
    public static double VISION_VARIANCE_H = Math.toRadians(5); // 5 degrees

    // --- KALMAN STATE ---
    // P: Current Uncertainty (Covariance)
    private double px = 0.0;
    private double py = 0.0;
    private double ph = 0.0;

    // Internal tracking
    private Pose lastPinpointPose;
    private NanoTimer timer;
    private double lastTime;

    public Pose fusedPose;
    Limelight3A limelight;
    /**
     * Constructor
     */
    public KalmanPinpointLocalizer(HardwareMap map, Limelight3A limelight, PinpointConstants constants, Pose setStartPose) {
        super(map, constants, setStartPose);

        lastPinpointPose = setStartPose;

        // Initialize timer for Static Drift calculation
        timer = new NanoTimer();
        lastTime = timer.getElapsedTimeSeconds();

        // Initial uncertainty (we trust start pose decently well)
        px = 0.001;
        py = 0.001;
        ph = 0.001;
    }

    /**
     * PREDICTION STEP
     * - Updates robot position from Pinpoint.
     * - Adds uncertainty based on Motion (Kinematic) and Time (Static).
     */
    @Override
    public void update() {
        // 1. Get raw update from hardware (Pinpoint)
        super.update();
        Pose currentPinpointPose = super.getPose();

        // 2. Calculate Deltas (Motion)
        double deltaX = currentPinpointPose.getX() - lastPinpointPose.getX();
        double deltaY = currentPinpointPose.getY() - lastPinpointPose.getY();
        double deltaH = MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), lastPinpointPose.getHeading());
        double distanceTraveled = Math.hypot(deltaX, deltaY);

        // 3. Calculate Time Elapsed (for Static Drift)
        double currentTime = timer.getElapsedTimeSeconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // 4. PREDICT NEW UNCERTAINTY (P = P + Q)

        // A. Kinematic Component (Motion-based)
        // Variance += (Distance * Sigma)^2
        double kinematicVarX = Math.pow(distanceTraveled * ODOMETRY_SIGMA_X, 2);
        double kinematicVarY = Math.pow(distanceTraveled * ODOMETRY_SIGMA_Y, 2);
        double kinematicVarH = Math.pow(Math.abs(deltaH) * ODOMETRY_SIGMA_H, 2);

        // B. Static Component (Time-based / Random Walk)
        // Variance += Sigma^2 * Time_Step
        // (Because Sigma is "drift per root second", Sigma^2 is "variance per second")
        double staticVarX = Math.pow(STATIC_SIGMA_X, 2) * deltaTime;
        double staticVarY = Math.pow(STATIC_SIGMA_Y, 2) * deltaTime;
        double staticVarH = Math.pow(STATIC_SIGMA_H, 2) * deltaTime;

        // Apply total Process Noise
        px += kinematicVarX + staticVarX;
        py += kinematicVarY + staticVarY;
        ph += kinematicVarH + staticVarH;

        // Update tracking variables
        lastPinpointPose = currentPinpointPose;
    }

    /**
     * CORRECTION STEP (Vision Fusion)
     * Call this whenever your camera sees an AprilTag.
     * * @param visionPose The absolute field pose calculated by the camera.
     */
    public void updateVision(Pose visionPose) {
        // Current Estimate (from Odometry Prediction)
        Pose x_pred = getPose();

        // 1. Calculate Kalman Gain (K)
        // K = P / (P + R)
        // If P is high (we are lost), K is high (trust vision).
        // If R is high (camera is bad), K is low (trust odometry).
        double kx = px / (px + Math.pow(VISION_VARIANCE_X, 2));
        double ky = py / (py + Math.pow(VISION_VARIANCE_Y, 2));
        double kh = ph / (ph + Math.pow(VISION_VARIANCE_H, 2));

        // 2. Update State (x = x + K * (z - x))
        double newX = x_pred.getX() + kx * (visionPose.getX() - x_pred.getX());
        double newY = x_pred.getY() + ky * (visionPose.getY() - x_pred.getY());

        // Handle Heading Wrap for the difference
        double headingDiff = MathFunctions.getSmallestAngleDifference(visionPose.getHeading(), x_pred.getHeading());
        double newH = x_pred.getHeading() + kh * headingDiff;

        // 3. Update Covariance (P = (1 - K) * P)
        // Our uncertainty shrinks because we just got a confirmation from Vision.
        px = (1.0 - kx) * px;
        py = (1.0 - ky) * py;
        ph = (1.0 - kh) * ph;

        // 4. FEEDBACK TO HARDWARE
        // This is crucial. We must tell the Pinpoint it was wrong.
        // This resets the Pinpoint's internal integrators to the new fused position.
        fusedPose = new Pose(newX, newY, newH);
        //setPose(fusedPose); // -> this is setting the pinpoint pose as well - we need to return a new value

        // Sync our local tracker so the next update() doesn't double-count the jump
        lastPinpointPose = fusedPose;
    }

    /**
     * Debugging: Get current Uncertainty
     */
    public double getUncertaintyX() { return px; }
    public double getUncertaintyY() { return py; }
}
