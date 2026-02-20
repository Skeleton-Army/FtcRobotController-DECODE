package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class KalmanPinpointLocalizer extends PinpointLocalizer {

    // --- Values from your 500-sample test ---
    public static double STATIC_SIGMA_X = 0.00846;
    public static double STATIC_SIGMA_Y = 0.00009;

    // --- Adaptive Vision Coefficients (Used for sigma_limelight^2) ---
    public static double VISION_BASE_VARIANCE = 0.5;
    public static double VISION_DISTANCE_COEFF = 0.05;
    public static double VISION_VELOCITY_COEFF = 0.2;

    // State Variables (xd in your images)
    private double driftX = 0;
    private double driftY = 0;

    // Uncertainty (P in your images)
    private double px = 0.001;
    private double py = 0.001;

    private NanoTimer timer;
    private double lastTime;
    private final Limelight3A limelight;
    private Pose fusedPose;

    public KalmanPinpointLocalizer(HardwareMap map, Limelight3A limelight, PinpointConstants constants, Pose setStartPose) {
        super(map, constants, setStartPose);
        this.limelight = limelight;
        this.limelight.pipelineSwitch(0);
        this.limelight.start();

        this.fusedPose = setStartPose;
        timer = new NanoTimer();
        lastTime = timer.getElapsedTimeSeconds();
    }

    @Override
    public void update() {
        // --- 1. PREDICTION STEP ---
        // xd = xd (drift remains same)

        super.update();
        Pose rawPinpoint = super.getPose();
        Pose velocity = super.getVelocity();

        double currentTime = timer.getElapsedTimeSeconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // P = P + sigma_d^2 * dt
        px += Math.pow(STATIC_SIGMA_X, 2) * dt;
        py += Math.pow(STATIC_SIGMA_Y, 2) * dt;

        // --- 2. PREPARE MEASUREMENT ---
        double robotYawDegrees = Math.toDegrees(rawPinpoint.getHeading());
        limelight.updateRobotOrientation(robotYawDegrees);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose3D = result.getBotpose_MT2();

            if (botPose3D != null) {
                // Convert Vision Meters to Inches
                double visionX = botPose3D.getPosition().x * 39.3701;
                double visionY = botPose3D.getPosition().y * 39.3701;

                // z = x_pinpoint - x_limelight
                double zx = rawPinpoint.getX() - visionX;
                double zy = rawPinpoint.getY() - visionY;

                // --- 3. CORRECTION STEP ---
                // Calculate adaptive sigma_limelight^2
                double speed = Math.hypot(velocity.getX(), velocity.getY());
                double sigmaLimelightSq = Math.pow(calculateAdaptiveVariance(result, speed), 2);

                // K = P / (P + sigma_limelight^2)
                double kx = px / (px + sigmaLimelightSq);
                double ky = py / (py + sigmaLimelightSq);

                // xd = xd + K * (z - xd)
                driftX = driftX + kx * (zx - driftX);
                driftY = driftY + ky * (zy - driftY);

                // P = (1 - K) * P
                px = (1.0 - kx) * px;
                py = (1.0 - ky) * py;
            }
        }

        // --- 4. OUTPUT ---
        // Final Pose = Pinpoint - Estimated Drift
        double correctedX = rawPinpoint.getX() - driftX;
        double correctedY = rawPinpoint.getY() - driftY;

        fusedPose = new Pose(correctedX, correctedY, rawPinpoint.getHeading());
    }

    private double calculateAdaptiveVariance(LLResult result, double speed) {
        double var = VISION_BASE_VARIANCE;
        if (!result.getFiducialResults().isEmpty()) {
            double distInches = result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().z * 39.3701;
            var += (distInches * VISION_DISTANCE_COEFF);
        }
        var += (speed * VISION_VELOCITY_COEFF);
        return var;
    }

    @Override
    public Pose getPose() { return fusedPose; }

    @Override
    public void setPose(Pose setPose) {
        super.setPose(setPose);
        this.fusedPose = setPose;
        this.driftX = 0;
        this.driftY = 0;
        this.px = 0.001;
        this.py = 0.001;
    }
}