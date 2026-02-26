package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.LimelightConfig;

import java.util.Objects;

public class KalmanPinpointLocalizer implements Localizer {

    // --- Values from your 500-sample test ---
    public static double STATIC_SIGMA_X = 0.00846;
    public static double STATIC_SIGMA_Y = 0.00009;

    // --- Adaptive Vision Coefficients (Used for sigma_limelight^2) ---
    public static double VISION_BASE_VARIANCE = 0;
    public static double VISION_DISTANCE_COEFF_X = 0.002997;
    public static double VISION_DISTANCE_COEFF_Y = 0.002360;
    public static double VISION_VELOCITY_COEFF_X = 0.2;
    public static double VISION_VELOCITY_COEFF_Y = 0.2;


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
    private double totalHeading;
    private double previousHeading;

    private Pose startPose;
    private Pose currentVelocity;
    private Pose pinpointPose;
    private final PinpointConstants constants;
    private GoBildaPinpointDriver odo;

    public KalmanPinpointLocalizer(HardwareMap map, PinpointConstants constants, Pose setStartPose) {

        odo = map.get(GoBildaPinpointDriver.class,constants.hardwareMapName);
        odo.setOffsets(constants.forwardPodY, constants.strafePodX, constants.distanceUnit);

        if(constants.yawScalar.isPresent()) {
            odo.setYawScalar(constants.yawScalar.getAsDouble());
        }

        if(constants.customEncoderResolution.isPresent()) {
            odo.setEncoderResolution(constants.customEncoderResolution.getAsDouble(), constants.distanceUnit);
        } else {
            odo.setEncoderResolution(constants.encoderResolution);
        }

        odo.setEncoderDirections(constants.forwardEncoderDirection, constants.strafeEncoderDirection);

        setStartPose(setStartPose);
        totalHeading = 0;
        pinpointPose = startPose;
        currentVelocity = new Pose();
        previousHeading = setStartPose.getHeading();
        this.constants = constants;

        this.limelight = map.get(Limelight3A.class, LimelightConfig.LIMELIGHT_NAME);
        this.limelight.pipelineSwitch(LimelightConfig.LIMELIGHT_INDEX);
        this.limelight.start();

        this.fusedPose = setStartPose;
        timer = new NanoTimer();
        lastTime = timer.getElapsedTimeSeconds();
    }

    @Override
    public void update() {
        // --- 1. PREDICTION STEP ---
        // xd = xd (drift remains same)
        odo.update();
        Pose rawPinpoint = PoseConverter.pose2DToPose(odo.getPosition(), PedroCoordinates.INSTANCE);
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized()));
        totalHeading += MathFunctions.getSmallestAngleDifference(rawPinpoint.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, rawPinpoint.getHeading());
        previousHeading = rawPinpoint.getHeading();
        pinpointPose = rawPinpoint;

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
                //double speed = Math.hypot(velocity.getX(), velocity.getY());
                double[] sigmaLimelightSq = calculateAdaptiveVariance(result, 0);

                // K = P / (P + sigma_limelight^2)
                double kx = px / (px + Math.pow(sigmaLimelightSq[0], 2));
                double ky = py / (py + Math.pow(sigmaLimelightSq[1], 2));

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

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return odo.getEncoderY();
    }

    @Override
    public double getLateralMultiplier() {
        return odo.getEncoderX();
    }

    @Override
    public double getTurningMultiplier() {
        return odo.getYawScalar();
    }

    @Override
    public void resetIMU() {
        resetPinpoint();
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    private double[] calculateAdaptiveVariance(LLResult result, double speed) {
        double varX = VISION_BASE_VARIANCE;
        double varY = VISION_BASE_VARIANCE;
        if (!result.getFiducialResults().isEmpty()) {
            double distInchesX = result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().x * 39.3701; // get the x and y separately for distance
            double distInchesY = result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().y * 39.3701; // get the x and y separately for distance
            varX += (distInchesX * VISION_DISTANCE_COEFF_X);
            varY += (distInchesY * VISION_DISTANCE_COEFF_Y);
        }
        varX += (speed * VISION_VELOCITY_COEFF_X);
        varY += (speed * VISION_VELOCITY_COEFF_Y);
        return new double[]{varX, varY};

    }

    @Override
    public Pose getPose() { return fusedPose; }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = pinpointPose.rotate(-startPose.getHeading(), false).minus(startPose);
            setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }

        this.startPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        odo.setPosition(PoseConverter.poseToPose2D(setPose, PedroCoordinates.INSTANCE));
        pinpointPose = setPose;
        previousHeading = setPose.getHeading();
        this.fusedPose = setPose;
        this.driftX = 0;
        this.driftY = 0;
        this.px = 0.001;
        this.py = 0.001;
    }

    @Override
    public void setX(double x) {
        odo.setPosX(x, constants.distanceUnit);
    }

    @Override
    public void setY(double y) {
        odo.setPosY(y, constants.distanceUnit);
    }

    @Override
    public void setHeading(double heading) {
        odo.setHeading(heading, AngleUnit.RADIANS);
    }
    public void recalibrate() {
        odo.recalibrateIMU();
    }
    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}