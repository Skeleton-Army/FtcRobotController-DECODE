package org.firstinspires.ftc.teamcode.utilities;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Full Pedro Pathing FusionLocalizer implementation, based on WPILib.
 * Predicts pose with twist integration and corrects with delayed measurements.
 * Features Adaptive Kalman Covariance and Mahalanobis Distance outlier rejection.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics (Original)
 * @author Modified for Adaptive Vision Fusion
 */
@Config
public class NewKalmanLocolizer implements Localizer {
    private final Localizer deadReckoning;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Matrix P;      // Covariance
    private final Matrix Q; // Process noise
    private long lastUpdateTime = -1;

    private final NavigableMap<Long, Pose> poseHistory = new TreeMap<>();
    private final NavigableMap<Long, Pose> twistHistory = new TreeMap<>();
    private final int bufferSize;

    // Baseline Measurement Standard Deviations (when standing still next to a tag)
    private final double[] baseMeasurementStdDevs;

    // --- FTC Dashboard Tuning Constants ---
    // Tune these live in the dashboard!
    public static double k_v = 0.05;     // Penalty multiplier for linear speed
    public static double k_omega = 0.1;  // Penalty multiplier for rotational speed

    // Independent distance penalty multipliers
    public static double k_dx = 0.0001;  // Penalty multiplier for X distance squared
    public static double k_dy = 0.0001;  // Penalty multiplier for Y distance squared
    public static double k_dh = 0.0001;  // Penalty multiplier for Heading distance squared

    public static double RegressionSlopeX = -0.004991;
    public static double RegressionSlopeY =  -0.005617;
    public static double  RegressionFreeX =  0.5512;
    public static double  RegressionFreeY =  0.6071;
    public static double maxSpeedThreshold = 40.0; // Speed in inches/sec where we ignore vision completely

    // Chi-square threshold for 3 DOF at 99% confidence is 11.34.
    // If the Limelight pose jumps through a wall, this gate rejects it.
    public static double mahalanobisThreshold = 11.34;
    private Pose lastRawPose = null; // Tracks the raw Pinpoint pose from the previous loop

    // 0.01% error = 0.0001
    public static double PINPOINT_ERROR_RATIO = 0.0001; // 0.00094 0.002

    public NewKalmanLocolizer(
            Localizer deadReckoning,
            double[] processStdDevs,
            double[] baseMeasurementStdDevs,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.baseMeasurementStdDevs = baseMeasurementStdDevs;
        this.currentPosition = new Pose();
        this.P = getIdentity3x3();
        this.Q = getDiagonal3x3(
                processStdDevs[0] * processStdDevs[0],
                processStdDevs[1] * processStdDevs[1],
                processStdDevs[2] * processStdDevs[2]
        );
        this.bufferSize = bufferSize;
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        // 1. Get the CURRENT raw pose from the Pinpoint
        Pose currentRawPose = deadReckoning.getPose();

        // 2. Calculate distance traveled since the LAST loop
        double distanceTraveledStep = 0;
        if (lastRawPose != null) {
            double dx_raw = currentRawPose.getX() - lastRawPose.getX();
            double dy_raw = currentRawPose.getY() - lastRawPose.getY();
            distanceTraveledStep = Math.hypot(dx_raw, dy_raw);
        }

        // 3. Predict step via twist (standard Pedro Pathing logic)
        Pose twist = deadReckoning.getVelocity();
        // ... (existing integration code to update currentPosition) ...

        // 4. Covariance propagation using the 0.01% rule
        // We square the standard deviation (distance * 0.0001) to get the variance
        double distanceVarianceStep = Math.pow(distanceTraveledStep * PINPOINT_ERROR_RATIO, 2);

        Matrix distanceDriftMatrix = getDiagonal3x3(
                distanceVarianceStep, // X variance
                distanceVarianceStep, // Y variance
                0.0                   // Heading variance (handled by IMU in Q)
        );

        // Grow P matrix: Time-based drift + Distance-based drift
        P = P.plus(Q.multiply(dt)).plus(distanceDriftMatrix);

        // 5. Cleanup for next loop
        lastRawPose = currentRawPose.copy();
        poseHistory.put(now, currentPosition.copy());
        if (poseHistory.size() > bufferSize) poseHistory.pollFirstEntry();
        if (twistHistory.size() > bufferSize) twistHistory.pollFirstEntry();
    }

    /**
     * Adds a delayed vision measurement with ADAPTIVE covariance and OUTLIER rejection.
     * @param measuredPose The pose returned by the Limelight
     * @param timestamp When the picture was taken (System.nanoTime() - latency.
     *
     */
    public void addVisionMeasurement(Pose measuredPose, long timestamp, LLResult result) {
        if (!poseHistory.containsKey(timestamp) || !twistHistory.containsKey(timestamp)) return;

        Pose twistAtMeasurement = twistHistory.get(timestamp);
        double translationalSpeed = Math.hypot(twistAtMeasurement.getX(), twistAtMeasurement.getY());

        // 1. Trust Odometry entirely if moving too fast
        if (translationalSpeed > maxSpeedThreshold) return;

        double rotationalSpeed = Math.abs(twistAtMeasurement.getHeading());

        // 2. Compute Dynamic Covariance (R)

        Matrix dynamicR = dynamicStdevR(result.getFiducialResults(),translationalSpeed, rotationalSpeed);

        // 3. Compute innovation (y) - The difference between vision and our past history
        Pose pastPose = poseHistory.get(timestamp);
        double y0 = measuredPose.getX() - pastPose.getX();
        double y1 = measuredPose.getY() - pastPose.getY();
        double y2 = MathFunctions.normalizeAngle(measuredPose.getHeading() - pastPose.getHeading());

        Matrix y = new Matrix(new double[][]{{y0}, {y1}, {y2}});

        // 4. Compute Innovation Covariance (S) and its inverse
        Matrix S = P.plus(dynamicR);
        Matrix S_inv = invert3x3(S);


        // --- 5. THE INNOVATION GATE (MAHALANOBIS DISTANCE) ---
        // Manually calculating y^T * S_inv * y to avoid needing a transpose() method in Matrix.java
        double s00 = S_inv.get(0, 0), s01 = S_inv.get(0, 1), s02 = S_inv.get(0, 2);
        double s10 = S_inv.get(1, 0), s11 = S_inv.get(1, 1), s12 = S_inv.get(1, 2);
        double s20 = S_inv.get(2, 0), s21 = S_inv.get(2, 1), s22 = S_inv.get(2, 2);

        double mahalanobisSquared = y0 * (s00 * y0 + s01 * y1 + s02 * y2) +
                y1 * (s10 * y0 + s11 * y1 + s12 * y2) +
                y2 * (s20 * y0 + s21 * y1 + s22 * y2);

        // Reject the measurement if it's mathematically impossible
        if (mahalanobisSquared > mahalanobisThreshold) {
            return;
        }

        // 6. Compute Kalman gain
        Matrix K = P.multiply(S_inv);

        // 7. Update past pose
        Matrix K_y = K.multiply(y);
        Pose updatedPast = new Pose(
                pastPose.getX() + K_y.get(0, 0),
                pastPose.getY() + K_y.get(1, 0),
                MathFunctions.normalizeAngle(pastPose.getHeading() + K_y.get(2, 0))
        );
        poseHistory.put(timestamp, updatedPast);

        // 8. Propagate update forward using stored twists
        long previousTime = timestamp;
        Pose previousPose = updatedPast;
        for (NavigableMap.Entry<Long, Pose> entry : poseHistory.tailMap(timestamp, false).entrySet()) {
            long t = entry.getKey();
            Pose twist = twistHistory.get(previousTime);
            double dt = (t - previousTime) / 1e9;
            Pose nextPose = integrate(previousPose, twist, dt);
            poseHistory.put(t, nextPose);
            previousPose = nextPose;
            previousTime = t;
        }

        // 9. Update current state
        currentPosition = poseHistory.lastEntry().getValue();
    }

    private Pose integrate(Pose previousPose, Pose twist, double dt) {
        double cosH = Math.cos(previousPose.getHeading());
        double sinH = Math.sin(previousPose.getHeading());
        double dx = (twist.getX() * cosH - twist.getY() * sinH) * dt;
        double dy = (twist.getX() * sinH + twist.getY() * cosH) * dt;
        double dTheta = twist.getHeading() * dt;

        return new Pose(
                previousPose.getX() + dx,
                previousPose.getY() + dy,
                MathFunctions.normalizeAngle(previousPose.getHeading() + dTheta)
        );
    }

    public Matrix dynamicStdevR(List<FiducialResult> fiducialResults, double translationalSpeed, double rotationalSpeed) {
        //List<FiducialResult> fiducialResults = limelight3A.getLatestResult().getFiducialResults();
        // add null check
        Pose targetPose = new Pose(fiducialResults.get(0).getTargetXDegrees(), fiducialResults.get(0).getTargetYDegrees());
        // use target pose to calculate dis from bot
        // target pose is the degree x y of the apriltag
        Pose disVector = null; //To remove errors

        double distancePenaltyX = RegressionSlopeX * disVector.getX() + RegressionFreeX;
        double distancePenaltyY = RegressionSlopeY * disVector.getY() + RegressionFreeY;
        double distancePenaltyH = k_dh;

        double dynamicStdX = baseMeasurementStdDevs[0] + (k_v * translationalSpeed) + distancePenaltyX;
        double dynamicStdY = baseMeasurementStdDevs[1] + (k_v * translationalSpeed) + distancePenaltyY;
        double dynamicStdH = baseMeasurementStdDevs[2] + (k_omega * rotationalSpeed) + distancePenaltyH;

          return getDiagonal3x3(
                dynamicStdX * dynamicStdX,
                dynamicStdY * dynamicStdY,
                dynamicStdH * dynamicStdH
        );
    }
    @Override
    public Pose getPose() { return currentPosition; }

    @Override
    public Pose getVelocity() {
        return currentVelocity != null ? currentVelocity : deadReckoning.getVelocity();
    }

    @Override
    public Vector getVelocityVector() { return getVelocity().getAsVector(); }

    @Override
    public void setStartPose(Pose setStart) { deadReckoning.setStartPose(setStart); }

    @Override
    public void setPose(Pose setPose) {
        currentPosition = setPose.copy();
        deadReckoning.setPose(setPose);
        poseHistory.lastEntry().setValue(setPose.copy());
    }

    @Override
    public double getTotalHeading() { return currentPosition.getHeading(); }

    @Override
    public double getForwardMultiplier() { return deadReckoning.getForwardMultiplier(); }

    @Override
    public double getLateralMultiplier() { return deadReckoning.getLateralMultiplier(); }

    @Override
    public double getTurningMultiplier() { return deadReckoning.getTurningMultiplier(); }

    @Override
    public void resetIMU() throws InterruptedException { deadReckoning.resetIMU(); }

    @Override
    public double getIMUHeading() { return deadReckoning.getIMUHeading(); }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPosition.getX()) || Double.isNaN(currentPosition.getY()) || Double.isNaN(currentPosition.getHeading());
    }

    // --- CUSTOM 3x3 MATRIX HELPERS ---

    private Matrix getIdentity3x3() {
        return new Matrix(new double[][]{
                {1.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0}
        });
    }

    private Matrix getDiagonal3x3(double a, double b, double c) {
        return new Matrix(new double[][]{
                {a, 0.0, 0.0},
                {0.0, b, 0.0},
                {0.0, 0.0, c}
        });
    }

    private Matrix invert3x3(Matrix m) {
        double m00 = m.get(0,0), m01 = m.get(0,1), m02 = m.get(0,2);
        double m10 = m.get(1,0), m11 = m.get(1,1), m12 = m.get(1,2);
        double m20 = m.get(2,0), m21 = m.get(2,1), m22 = m.get(2,2);

        // Calculate the determinant
        double det = m00 * (m11 * m22 - m12 * m21)
                - m01 * (m10 * m22 - m12 * m20)
                + m02 * (m10 * m21 - m11 * m20);

        // If determinant is roughly zero, the matrix is singular (this shouldn't happen with our math, but protects from crashes)
        if (Math.abs(det) < 1e-9) {
            return getIdentity3x3();
        }

        double invDet = 1.0 / det;

        // Calculate the adjugate matrix multiplied by 1/det
        double[][] inv = new double[3][3];
        inv[0][0] =  (m11 * m22 - m12 * m21) * invDet;
        inv[0][1] = -(m01 * m22 - m02 * m21) * invDet;
        inv[0][2] =  (m01 * m12 - m02 * m11) * invDet;
        inv[1][0] = -(m10 * m22 - m12 * m20) * invDet;
        inv[1][1] =  (m00 * m22 - m02 * m20) * invDet;
        inv[1][2] = -(m00 * m12 - m02 * m10) * invDet;
        inv[2][0] =  (m10 * m21 - m11 * m20) * invDet;
        inv[2][1] = -(m00 * m21 - m01 * m20) * invDet;
        inv[2][2] =  (m00 * m11 - m01 * m10) * invDet;

        return new Matrix(inv);
    }
}