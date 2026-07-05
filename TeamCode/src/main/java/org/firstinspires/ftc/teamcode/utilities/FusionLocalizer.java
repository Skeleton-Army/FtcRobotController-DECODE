package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private static class KalmanState {
        Pose pose;
        Pose twist;
        Pose relativeTransform;
        Matrix covariance;

        public KalmanState(Pose pose, Pose twist, Pose relativeTransform, Matrix covariance) {
            this.pose = pose;
            this.twist = twist;
            this.relativeTransform = relativeTransform;
            this.covariance = covariance;
        }
    }

    public static double EPSILON = 1e-6; //floor for covariance matrices
    public static double OUTLIER_THRESHOLD = 9.21; // Mahalanobis^2 gate, ~99% for 2 active dims
    public static double MAX_TRANSLATION_ERROR = 10.0; // inches

    private final Localizer deadReckoning;
    private Pose currentRawPose;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Pose currentRelativeTransform;
    private Matrix P; //State Covariance
    private final Matrix Q; //Process Noise Covariance
    private final Matrix R; //Measurement Noise Covariance
    private long lastUpdateTime = -1;
    private final NavigableMap<Long, KalmanState> history = new TreeMap<>();
    private final int bufferSize;

    public FusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose();
        currentRawPose = new Pose();

        //Standard Deviations for Kalman Filter
        this.P = Matrix.diag(
                Math.max(initialCovariance.getX(), EPSILON),
                Math.max(initialCovariance.getY(), EPSILON),
                Math.max(initialCovariance.getHeading(), EPSILON)
        );
        this.Q = Matrix.diag(
                Math.max(processVariance.getX(), EPSILON),
                Math.max(processVariance.getY(), EPSILON),
                Math.max(processVariance.getHeading(), EPSILON)
        );
        this.R = Matrix.diag(
                Math.max(measurementVariance.getX(), EPSILON),
                Math.max(measurementVariance.getY(), EPSILON),
                Math.max(measurementVariance.getHeading(), EPSILON)
        );
        this.bufferSize = bufferSize;
        history.put(0L, new KalmanState(currentPosition, new Pose(), currentRawPose, P));
    }

    @Override
    public void update() {
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        currentVelocity = deadReckoning.getVelocity();

        Pose rawPose = deadReckoning.getPose();
        currentRelativeTransform = compose(invert(currentRawPose), rawPose);

        P = updateCovariance(P, currentPosition, currentVelocity, dt);
        currentPosition = compose(currentPosition, currentRelativeTransform);
        currentRawPose = rawPose;

        history.put(now, new KalmanState(currentPosition, currentVelocity, currentRelativeTransform, P));
        if (history.size() > bufferSize) history.pollFirstEntry();
    }

    private Matrix updateCovariance(Matrix P, Pose pose, Pose twist, double dt) {
        Pose bodyTwist = twist.rotate(-pose.getHeading(), false);
        double dist_x = Math.abs(bodyTwist.getX() * dt);
        double dist_y = Math.abs(bodyTwist.getY() * dt);
        double dist_theta = Math.abs(bodyTwist.getHeading() * dt);

        double q_x = Q.get(0, 0);
        double q_y = Q.get(1, 1);
        double q_theta = Q.get(2, 2);

        Matrix bodyQ = Matrix.diag(
                dist_x * q_x,
                dist_y * q_y,
                dist_theta * q_theta
        );

        Matrix rotation = Matrix.createRotation(pose.getHeading());
        Matrix worldQ = rotation.times(bodyQ).times(rotation.transposed());
        P = P.plus(worldQ);
        clampCovariance(P);
        return P;
    }

    private KalmanState getKalmanState() {
        return new KalmanState(currentPosition, currentVelocity, currentRelativeTransform, P);
    }

    public void addMeasurement(Pose measuredPose, long timestamp) {
        addMeasurement(measuredPose, timestamp, null);
    }

    public void addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        Matrix measurementR = measurementVariance == null ? R :
                Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        clampCovariance(measurementR);

        if (history.isEmpty() || timestamp < history.firstKey() || timestamp > history.lastKey())
            return;

        KalmanState interpolatedData = interpolate(timestamp);
        if (interpolatedData == null) interpolatedData = getKalmanState();
        Pose interpPose = interpolatedData.pose;

        if (interpPose == null)
            interpPose = getPose();

        boolean measX = !Double.isNaN(measuredPose.getX());
        boolean measY = !Double.isNaN(measuredPose.getY());
        boolean measH = !Double.isNaN(measuredPose.getHeading());

        double dx = measX ? measuredPose.getX() - interpPose.getX() : 0;
        double dy = measY ? measuredPose.getY() - interpPose.getY() : 0;
        double dh = measH ? MathFunctions.normalizeAngleSigned(measuredPose.getHeading() - interpPose.getHeading()) : 0;

        // --- Hard distance gate ---
        double translationError = Math.hypot(dx, dy);
        if (translationError > MAX_TRANSLATION_ERROR) {
            return; // AprilTag pose too far from odometry — reject outright
        }

        Matrix y = new Matrix(new double[][]{
                {dx},
                {dy},
                {dh}
        });

        Matrix M = Matrix.diag(
                measX ? 1 : 0,
                measY ? 1 : 0,
                measH ? 1 : 0
        );

        Matrix Pm = interpolatedData.covariance;
        Matrix S = Pm.plus(measurementR);
        Matrix S_inv = invert(S);
        if (S_inv == null) return;

        // --- Outlier rejection (Mahalanobis distance gate) ---
        Matrix yGated = M.multiply(y); // zero out unmeasured axes before testing
        Matrix mahalanobis = yGated.transposed().multiply(S_inv).multiply(yGated);
        double d2 = mahalanobis.get(0, 0);
        if (d2 > OUTLIER_THRESHOLD) {
            // Measurement is too inconsistent with current estimate — reject it.
            return;
        }

        Matrix K = Pm.multiply(S_inv);
        K = M.multiply(K);
        y = M.multiply(y);

        Matrix Ky = K.multiply(y);
        Pose updatedInterp = new Pose(
                interpPose.getX() + Ky.get(0, 0),
                interpPose.getY() + Ky.get(1, 0),
                MathFunctions.normalizeAngle(interpPose.getHeading() + Ky.get(2, 0))
        );

        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix cov = IK.multiply(Pm).multiply(IK.transposed()).plus(K.multiply(measurementR).multiply(K.transposed()));
        clampCovariance(cov);

        // 1. Put the newly updated interpolated state into history FIRST
        history.put(timestamp, new KalmanState(updatedInterp, interpolatedData.twist, interpolatedData.relativeTransform, cov));

        // 2. Fix the Double Counting by shrinking the relative transform of the strictly next state
        Long nextTime = history.higherKey(timestamp);
        if (nextTime != null) {
            KalmanState nextState = history.get(nextTime);

            Pose unupdatedNextPose = nextState.pose;
            Pose fixedRelativeTransform = compose(invert(interpPose), unupdatedNextPose);

            nextState.relativeTransform = fixedRelativeTransform;
        }

        // 3. Forward propagate pose + covariance
        long prevTime = timestamp;
        Pose prevPose = updatedInterp;

        for (NavigableMap.Entry<Long, KalmanState> entry : history.tailMap(timestamp, false).entrySet()) {
            long t = entry.getKey();
            Pose twist = entry.getValue().twist;
            if (twist == null) twist = getVelocity();

            double dt = (t - prevTime) / 1e9;
            Pose relativeTransform = entry.getValue().relativeTransform;

            cov = updateCovariance(cov, prevPose, twist, dt);
            prevPose = compose(prevPose, relativeTransform);

            history.put(t, new KalmanState(prevPose, twist, relativeTransform, cov));
            prevTime = t;
        }

        currentPosition = history.lastEntry().getValue().pose;
        P = history.lastEntry().getValue().covariance;
    }

    private KalmanState interpolate(long timestamp) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey);

        KalmanState lower = history.get(lowerKey);
        KalmanState upper = history.get(upperKey);
        Pose[] lowerKalmanState = new Pose[] {lower.pose, lower.twist};
        Pose[] upperKalmanState = new Pose[] {upper.pose, upper.twist};

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        Pose[] interpolData = new Pose[2]; // Reduced size to 2 to avoid NPEs
        for (int i = 0; i < 2; i++) {
            Pose lowerPose = lowerKalmanState[i];
            Pose upperPose = upperKalmanState[i];
            double x = lowerPose.getX() + ratio * (upperPose.getX() - lowerPose.getX());
            double y = lowerPose.getY() + ratio * (upperPose.getY() - lowerPose.getY());
            double headingDiff = MathFunctions.getSmallestAngleDifference(upperPose.getHeading(), lowerPose.getHeading());
            double heading = MathFunctions.normalizeAngle(lowerPose.getHeading() + ratio * headingDiff);
            interpolData[i] = new Pose(x, y, heading);
        }

        Pose rawPose = interpolData[0];
        Pose relativeTransform = compose(invert(lowerKalmanState[0]), rawPose);
        double dt = (timestamp - lowerKey) / 1e9;

        Matrix cov = updateCovariance(lower.covariance, lowerKalmanState[0], interpolData[1], dt);

        return new KalmanState(interpolData[0], interpolData[1], relativeTransform, cov);
    }

    public static Pose invert(Pose pose) {
        double cos = Math.cos(pose.getHeading());
        double sin = Math.sin(pose.getHeading());

        double x = -pose.getX() * cos - pose.getY() * sin;
        double y =  pose.getX() * sin - pose.getY() * cos;
        double h = -pose.getHeading();

        return new Pose(x, y, h);
    }

    public static Pose compose(Pose a, Pose b) {
        double cos = Math.cos(a.getHeading());
        double sin = Math.sin(a.getHeading());

        double x = a.getX() + b.getX() * cos - b.getY() * sin;
        double y = a.getY() + b.getX() * sin + b.getY() * cos;
        double h = MathFunctions.normalizeAngle(a.getHeading() + b.getHeading());

        return new Pose(x, y, h);
    }

    public static Pose interpolateTransform(Pose a, Pose b, double ratio) {
        double dx = a.getX() + ratio * (b.getX() - a.getX());
        double dy = a.getY() + ratio * (b.getY() - a.getY());
        double headingDiff = MathFunctions.getSmallestAngleDifference(b.getHeading(), a.getHeading());
        double dtheta = MathFunctions.normalizeAngle(a.getHeading() + ratio * headingDiff);

        double eps = 1e-4;
        double x, y;

        if (Math.abs(dtheta) < eps) {
            x = dx;
            y = dy;
        } else {
            double sinT = Math.sin(dtheta);
            double cosT = Math.cos(dtheta);
            double v = sinT / dtheta;
            double w = (1 - cosT) / dtheta;

            x = v * dx - w * dy;
            y = w * dx + v * dy;
        }

        return new Pose(x, y, dtheta);
    }

    private void clampCovariance(Matrix P) {
        for (int i = 0; i < 3; i++) {
            double v = P.get(i, i);
            if (v < EPSILON) {
                P.set(i, i, EPSILON);
            }
        }
    }

    private static Matrix invert(Matrix matrix) {
        if (matrix.getRows() != matrix.getColumns()) return null;

        Matrix I = Matrix.identity(matrix.getRows());
        Matrix[] r = Matrix.rref(matrix, I);

        if (!r[0].equals(I)) return null;
        return r[1];
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
    public void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        history.put(0L, new KalmanState(setStart, new Pose(), setStart, P));
        currentPosition = setStart;
        currentRawPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        currentPosition = setPose;
        deadReckoning.setPose(setPose);
        currentRawPose = setPose;

        if (!history.isEmpty())
            history.lastEntry().getValue().pose = setPose;
        else
            setStartPose(setPose);
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

    public Localizer getDeadReckoning() {
        return deadReckoning;
    }

    public Matrix getCovariance() {
        return P;
    }
}