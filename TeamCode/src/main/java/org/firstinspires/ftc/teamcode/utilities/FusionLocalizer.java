package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedroPathing.math.Matrix;

import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Extended Kalman Filter localizer that fuses a dead-reckoning {@link Localizer} (e.g. the
 * goBILDA Pinpoint) with retroactively-timestamped vision pose measurements (e.g. AprilTag).
 */
public class FusionLocalizer implements Localizer {

    private static class KalmanState {
        Pose pose;
        Pose twist;
        Pose relativeTransform;
        Matrix covariance;

        KalmanState(Pose pose, Pose twist, Pose relativeTransform, Matrix covariance) {
            this.pose = pose;
            this.twist = twist;
            this.relativeTransform = relativeTransform;
            this.covariance = covariance;
        }
    }

    private static final double EPSILON = 1e-10;
    private static final double SINGULARITY_EPSILON = 1e-9;
    private static final long MAX_PLAUSIBLE_TIMESTAMP_SKEW_NANOS = (long) 5e9;
    private static final long DEFAULT_MAX_HISTORY_AGE_NANOS = (long) 1e9;

    public double[] chiSquaredThreshold = {6.63, 9.21, 11.34};

    private final Localizer deadReckoning;
    private final Matrix Q;
    private final Matrix R;
    private final Matrix initialCovariance;
    private final int bufferSize;
    private final long maxHistoryAgeNanos;

    private Pose currentRawPose;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Pose currentRelativeTransform;
    private Matrix P;
    private long lastUpdateTime = -1;

    private boolean ignoreCameraHeading = true;

    private final NavigableMap<Long, KalmanState> history = new TreeMap<>();

    private Matrix lastInnovationCovariance;
    private Matrix lastKalmanGain;
    private double lastNIS = Double.NaN;
    private int lastMeasurementDof = 0;
    private boolean lastMeasurementAccepted = false;

    public FusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize
    ) {
        this(deadReckoning, initialCovariance, processVariance, measurementVariance, bufferSize,
                DEFAULT_MAX_HISTORY_AGE_NANOS);
    }

    public FusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize,
            long maxHistoryAgeNanos
    ) {
        this.deadReckoning = deadReckoning;
        this.bufferSize = bufferSize;
        this.maxHistoryAgeNanos = maxHistoryAgeNanos;

        this.currentPosition = new Pose();
        this.currentRawPose = new Pose();
        this.currentVelocity = new Pose();
        this.currentRelativeTransform = new Pose();

        this.initialCovariance = Matrix.diag(
                Math.max(initialCovariance.getX(), EPSILON),
                Math.max(initialCovariance.getY(), EPSILON),
                Math.max(initialCovariance.getHeading(), EPSILON)
        );
        this.P = this.initialCovariance.copy();

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

        history.put(0L, new KalmanState(currentPosition.copy(), new Pose(), currentPosition.copy(), P.copy()));
    }

    public void setIgnoreCameraHeading(boolean ignore) {
        this.ignoreCameraHeading = ignore;
    }

    @Override
    public synchronized void update() {
        deadReckoning.update();
        long now = System.nanoTime();
        lastUpdateTime = now;

        currentVelocity = deadReckoning.getVelocity().copy();

        Pose rawPose = deadReckoning.getPose();
        currentRelativeTransform = compose(invert(currentRawPose), rawPose);

        P = updateCovariance(P, currentPosition, currentRelativeTransform);
        currentPosition = compose(currentPosition, currentRelativeTransform);
        currentRawPose = rawPose.copy();

        history.put(now, new KalmanState(currentPosition.copy(), currentVelocity.copy(),
                currentRelativeTransform.copy(), P.copy()));
        trimHistory(now);
    }

    private void trimHistory(long now) {
        while (history.size() > bufferSize) {
            history.pollFirstEntry();
        }
        long cutoff = now - maxHistoryAgeNanos;
        while (history.size() > 1 && history.firstKey() < cutoff) {
            history.pollFirstEntry();
        }
    }

    private Matrix updateCovariance(Matrix P, Pose pose, Pose localDelta) {
        double distX = localDelta.getX();
        double distY = localDelta.getY();
        double distTheta = localDelta.getHeading();

        double heading = pose.getHeading();
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);

        double f02 = -distX * sinH - distY * cosH;
        double f12 =  distX * cosH - distY * sinH;

        Matrix F = new Matrix(new double[][] {
                {1.0, 0.0, f02},
                {0.0, 1.0, f12},
                {0.0, 0.0, 1.0}
        });

        Matrix bodyQ = Matrix.diag(
                Math.abs(distX) * Q.get(0, 0),
                Math.abs(distY) * Q.get(1, 1),
                Math.abs(distTheta) * Q.get(2, 2)
        );

        Matrix rotation = Matrix.createRotation(heading);
        Matrix worldQ = rotation.multiply(bodyQ).multiply(rotation.transposed());

        Matrix FPFt = F.multiply(P).multiply(F.transposed());

        /*for (int i = 0; i < 3; i++) {
            if (FPFt.get(i,i) < P.get(i,i)) {
                FPFt.set(i,i, P.get(i,i));
            }
        }*/

        Matrix result = FPFt.plus(worldQ);
        forceSymmetric(result);
        clampDiagonal(result);
        return result;
    }

    private KalmanState getKalmanState() {
        return new KalmanState(currentPosition.copy(), currentVelocity.copy(),
                currentRelativeTransform.copy(), P.copy());
    }

    public boolean addMeasurement(Pose measuredPose, long timestamp) {
        return addMeasurement(measuredPose, timestamp, null);
    }

    public synchronized boolean addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        lastMeasurementAccepted = false;

        long skew = Math.abs(System.nanoTime() - timestamp);
        if (skew > MAX_PLAUSIBLE_TIMESTAMP_SKEW_NANOS) {
            throw new IllegalArgumentException(
                    "addMeasurement timestamp is " + (skew / 1e9) + "s from System.nanoTime().");
        }

        Matrix measurementR;
        if (measurementVariance == null) {
            measurementR = R;
        } else {
            measurementR = Matrix.diag(
                    Math.max(measurementVariance.getX(), EPSILON),
                    Math.max(measurementVariance.getY(), EPSILON),
                    Math.max(measurementVariance.getHeading(), EPSILON)
            );
        }

        if (history.isEmpty() || timestamp < history.firstKey() || timestamp > history.lastKey())
            return false;

        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);
        boolean splicingNewNode = lowerKey != null && upperKey != null && !lowerKey.equals(upperKey);

        /*===================================*/
        /* Check if interpolate method is implemented correctly */
        KalmanState interpolated = interpolate(timestamp, lowerKey, upperKey);
        /*===================================*/

        if (interpolated == null) interpolated = getKalmanState();
        Pose pastPose = interpolated.pose;

        boolean measX = !Double.isNaN(measuredPose.getX());
        boolean measY = !Double.isNaN(measuredPose.getY());
        boolean measH = !Double.isNaN(measuredPose.getHeading()) && !ignoreCameraHeading;

        int dof = (measX ? 1 : 0) + (measY ? 1 : 0) + (measH ? 1 : 0);
        if (dof == 0) return false;

        int[] activeIndices = new int[dof];
        int idx = 0;
        if (measX) activeIndices[idx++] = 0;
        if (measY) activeIndices[idx++] = 1;
        if (measH) activeIndices[idx++] = 2;

        double[] yFull = {
                measX ? measuredPose.getX() - pastPose.getX() : 0,
                measY ? measuredPose.getY() - pastPose.getY() : 0,
                measH ? MathFunctions.normalizeAngleSigned(measuredPose.getHeading() - pastPose.getHeading()) : 0
        };

        Matrix Pm = interpolated.covariance;

        Matrix ySub = extractRows(yFull, activeIndices);
        Matrix Psub = extractSquareSubmatrix(Pm, activeIndices);
        Matrix Rsub = extractSquareSubmatrix(measurementR, activeIndices);
        Matrix PHt = extractColumns(Pm, activeIndices);

        Matrix Ssub = Psub.plus(Rsub);
        forceSymmetric(Ssub);

        Matrix SsubInv = invertSmall(Ssub);
        if (SsubInv == null) return false;

        double nis = ySub.transposed().multiply(SsubInv).multiply(ySub).get(0, 0);
        lastNIS = nis;
        lastMeasurementDof = dof;
        lastInnovationCovariance = Ssub;

        if (nis > chiSquaredThreshold[dof - 1]) return false;

        Matrix Ksub = PHt.multiply(SsubInv);
        lastKalmanGain = Ksub;

        Matrix Ky = Ksub.multiply(ySub);
        Pose updatedPast = new Pose(
                pastPose.getX() + Ky.get(0, 0),
                pastPose.getY() + Ky.get(1, 0),
                MathFunctions.normalizeAngle(pastPose.getHeading() + Ky.get(2, 0))
        );

        Matrix KH = embedColumns(Ksub, activeIndices, 3);
        Matrix I = Matrix.identity(3);
        Matrix IKH = I.minus(KH);
        Matrix updatedCovariance = IKH.multiply(Pm).multiply(IKH.transposed())
                .plus(Ksub.multiply(Rsub).multiply(Ksub.transposed()));
        forceSymmetric(updatedCovariance);
        clampDiagonal(updatedCovariance);

        if (splicingNewNode) {
            double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);
            KalmanState upperState = history.get(upperKey);
            upperState.relativeTransform = scaleTransform(upperState.relativeTransform, 1.0 - ratio);
        }

        // --- BUG FIX: Explicitly copy state values when writing back to history buffer ---
        history.put(timestamp, new KalmanState(updatedPast.copy(), interpolated.twist.copy(),
                interpolated.relativeTransform.copy(), updatedCovariance.copy()));

        Pose prevPose = updatedPast.copy();
        Matrix prevCov = updatedCovariance.copy();

        for (NavigableMap.Entry<Long, KalmanState> entry : history.tailMap(timestamp, false).entrySet()) {
            KalmanState st = entry.getValue();

            Pose nextPose = compose(prevPose, st.relativeTransform);
            Matrix nextCov = updateCovariance(prevCov, prevPose, st.relativeTransform);

            // --- BUG FIX: Use deep copies to secure the nodes against reference leakage ---
            st.pose = nextPose.copy();
            st.covariance = nextCov.copy();

            prevPose = nextPose.copy();
            prevCov = nextCov.copy();
        }

        currentPosition = prevPose.copy();
        P = prevCov.copy();
        trimHistory(System.nanoTime());
        lastMeasurementAccepted = true;
        return true;
    }

    private KalmanState interpolate(long timestamp, Long lowerKey, Long upperKey) {
        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey);

        KalmanState lower = history.get(lowerKey);
        KalmanState upper = history.get(upperKey);
        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        Pose scaledDelta = scaleTransform(upper.relativeTransform, ratio);
        Pose interpPose = compose(lower.pose, scaledDelta);
        Pose interpTwist = lerpPoseLinear(lower.twist, upper.twist, ratio);
        Matrix interpCov = updateCovariance(lower.covariance, lower.pose, scaledDelta);

        return new KalmanState(interpPose, interpTwist, scaledDelta, interpCov);
    }

    private static Pose lerpPoseLinear(Pose a, Pose b, double ratio) {
        double x = a.getX() + ratio * (b.getX() - a.getX());
        double y = a.getY() + ratio * (b.getY() - a.getY());
        double headingDiff = MathFunctions.getSmallestAngleDifference(b.getHeading(), a.getHeading());
        double heading = MathFunctions.normalizeAngle(a.getHeading() + ratio * headingDiff);
        return new Pose(x, y, heading);
    }

    public static Pose scaleTransform(Pose delta, double ratio) {
        double x = delta.getX() * ratio;
        double y = delta.getY() * ratio;
        double heading = MathFunctions.normalizeAngleSigned(delta.getHeading()) * ratio;
        return new Pose(x, y, heading);
    }

    public static Pose invert(Pose pose) {
        double cos = Math.cos(pose.getHeading());
        double sin = Math.sin(pose.getHeading());

        double x = -pose.getX() * cos - pose.getY() * sin;
        double y = pose.getX() * sin - pose.getY() * cos;
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

    private static Matrix extractRows(double[] vec, int[] indices) {
        double[][] out = new double[indices.length][1];
        for (int i = 0; i < indices.length; i++) out[i][0] = vec[indices[i]];
        return new Matrix(out);
    }

    private static Matrix extractSquareSubmatrix(Matrix m, int[] indices) {
        double[][] out = new double[indices.length][indices.length];
        for (int i = 0; i < indices.length; i++)
            for (int j = 0; j < indices.length; j++)
                out[i][j] = m.get(indices[i], indices[j]);
        return new Matrix(out);
    }

    private static Matrix extractColumns(Matrix m, int[] colIndices) {
        int rows = m.getRows();
        double[][] out = new double[rows][colIndices.length];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < colIndices.length; j++)
                out[i][j] = m.get(i, colIndices[j]);
        return new Matrix(out);
    }

    private static Matrix embedColumns(Matrix sub, int[] colIndices, int n) {
        double[][] out = new double[n][n];
        for (int m = 0; m < colIndices.length; m++)
            for (int i = 0; i < n; i++)
                out[i][colIndices[m]] = sub.get(i, m);
        return new Matrix(out);
    }

    private static Matrix invertSmall(Matrix m) {
        int n = m.getRows();
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = m.get(i, j);
            aug[i][n + i] = 1.0;
        }

        for (int col = 0; col < n; col++) {
            int pivotRow = col;
            double maxVal = Math.abs(aug[col][col]);
            for (int r = col + 1; r < n; r++) {
                if (Math.abs(aug[r][col]) > maxVal) {
                    maxVal = Math.abs(aug[r][col]);
                    pivotRow = r;
                }
            }
            if (maxVal < SINGULARITY_EPSILON) return null;

            double[] tmp = aug[col];
            aug[col] = aug[pivotRow];
            aug[pivotRow] = tmp;

            double pivot = aug[col][col];
            for (int k = 0; k < 2 * n; k++) aug[col][k] /= pivot;

            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                if (factor == 0) continue;
                for (int k = 0; k < 2 * n; k++) aug[r][k] -= factor * aug[col][k];
            }
        }

        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++)
            System.arraycopy(aug[i], n, inv[i], 0, n);
        return new Matrix(inv);
    }

    private static void forceSymmetric(Matrix m) {
        int n = m.getRows();
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double avg = (m.get(i, j) + m.get(j, i)) / 2.0;
                m.set(i, j, avg);
                m.set(j, i, avg);
            }
        }
    }

    private static void clampDiagonal(Matrix m) {
        int n = m.getRows();
        for (int i = 0; i < n; i++) {
            if (m.get(i, i) < EPSILON) m.set(i, i, EPSILON);
        }
    }

    @Override
    public synchronized void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        resetTo(0L, setStart, initialCovariance);
    }

    @Override
    public synchronized void setPose(Pose setPose) {
        setPose(setPose, null);
    }

    public synchronized void setPose(Pose setPose, Pose newCovariance) {
        deadReckoning.setPose(setPose);
        Matrix covToUse = newCovariance == null
                ? initialCovariance.copy()
                : Matrix.diag(
                Math.max(newCovariance.getX(), EPSILON),
                Math.max(newCovariance.getY(), EPSILON),
                Math.max(newCovariance.getHeading(), EPSILON));
        long key = lastUpdateTime >= 0 ? lastUpdateTime : System.nanoTime();
        resetTo(key, setPose, covToUse);
    }

    private void resetTo(long key, Pose pose, Matrix covariance) {
        currentPosition = pose.copy();
        currentRawPose = pose.copy();
        currentVelocity = new Pose();
        currentRelativeTransform = new Pose();
        P = covariance.copy();
        history.clear();
        history.put(key, new KalmanState(currentPosition.copy(), currentVelocity.copy(),
                currentPosition.copy(), P.copy()));
    }

    @Override
    public synchronized Pose getPose() {
        return currentPosition.copy();
    }

    @Override
    public synchronized Pose getVelocity() {
        return (currentVelocity != null ? currentVelocity : deadReckoning.getVelocity()).copy();
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public double getTotalHeading() {
        return currentPosition.getHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return deadReckoning.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return deadReckoning.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return deadReckoning.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        deadReckoning.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return deadReckoning.getIMUHeading();
    }

    @Override
    public synchronized boolean isNAN() {
        return Double.isNaN(currentPosition.getX())
                || Double.isNaN(currentPosition.getY())
                || Double.isNaN(currentPosition.getHeading());
    }

    public Localizer getDeadReckoning() {
        return deadReckoning;
    }

    public synchronized Matrix getP() {
        return P;
    }

    public synchronized Matrix getInnovationCovariance() {
        return lastInnovationCovariance;
    }

    public synchronized Matrix getKalmanGain() {
        return lastKalmanGain;
    }

    public synchronized double getLastNIS() {
        return lastNIS;
    }

    public synchronized int getLastMeasurementDof() {
        return lastMeasurementDof;
    }

    public synchronized boolean getLastMeasurementAccepted() {
        return lastMeasurementAccepted;
    }
}