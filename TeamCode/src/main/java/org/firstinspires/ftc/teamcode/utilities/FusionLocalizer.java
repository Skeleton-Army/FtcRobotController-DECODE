package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Port of WPILib's {@code PoseEstimator} fixed-gain fusion strategy to FTC/pedroPathing.
 * <p>
 * NOTE: this class is intentionally named {@code FusionLocalizer}, same as the full discrete-KF
 * version with the time-varying covariance/gain. The two CANNOT coexist in the same package as
 * written -- Java will reject the duplicate class name. Use this file to replace the original,
 * or move one of the two into a different package, before compiling.
 * <p>
 * This does NOT propagate a covariance matrix at runtime. Instead it computes a single,
 * fixed-per-axis Kalman gain up front from the closed-form steady-state solution of a
 * continuous KF with A = 0, C = I:
 * <pre>
 *     k = q / (q + sqrt(q * r))
 * </pre>
 * where q is process variance and r is measurement variance for that axis. A vision
 * measurement then nudges the estimate toward the vision pose by that fixed fraction, and the
 * resulting correction is stored as a *rigid SE(2) offset* (a {@link VisionUpdate}) that is
 * re-applied to every subsequent odometry pose until the next vision measurement arrives.
 * <p>
 * This is much cheaper than a full KF (no matrix inversion every update) at the cost of the
 * gain no longer adapting to how much the robot has moved or how confident we currently are.
 */
public class FusionLocalizer implements Localizer {

    /** Anchors a vision-compensated pose to the odometry pose it was computed against. */
    private static final class VisionUpdate {
        final Pose visionPose;
        final Pose odometryPose;

        VisionUpdate(Pose visionPose, Pose odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /** Re-expresses {@code pose} (an odometry pose) relative to this anchor's vision pose. */
        Pose compensate(Pose pose) {
            // delta = pose (-) odometryPose, in SE(2): how far has odometry moved since the anchor?
            Pose delta = compose(invert(odometryPose), pose);
            // visionPose (+) delta
            return compose(visionPose, delta);
        }
    }

    private static final long DEFAULT_BUFFER_DURATION_NANOS = (long) (1.5e9); // 1.5s, matches WPILib default

    private final Localizer deadReckoning;
    private final long bufferDurationNanos;

    // Diagonal of process noise covariance Q (variances, not std devs)
    private final double[] q = new double[3];

    public double[] getVisionK() {
        return visionK;
    }

    // Fixed per-axis Kalman gain
    private final double[] visionK = new double[3];

    // timestamp (ns) -> raw odometry pose
    private final NavigableMap<Long, Pose> odometryPoseBuffer = new TreeMap<>();
    // timestamp (ns) -> vision anchor
    private final NavigableMap<Long, VisionUpdate> visionUpdates = new TreeMap<>();

    private Pose poseEstimate;
    private Pose currentVelocity;

    public FusionLocalizer(Localizer deadReckoning, Pose stateStdDevs, Pose visionStdDevs) {
        this(deadReckoning, stateStdDevs, visionStdDevs, DEFAULT_BUFFER_DURATION_NANOS);
    }

    public FusionLocalizer(Localizer deadReckoning, Pose stateStdDevs, Pose visionStdDevs,
                                    long bufferDurationNanos) {
        this.deadReckoning = deadReckoning;
        this.bufferDurationNanos = bufferDurationNanos;
        this.poseEstimate = deadReckoning.getPose();

        q[0] = stateStdDevs.getX() * stateStdDevs.getX();
        q[1] = stateStdDevs.getY() * stateStdDevs.getY();
        q[2] = stateStdDevs.getHeading() * stateStdDevs.getHeading();

        setVisionMeasurementStdDevs(visionStdDevs);
    }

    /** Recomputes the fixed gain. Call this any time you want to change how much you trust vision. */
    public void setVisionMeasurementStdDevs(Pose visionStdDevs) {
        double[] r = new double[] {
                visionStdDevs.getX() * visionStdDevs.getX(),
                visionStdDevs.getY() * visionStdDevs.getY(),
                visionStdDevs.getHeading() * visionStdDevs.getHeading()
        };
        for (int i = 0; i < 3; i++) {
            visionK[i] = (q[i] == 0.0) ? 0.0 : q[i] / (q[i] + Math.sqrt(q[i] * r[i]));
        }
    }

    @Override
    public void update() {
        deadReckoning.update();
        currentVelocity = deadReckoning.getVelocity();

        long now = System.nanoTime();
        Pose rawOdometry = deadReckoning.getPose();
        odometryPoseBuffer.put(now, rawOdometry);
        trimOdometryBuffer();

        if (visionUpdates.isEmpty()) {
            poseEstimate = rawOdometry;
        } else {
            poseEstimate = visionUpdates.lastEntry().getValue().compensate(rawOdometry);
        }
    }

    private void trimOdometryBuffer() {
        if (odometryPoseBuffer.isEmpty()) return;
        long cutoff = odometryPoseBuffer.lastKey() - bufferDurationNanos;
        odometryPoseBuffer.headMap(cutoff, false).clear();
    }

    /** Linear (lerp, shortest-angle-on-heading) interpolation within the odometry buffer. */
    private Pose sampleOdometry(long timestamp) {
        if (odometryPoseBuffer.isEmpty()) return null;
        // MathFunctions.clamp returns a double, not a long, so clamp manually here.
        long earliest = odometryPoseBuffer.firstKey();
        long latest = odometryPoseBuffer.lastKey();
        long t = Math.max(earliest, Math.min(latest, timestamp));

        Long lo = odometryPoseBuffer.floorKey(t);
        Long hi = odometryPoseBuffer.ceilingKey(t);
        if (lo == null) return odometryPoseBuffer.get(hi);
        if (hi == null) return odometryPoseBuffer.get(lo);
        if (lo.equals(hi)) return odometryPoseBuffer.get(lo);

        Pose a = odometryPoseBuffer.get(lo);
        Pose b = odometryPoseBuffer.get(hi);
        double ratio = (double) (t - lo) / (hi - lo);

        double x = a.getX() + ratio * (b.getX() - a.getX());
        double y = a.getY() + ratio * (b.getY() - a.getY());
        double headingDiff = MathFunctions.getSmallestAngleDifference(b.getHeading(), a.getHeading());
        double h = MathFunctions.normalizeAngle(a.getHeading() + ratio * headingDiff);
        return new Pose(x, y, h);
    }

    private void cleanUpVisionUpdates() {
        if (odometryPoseBuffer.isEmpty()) return;
        long oldest = odometryPoseBuffer.firstKey();
        if (visionUpdates.isEmpty() || oldest < visionUpdates.firstKey()) return;
        long newestNeeded = visionUpdates.floorKey(oldest);
        visionUpdates.headMap(newestNeeded, false).clear();
    }

    /** Vision-compensated pose at an arbitrary past timestamp (for latency compensation). */
    public Pose sampleAt(long timestamp) {
        if (odometryPoseBuffer.isEmpty()) return null;

        if (visionUpdates.isEmpty() || timestamp < visionUpdates.firstKey()) {
            return sampleOdometry(timestamp);
        }

        long floor = visionUpdates.floorKey(timestamp);
        VisionUpdate anchor = visionUpdates.get(floor);
        Pose odomAtT = sampleOdometry(timestamp);
        if (odomAtT == null) return null;

        return anchor.compensate(odomAtT);
    }

    /**
     * Adds a latency-compensated vision measurement, nudging the estimate by the fixed gain
     * rather than recomputing a time-varying gain from covariance.
     */
    public void addVisionMeasurement(Pose visionPose, long timestampNanos) {
        if (odometryPoseBuffer.isEmpty()
                || odometryPoseBuffer.lastKey() - bufferDurationNanos > timestampNanos) {
            return;
        }

        //visionPose.minus(new Pose(0.5,0.5,0));
        cleanUpVisionUpdates();

        Pose odometrySample = sampleOdometry(timestampNanos);
        if (odometrySample == null) return;

        Pose visionSample = sampleAt(timestampNanos);
        if (visionSample == null) return;

        // transform = visionPose (-) visionSample  (residual, in the local frame of visionSample)
        Pose transform = compose(invert(visionSample), visionPose);

        // Scale residual by fixed per-axis gain (this is the part that replaces K = P*(P+R)^-1)
        Pose scaledTransform = new Pose(
                visionK[0] * transform.getX(),
                visionK[1] * transform.getY(),
                visionK[2] * transform.getHeading()
        );

        Pose newVisionPose = compose(visionSample, scaledTransform);
        VisionUpdate update = new VisionUpdate(newVisionPose, odometrySample);

        visionUpdates.put(timestampNanos, update);
        visionUpdates.tailMap(timestampNanos, false).clear(); // drop now-superseded later updates

        poseEstimate = update.compensate(deadReckoning.getPose());
    }

    // --- Localizer plumbing, mirrors FusionLocalizer ---

    @Override
    public Pose getPose() { return poseEstimate; }

    @Override
    public Pose getVelocity() {
        return currentVelocity != null ? currentVelocity : deadReckoning.getVelocity();
    }

    @Override
    public Vector getVelocityVector() { return getVelocity().getAsVector(); }

    @Override
    public void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        odometryPoseBuffer.clear();
        visionUpdates.clear();
        poseEstimate = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        deadReckoning.setPose(setPose);
        odometryPoseBuffer.clear();
        visionUpdates.clear();
        poseEstimate = setPose;
    }

    @Override
    public double getTotalHeading() { return poseEstimate.getHeading(); }

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
        return Double.isNaN(poseEstimate.getX()) || Double.isNaN(poseEstimate.getY())
                || Double.isNaN(poseEstimate.getHeading());
    }

    public Localizer getDeadReckoning() { return deadReckoning; }

    // --- Inlined SE(2) helpers (identical to the ones on the full-KF FusionLocalizer) ---
    // Kept local so this file has no compile-time dependency on the other class, since they
    // share a class name and can't both be on the classpath at once anyway.

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
}