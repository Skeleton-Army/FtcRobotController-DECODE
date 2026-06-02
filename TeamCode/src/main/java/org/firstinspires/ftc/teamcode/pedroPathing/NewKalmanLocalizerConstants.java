package org.firstinspires.ftc.teamcode.pedroPathing;

/**
 * Configuration and tuning constants for the NewKalmanLocalizer.
 * Isolated here to clean up localizer files and facilitate FTC Dashboard tuning.
 */

public class NewKalmanLocalizerConstants {

    // --- FTC Dashboard Tuning Parameters ---

    /** Penalty multiplier for linear speed */
    public static double k_v = 0.05;

    /** Penalty multiplier for rotational speed */
    public static double k_omega = 0.1;

    /** Penalty multiplier for Heading distance squared */
    public static double k_dh = 0.0001;

    // --- Distance Error Linear Regressions ---
    public static double RegressionSlopeX = -0.004991;
    public static double RegressionSlopeY = -0.005617;
    public static double RegressionFreeX = 0.5512;
    public static double RegressionFreeY = 0.6071;

    // --- Filter Safeguards & Thresholds ---

    /** Speed threshold in inches/sec where vision data is ignored completely */
    public static double maxSpeedThreshold = 40.0;

    /** Chi-square threshold for 3 DOF at 99% confidence. Rejects faulty camera jumps. */
    public static double mahalanobisThreshold = 11.34;

    /** Expected drift/error ratio of the Pinpoint odometry (e.g., 0.01% error = 0.0001) */
    public static double PINPOINT_ERROR_RATIO = 0.0001;

    // --- Instance Fields for Constructor Injection ---
    public final double[] processStdDevs;
    public final double[] baseMeasurementStdDevs;
    public final int bufferSize;

    /**
     * Default constructor to bundle the non-static setup parameters 
     * alongside the dashboard-tunable values.
     */
    public NewKalmanLocalizerConstants(
            double[] processStdDevs,
            double[] baseMeasurementStdDevs,
            int bufferSize
    ) {
        this.processStdDevs = processStdDevs;
        this.baseMeasurementStdDevs = baseMeasurementStdDevs;
        this.bufferSize = bufferSize;
    }
}
