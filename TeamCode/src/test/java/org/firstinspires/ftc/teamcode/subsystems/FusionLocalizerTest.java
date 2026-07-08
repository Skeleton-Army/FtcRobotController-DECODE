package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.utilities.FusionLocalizer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class FusionLocalizerTest {

    @Mock
    private Localizer mockDeadReckoning;

    private FusionLocalizer fusionLocalizer;

    // A variance so high the Kalman Filter will multiply the measurement by practically 0.
    private final Pose MASSIVE_VARIANCE = new Pose(1e12, 1e12, 1e12);

    @BeforeEach
    public void setUp() {
        MockitoAnnotations.openMocks(this);

        when(mockDeadReckoning.getVelocity()).thenReturn(new Pose(0, 0, 0));
        when(mockDeadReckoning.getVelocityVector()).thenReturn(new Vector(0, 0));

        fusionLocalizer = new FusionLocalizer(
                mockDeadReckoning,
                new Pose(0.1, 0.1, 0.1),
                new Pose(0.001, 0.001, 0.01),
                new Pose(1, 1, 100),
                200 // Increased buffer size for longer tests
        );
    }

    // ... [Keep your existing testAddMeasurement_Fixes700InchJumpAndHistoryCorruption here] ...

    @Test
    public void testFigureEightPath_HighVarianceMeasurement_NoDrift() throws InterruptedException {
        int steps = 120;
        long[] timestamps = new long[steps];
        Pose currentOdo = new Pose(0, 0, 0);

        when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
        fusionLocalizer.update();
        timestamps[0] = System.nanoTime();

        // Simulate a Figure-8 path (alternating angular velocity)
        for (int i = 1; i < steps; i++) {
            Thread.sleep(2);

            // Sine wave heading creates a figure 8
            double heading = Math.sin(i * 0.1);
            currentOdo = new Pose(
                    currentOdo.getX() + 1.5 * Math.cos(heading),
                    currentOdo.getY() + 1.5 * Math.sin(heading),
                    heading
            );
            when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
            fusionLocalizer.update();
            timestamps[i] = System.nanoTime();
        }

        Pose preMeasurementPose = fusionLocalizer.getPose();
        long delayedTimestamp = timestamps[60]; // Exactly in the middle of the complex curve
        Pose terribleMeasurement = new Pose(-1000, 1000, Math.PI);

        fusionLocalizer.addMeasurement(terribleMeasurement, delayedTimestamp, MASSIVE_VARIANCE);
        Pose postMeasurementPose = fusionLocalizer.getPose();

        assertPoseEquals("Figure 8 Path Drifted!", preMeasurementPose, postMeasurementPose, 0.001);
    }

    @Test
    public void testContinuousSpinning_HighVarianceMeasurement_NoDrift() throws InterruptedException {
        int steps = 150;
        long[] timestamps = new long[steps];
        Pose currentOdo = new Pose(0, 0, 0);

        when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
        fusionLocalizer.update();
        timestamps[0] = System.nanoTime();

        // Simulate a robot driving in a tight circle very fast
        for (int i = 1; i < steps; i++) {
            Thread.sleep(2);

            currentOdo = new Pose(
                    currentOdo.getX() + 2.0,
                    currentOdo.getY() + 2.0,
                    currentOdo.getHeading() + 0.3 // ~17 degrees per tick (very fast spin)
            );
            when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
            fusionLocalizer.update();
            timestamps[i] = System.nanoTime();
        }

        Pose preMeasurementPose = fusionLocalizer.getPose();
        long delayedTimestamp = timestamps[75];
        Pose terribleMeasurement = new Pose(0, 0, 0);

        fusionLocalizer.addMeasurement(terribleMeasurement, delayedTimestamp, MASSIVE_VARIANCE);
        Pose postMeasurementPose = fusionLocalizer.getPose();

        assertPoseEquals("Continuous Spin Drifted!", preMeasurementPose, postMeasurementPose, 0.001);
    }

    @Test
    public void testMultipleDelayedMeasurements_HighVariance_NoDrift() throws InterruptedException {
        int steps = 100;
        long[] timestamps = new long[steps];
        Pose currentOdo = new Pose(0, 0, 0);

        when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
        fusionLocalizer.update();
        timestamps[0] = System.nanoTime();

        // Standard curved path
        for (int i = 1; i < steps; i++) {
            Thread.sleep(2);
            currentOdo = new Pose(
                    currentOdo.getX() + 1.0,
                    currentOdo.getY() + 0.2,
                    currentOdo.getHeading() + 0.02
            );
            when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
            fusionLocalizer.update();
            timestamps[i] = System.nanoTime();
        }

        Pose preMeasurementPose = fusionLocalizer.getPose();

        // Inject three separate, highly delayed measurements at different points in history
        // Doing this out-of-order stress-tests the map key splicing.
        fusionLocalizer.addMeasurement(new Pose(999, 999, 0), timestamps[80], MASSIVE_VARIANCE);
        fusionLocalizer.addMeasurement(new Pose(-999, -999, 0), timestamps[20], MASSIVE_VARIANCE);
        fusionLocalizer.addMeasurement(new Pose(500, -500, 0), timestamps[50], MASSIVE_VARIANCE);

        Pose postMeasurementPose = fusionLocalizer.getPose();

        assertPoseEquals("Multiple Map Injections Drifted!", preMeasurementPose, postMeasurementPose, 0.001);
    }

    @Test
    public void testStationaryToSuddenAcceleration_HighVariance_NoDrift() throws InterruptedException {
        int steps = 100;
        long[] timestamps = new long[steps];
        Pose currentOdo = new Pose(0, 0, 0);

        when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
        fusionLocalizer.update();
        timestamps[0] = System.nanoTime();

        // Step 1: Sit perfectly still for 50 ticks
        for (int i = 1; i < 50; i++) {
            Thread.sleep(2);
            when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
            fusionLocalizer.update();
            timestamps[i] = System.nanoTime();
        }

        // Step 2: Suddenly move very fast
        for (int i = 50; i < steps; i++) {
            Thread.sleep(2);
            currentOdo = new Pose(
                    currentOdo.getX() + 5.0,
                    currentOdo.getY() - 2.0,
                    currentOdo.getHeading() - 0.1
            );
            when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
            fusionLocalizer.update();
            timestamps[i] = System.nanoTime();
        }

        Pose preMeasurementPose = fusionLocalizer.getPose();

        // Inject measurement right at the inflection point where movement starts
        long delayedTimestamp = timestamps[50];
        Pose terribleMeasurement = new Pose(100, 100, Math.PI / 2);

        fusionLocalizer.addMeasurement(terribleMeasurement, delayedTimestamp, MASSIVE_VARIANCE);
        Pose postMeasurementPose = fusionLocalizer.getPose();

        assertPoseEquals("Stationary/Acceleration Transition Drifted!", preMeasurementPose, postMeasurementPose, 0.001);
    }

    /**
     * Helper method to ensure strict equality and trap NaNs.
     */
    private void assertPoseEquals(String message, Pose expected, Pose actual, double delta) {
        assertFalse(Double.isNaN(actual.getX()) || Double.isNaN(actual.getY()) || Double.isNaN(actual.getHeading()),
                message + " (Pose became NaN)");

        assertEquals(expected.getX(), actual.getX(), delta, message + " (X mismatch)");
        assertEquals(expected.getY(), actual.getY(), delta, message + " (Y mismatch)");
        assertEquals(expected.getHeading(), actual.getHeading(), delta, message + " (Heading mismatch)");
    }
}