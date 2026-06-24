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

    @BeforeEach
    public void setUp() {
        // Manually initialize Mockito without needing the Jupiter extension
        MockitoAnnotations.openMocks(this);

        // Mock default behaviors for the dead reckoning localizer
        when(mockDeadReckoning.getVelocity()).thenReturn(new Pose(0, 0, 0));
        when(mockDeadReckoning.getVelocityVector()).thenReturn(new Vector(0, 0));

        // Initialize the FusionLocalizer with standard FTC-scale variances
        fusionLocalizer = new FusionLocalizer(
                mockDeadReckoning,
                new Pose(0.1, 0.1, 0.1), // Initial Covariance
                new Pose(0.1, 0.1, 0.1), // Process Variance
                new Pose(0.1, 0.1, 0.1), // Measurement Variance
                100                      // Buffer Size
        );
    }

    @Test
    public void testAddMeasurement_Fixes700InchJumpAndHistoryCorruption() throws InterruptedException {
        // STEP 1: Set initial state at t=0
        when(mockDeadReckoning.getPose()).thenReturn(new Pose(0, 0, 0));
        fusionLocalizer.update();
        long time0 = System.nanoTime();

        // Advance time to simulate hardware cycle
        Thread.sleep(20);

        // STEP 2: Simulate moving forward 10 inches with a TINY NEGATIVE heading.
        // This tiny negative heading (-0.01 rad) is what triggers the 359-degree wrap-around
        // bug in logTwist causing the 700-inch jump.
        when(mockDeadReckoning.getPose()).thenReturn(new Pose(10, 0, -0.01));
        fusionLocalizer.update();
        long time1 = System.nanoTime();

        // Verify dead reckoning updated the pose correctly
        Pose currentPose = fusionLocalizer.getPose();
        assertEquals(10.0, currentPose.getX(), 0.1);

        // STEP 3: Inject a delayed vision measurement halfway between time0 and time1.
        // We simulate the camera saying we were actually at X=4 instead of X=5.
        long interpolatedTime = (time0 + time1) / 2;
        Pose delayedVisionMeasurement = new Pose(4, 0, -0.005);

        // Call the method containing the fixes
        fusionLocalizer.addMeasurement(delayedVisionMeasurement, interpolatedTime);

        // STEP 4: Assert Bug 1 (700-inch jump) is fixed.
        // If the bug exists, X and Y will be massive (NaN or > 100).
        // If fixed, X should be slightly pulled back from 10 (around 9.0) due to the Kalman update.
        Pose postUpdatePose = fusionLocalizer.getPose();
        assertFalse(fusionLocalizer.isNAN(), "Pose exploded into NaN due to Bug 1!");
        assertTrue(postUpdatePose.getX() < 15.0, "Pose jumped massively! Bug 1 still exists.");
        assertTrue(Math.abs(postUpdatePose.getY()) < 5.0, "Pose jumped massively! Bug 1 still exists.");

        // STEP 5: Assert Bug 2 (History Corruption) is fixed.
        // Advance time again to create a third node in history.
        Thread.sleep(20);
        when(mockDeadReckoning.getPose()).thenReturn(new Pose(20, 0, -0.02));
        fusionLocalizer.update();
        long time2 = System.nanoTime();

        // Inject a SECOND delayed measurement between time1 and time2.
        // If Bug 2 exists (history chain broken), the partial transform math will multiply
        // incorrectly, causing this second update to jump wildly.
        long secondInterpolatedTime = (time1 + time2) / 2;
        Pose secondVisionMeasurement = new Pose(14, 0, -0.015);

        fusionLocalizer.addMeasurement(secondVisionMeasurement, secondInterpolatedTime);

        Pose finalPose = fusionLocalizer.getPose();
        // FIX: Call isNAN() on fusionLocalizer, not finalPose
        assertFalse(fusionLocalizer.isNAN(), "Pose exploded on second update due to Bug 2!");
        assertTrue(finalPose.getX() < 25.0, "History corrupted causing massive drift!");
    }
    @Test
    public void testLongHistoryWithHighVarianceMeasurement() throws InterruptedException {
        // We will store exactly when each update happened to pull a delayed timestamp later
        long[] timestamps = new long[50];

        // STEP 1: Set initial state
        Pose currentOdo = new Pose(0, 0, 0);
        when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
        fusionLocalizer.update();
        timestamps[0] = System.nanoTime();

        // STEP 2: Simulate 50 hardware cycles of continuous arcing movement
        for (int i = 1; i < 50; i++) {
            Thread.sleep(10); // Simulate 10ms passing between loop iterations

            // Move forward and turn slightly to create a complex curved path
            currentOdo = new Pose(
                    currentOdo.getX() + 2.0,
                    currentOdo.getY() + 0.5,
                    currentOdo.getHeading() + 0.05
            );
            when(mockDeadReckoning.getPose()).thenReturn(currentOdo);
            fusionLocalizer.update();

            timestamps[i] = System.nanoTime();
        }

        // STEP 3: Record the pure, uninterrupted dead-reckoning pose
        Pose preMeasurementPose = fusionLocalizer.getPose();

        // STEP 4: Inject a measurement exactly halfway through the recorded history
        long delayedTimestamp = timestamps[25];

        // We simulate a wildly wrong camera reading (e.g. seeing an AprilTag on the wrong field wall)
        Pose terribleMeasurement = new Pose(500, -500, Math.PI);

        // We provide a massive measurement variance.
        // This tells the Kalman Filter "I have zero confidence in this camera reading."
        Pose massiveVariance = new Pose(1e9, 1e9, 1e9);

        fusionLocalizer.addMeasurement(terribleMeasurement, delayedTimestamp, massiveVariance);

        // STEP 5: Verify the pose remains completely undisturbed
        Pose postMeasurementPose = fusionLocalizer.getPose();

        // If Bug 1 (Explosion) or Bug 2 (History Corruption) still exist, splitting the
        // transform at step 25 and replaying the history will cause these assertions to fail
        // because the relative transforms will mathematically miss-align.
        assertFalse(fusionLocalizer.isNAN(), "Pose exploded to NaN during loop or replay!");

        // Assert that the X, Y, and Heading are identical to the pure odometry path
        // within a 0.01 tolerance margin.
        assertEquals(preMeasurementPose.getX(), postMeasurementPose.getX(), 0.01, "X position drifted during history replay!");
        assertEquals(preMeasurementPose.getY(), postMeasurementPose.getY(), 0.01, "Y position drifted during history replay!");
        assertEquals(preMeasurementPose.getHeading(), postMeasurementPose.getHeading(), 0.01, "Heading drifted during history replay!");
    }
}