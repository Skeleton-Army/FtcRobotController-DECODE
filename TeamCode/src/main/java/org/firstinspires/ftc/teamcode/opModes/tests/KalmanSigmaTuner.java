package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

/**
 * KalmanStaticTuner
 * * Determines the "Time-Independent Process Noise" (Static Sigma) for the Kalman Filter.
 * * THEORY:
 * Pinpoint drift behaves like a Random Walk. To find the process noise constant:
 * Sigma = (Change in Position) / sqrt(Time Delta)
 * * PROCEDURE:
 * 1. Robot stays STATIONARY.
 * 2. Auxiliary motors (Flywheel/Intake) run to simulate vibration.
 * 3. We collect 100 samples. Each sample is the average position over exactly 1 second.
 * 4. We compare samples across different time gaps (1s, 4s, 10s, full duration).
 * 5. The result is the specific 'staticSigma' value to use in your filter.
 */
@TeleOp(name = "Kalman Static Tuner", group = "Tuning")
@Disabled
public class KalmanSigmaTuner extends LinearOpMode {

    private Follower follower;

    // Replace these with your actual auxiliary motor names if you want code to run them
    // Or just turn them on manually before starting the test.
    private Shooter shooter;
    TimerEx timer;

    private List<Pose> timeSamples = new ArrayList<>();
    private final int SAMPLE_COUNT = 100; // 100 seconds of data

    IShooterCalculator shooterCalc;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize
        follower = Constants.createFollower(hardwareMap);
        //follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setPose(new Pose(72,72,0));

        shooterCalc = new ShooterCalculator(new ShooterCoefficients());

        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, shooterCalc, Alliance.RED);

        // Optional: Initialize auxiliary motors to create vibration
        // flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        telemetry.addLine("Ready to measure Static Drift.");
        telemetry.addLine("1. Ensure robot is STATIONARY.");
        telemetry.addLine("2. Ensure Intake/Flywheel are RUNNING (if applicable).");
        telemetry.addLine("3. Press START to begin 100s data collection.");
        telemetry.update();

        waitForStart();

        // Optional: Turn on motors
        // flywheelMotor.setPower(1.0);

        telemetry.addLine("Collecting Data... DO NOT TOUCH ROBOT.");
        telemetry.update();

        NanoTimer timer = new NanoTimer();
        timer.resetTimer();

        // --- PHASE 1: DATA COLLECTION ---
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            if (isStopRequested()) return;

            // Collect average position over 1 second
            double sumX = 0;
            double sumY = 0;
            double sumH = 0;
            int count = 0;

            double loopStartTime = timer.getElapsedTimeSeconds();

            // Loop for exactly 1.0 second
            while (timer.getElapsedTimeSeconds() - loopStartTime < 1.0 && opModeIsActive()) {
                follower.update();
                Pose current = follower.getPose();

                sumX += current.getX();
                sumY += current.getY();
                sumH += current.getHeading();
                count++;
            }

            // Calculate and store the 1-second average
            Pose averagePose = new Pose(sumX / count, sumY / count, sumH / count);
            timeSamples.add(averagePose);

            // Progress Telemetry
            telemetry.addData("Status", "Collecting Sample %d/%d", i + 1, SAMPLE_COUNT);
            telemetry.addData("Current Avg Pose", "X: %.4f, Y: %.4f", averagePose.getX(), averagePose.getY());
            telemetry.update();
        }

        // Turn off motors
        // flywheelMotor.setPower(0);

        // --- PHASE 2: CALCULATION ---
        telemetry.addLine("Calculating Statistics...");
        telemetry.update();

        // Calculate Sigma X using multiple time deltas
        double sigmaX_1s = calculateSigmaForGap(1, true);   // true for X
        double sigmaX_4s = calculateSigmaForGap(4, true);
        double sigmaX_10s = calculateSigmaForGap(10, true);
        double sigmaX_Total = calculateSigmaForGap(SAMPLE_COUNT - 1, true);

        // Average the methods to get a final robust value for X
        double finalSigmaX = (sigmaX_1s + sigmaX_4s + sigmaX_10s + sigmaX_Total) / 4.0;

        // Calculate Sigma Y using multiple time deltas
        double sigmaY_1s = calculateSigmaForGap(1, false);  // false for Y
        double sigmaY_4s = calculateSigmaForGap(4, false);
        double sigmaY_10s = calculateSigmaForGap(10, false); // <--- Added this!
        double sigmaY_Total = calculateSigmaForGap(SAMPLE_COUNT - 1, false);

        // Average the methods to get a final robust value for Y
        double finalSigmaY = (sigmaY_1s + sigmaY_4s + sigmaY_10s + sigmaY_Total) / 4.0;

        // --- PHASE 3: RESULTS ---
        while (opModeIsActive()) {
            telemetry.addLine("### TUNING COMPLETE ###");
            telemetry.addLine("Copy these values into KalmanPinpointLocalizer:");
            telemetry.addLine("---------------------------------------------");
            telemetry.addData("Static Sigma X", "%.5f", finalSigmaX);
            telemetry.addData("Static Sigma Y", "%.5f", finalSigmaY);
            telemetry.addLine("---------------------------------------------");
            telemetry.addData("Raw X (10s gap)", "%.5f", sigmaX_10s);
            telemetry.addData("Raw Y (10s gap)", "%.5f", sigmaY_10s);
            telemetry.update();
        }
    }

    /**
     * Helper to calculate Sigma for X or Y axis based on a specific time gap.
     * @param strideSeconds The gap in seconds between samples.
     * @param isX True for X axis, False for Y axis.
     * @return The calculated sigma.
     */
    private double calculateSigmaForGap(int strideSeconds, boolean isX) {
        if (strideSeconds >= timeSamples.size()) return 0;

        double sumSigmas = 0;
        int comparisons = 0;

        // Denominator is sqrt(Delta Time)
        double sqrtTime = Math.sqrt(strideSeconds);

        // Slide through the array comparing samples 'stride' apart
        for (int i = 0; i < timeSamples.size() - strideSeconds; i++) {
            double val1 = isX ? timeSamples.get(i).getX() : timeSamples.get(i).getY();
            double val2 = isX ? timeSamples.get(i + strideSeconds).getX() : timeSamples.get(i + strideSeconds).getY();

            double diff = Math.abs(val2 - val1);

            // Sigma = Drift / sqrt(time)
            sumSigmas += (diff / sqrtTime);
            comparisons++;
        }

        return (comparisons > 0) ? (sumSigmas / comparisons) : 0;
    }
}