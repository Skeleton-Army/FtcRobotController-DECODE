package org.firstinspires.ftc.teamcode.opModes.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "Kalman Constant Tuner", group = "Tuning")
public class KalmanTunerOpMode extends LinearOpMode {

    // Note: Initialize your specific Pinpoint and Fusion localizers here
    // private PinpointLocalizer pinpoint;
    // private FusionLocalizer fusionLocalizer;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Reroute telemetry to FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Mapping
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // pinpoint = new PinpointLocalizer(hardwareMap);
        // fusionLocalizer = new FusionLocalizer(pinpoint, ...);

        waitForStart();

        while (opModeIsActive()) {
            // fusionLocalizer.update();
            // Pose pinpointPose = fusionLocalizer.getPose();
            // Pose pinpointVelocity = fusionLocalizer.getVelocity();

            // Mock variables for compilation - replace with actual localizer calls above
            Pose pinpointPose = new Pose(0,0,0);
            Pose pinpointVelocity = new Pose(0,0,0);

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // --- 1. PERFECT TIMESTAMP CALCULATION ---
                // Get the exact nanoTime the packet arrived at the Control Hub
                long arrivalTimeNanos = result.getControlHubTimeStampNanos();

                // Sum the internal camera processing times (in milliseconds)
                double totalLatencyMs = result.getCaptureLatency() + result.getTargetingLatency();

                // Convert latency to nanoseconds and subtract from arrival time
                long imageCaptureTimeNanos = arrivalTimeNanos - (long)(totalLatencyMs * 1_000_000.0);

                // --- 2. POSE EXTRACTION ---
                Pose limelightPose = new Pose(
                        result.getBotpose().getPosition().x,
                        result.getBotpose().getPosition().y,
                        Math.toRadians(result.getBotpose().getOrientation().getYaw())
                );

                // --- 3. TRUE 3D DISTANCE CALCULATION ---
                double distanceToTag = 0.0;
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    double totalDistance = 0.0;
                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        double x = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH).x;
                        double y = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH).y;
                        double z = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH).z;

                        // Euclidean hypotenuse
                        totalDistance += Math.sqrt((x * x) + (y * y) + (z * z));
                    }
                    distanceToTag = totalDistance / fiducials.size();
                }

                // --- 4. FEED THE KALMAN FILTER ---
                // Uncomment this when running for real:
                // fusionLocalizer.addVisionMeasurement(limelightPose, imageCaptureTimeNanos, distanceToTag);

                // --- 5. TELEMETRY FOR TUNING ---
                double errorX = Math.abs(pinpointPose.getX() - limelightPose.getX());
                double errorY = Math.abs(pinpointPose.getY() - limelightPose.getY());
                double errorHeading = Math.abs(pinpointPose.getHeading() - limelightPose.getHeading());

                double translationalSpeed = Math.hypot(pinpointVelocity.getX(), pinpointVelocity.getY());
                double rotationalSpeed = Math.abs(pinpointVelocity.getHeading());

                telemetry.addData("1_Error_X", errorX);
                telemetry.addData("1_Error_Y", errorY);
                telemetry.addData("1_Error_Heading", errorHeading);

                telemetry.addData("2_Translational_Speed", translationalSpeed);
                telemetry.addData("2_Rotational_Speed", rotationalSpeed);
                telemetry.addData("2_Distance_To_Tag", distanceToTag);

                telemetry.addData("3_Total_Latency_ms", totalLatencyMs);

                telemetry.update();
            }
        }
        limelight.stop();
    }
}
