package org.firstinspires.ftc.teamcode.opModes.tests;


import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Kalman Constant Tuner", group = "Tuning")
public class KalmanTunerOpMode extends OpMode {

    // Note: Initialize your specific Pinpoint and Fusion localizers here
    // private PinpointLocalizer pinpoint;
    // private FusionLocalizer fusionLocalizer;
    private Limelight3A limelight;

    public static final double WIDTH = 14.96;
    public static final double HEIGHT = 16.53;
    public static final double X_OFFSET = WIDTH / 2.0;
    public static final double Y_OFFSET = HEIGHT / 2.0;
    Follower follower;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setPose(new Pose(X_OFFSET, Y_OFFSET)); // sets the start pose in the left-bottom corner
        follower.setMaxPower(1);

        // Hardware Mapping
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    @Override
    public void loop() {
        // fusionLocalizer.update();
        // Pose pinpointPose = fusionLocalizer.getPose();
        // Pose pinpointVelocity = fusionLocalizer.getVelocity();

        follower.update();
        // Mock variables for compilation - replace with actual localizer calls above
        Pose pinpointPose = follower.getPose();
        Vector pinpointVelVector = follower.poseTracker.getVelocity();

        limelight.updateRobotOrientation(follower.getHeading());
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            Pose3D mt2Pose = result.getBotpose_MT2();

            // --- 1. PERFECT TIMESTAMP CALCULATION ---
            // Get the exact nanoTime the packet arrived at the Control Hub
            long arrivalTimeNanos = result.getControlHubTimeStampNanos();

            // Sum the internal camera processing times (in milliseconds)
            double totalLatencyMs = result.getCaptureLatency() + result.getTargetingLatency();

            // Convert latency to nanoseconds and subtract from arrival time
            long imageCaptureTimeNanos = arrivalTimeNanos - (long)(totalLatencyMs * 1_000_000.0);

            // --- 2. POSE EXTRACTION ---
            Pose limelightPose = new Pose(
                    mt2Pose.getPosition().x,
                    mt2Pose.getPosition().y,
                    Math.toRadians(mt2Pose.getOrientation().getYaw())
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

            double translationalSpeed = Math.hypot(pinpointVelVector.getXComponent(), pinpointVelVector.getYComponent());
            double rotationalSpeed = Math.abs(follower.getAngularVelocity());

            telemetry.addData("1_Error_X", errorX);
            telemetry.addData("1_Error_Y", errorY);
            telemetry.addData("1_Error_Heading", errorHeading);

            telemetry.addData("2_Translational_Speed", translationalSpeed);
            telemetry.addData("2_Rotational_Speed", rotationalSpeed);
            telemetry.addData("2_Distance_To_Tag", distanceToTag);

            telemetry.addData("3_Total_Latency_ms", totalLatencyMs);

            telemetry.addData("stdev mt2", result.getStddevMt2());

            telemetry.update();
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
