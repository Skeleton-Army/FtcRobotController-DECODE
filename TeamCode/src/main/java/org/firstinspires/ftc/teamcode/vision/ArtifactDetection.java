package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static java.lang.Math.*;

@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    MultipleTelemetry tolematry;
    Follower follower;
    // Hc: Height of the Limelight lens from the floor (e.g., 0.35m or ~13.78 inches)
    private final double CAMERA_HEIGHT_M = 0.35;

    // alpha: Camera mounting pitch angle relative to the floor (degrees)
    // Positive means tilted up, negative means tilted down
    private final double CAMERA_PITCH_DEG = 10.0;

    // Ht: Known height of the artifact's center from the floor (e.g., 2.0m)
    private final double TARGET_HEIGHT_M = 2.0;

    // Camera Mounting Offset (Relative to Robot Center in Meters)
    // FTC uses X=side-to-side (Right is positive), Y=forward/backward (Forward is positive)
    private final double CAM_OFFSET_X_M = 0.0;  // Center-mounted (X)
    private final double CAM_OFFSET_Y_M = 0.25; // 25 cm (0.25m) forward (Y)
    private final double CAM_YAW_DEG = -5.0;    // 5 degrees to the right (negative yaw)

    // Pre-calculate fixed angle constants
    private final double CAM_PITCH_RAD = toRadians(CAMERA_PITCH_DEG);
    private final double CAM_YAW_RAD = toRadians(CAM_YAW_DEG);

    @Override
    public void init() {
        tolematry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Goofy.pipelineIndex);
        // 0: apriltag detection
        // 1: artifact detection
        // 2: test pipeline
        limelight.start();



    }

    @Override
    public void loop() {
        // Update odometry first
        if (follower != null) {
            follower.update();
        } else {
            telemetry.addData("Error", "Follower (Pedro Pathing) is not initialized!");
            return;
        }
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null) {
            tolematry.addData("Tx: ", llResult.getTx());
            tolematry.addData("Ty: ", llResult.getTy());
            tolematry.addData("Ta: ", llResult.getTa());
            tolematry.addLine("python Output: ");
            for (double pyOut : llResult.getPythonOutput()) {
                tolematry.addLine(String.valueOf(pyOut));
            }
            Pose robotPose = follower.getPose();

            double[] artifactPos = artifactFieldPos(
                    robotPose.getX(),
                    robotPose.getY(),
                    robotPose.getHeading(), // Ensure this is in Radians!
                    llResult.getTx(),
                    llResult.getTy()
            );
        }
        tolematry.update();
    }
    private double[] artifactFieldPos(double robotX, double robotY, double robotHeadingRad,
                                              double tx_deg, double ty_deg) {
        return new double[0];//temp value to stop error for the time being
    }


    @Override
    public void stop() {
        limelight.stop();
    }
}
