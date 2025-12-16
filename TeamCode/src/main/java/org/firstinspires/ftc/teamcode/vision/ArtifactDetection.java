package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    MultipleTelemetry tolematry;
    Follower follower;
    // Hc: Height of the Limelight lens from the floor (e.g., 0.35m or ~13.78 inches)


    @Override
    public void init() {
        tolematry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(ArtifactTrackingConfig.PIPELINE_INDEX);
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
                    robotPose.getHeading(),
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
