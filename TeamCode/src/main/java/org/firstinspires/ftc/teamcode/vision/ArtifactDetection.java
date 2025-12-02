package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    MultipleTelemetry tolematry;
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
        LLResult llResult = limelight.getLatestResult();
                tolematry.addData("Tx: ",llResult.getTx());
                tolematry.addData("Ty: ", llResult.getTy());
                tolematry.addData("Ta: ", llResult.getTa());
                tolematry.addLine("python Output: ");
                for (double pyOut : llResult.getPythonOutput()) {
                    tolematry.addLine(String.valueOf(pyOut));
                }

        tolematry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
