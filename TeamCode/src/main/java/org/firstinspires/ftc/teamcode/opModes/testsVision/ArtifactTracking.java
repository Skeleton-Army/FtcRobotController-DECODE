package org.firstinspires.ftc.teamcode.opModes.testsVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArtifactTracking extends OpMode {

    double LLhight = 9.7;

    private Limelight3A limelight;
    LLResult llResult;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        // 0: apriltag detection
        // 1: artifact detection
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        llResult = limelight.getLatestResult();
        LLStatus llStatus = limelight.getStatus();

        if (llResult.isValid() && llResult != null) {

            double ty = llResult.getTy();
            double dx = LLhight * Math.tan(Math.toRadians(ty+90));
            double tx = Math.toRadians(llResult.getTx());
            double dy = Math.tan(tx)/dx;
            telemetry.addData("Artifact X", dx);
            telemetry.addData("Artifact Y", dy);
            telemetry.addLine("Data is available!");
        }

        if (gamepad1.a) {
            limelight.reloadPipeline();
        }

        telemetry.addData("pipeline index", llStatus.getPipelineIndex());
        telemetry.addData("Cpu", llStatus.getCpu());
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}

