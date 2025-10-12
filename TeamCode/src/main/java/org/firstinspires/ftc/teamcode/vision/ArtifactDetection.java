package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        // 0: apriltag detection
        // 1: artifact detection
        limelight.start();

    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
            telemetry.addData("tx: ", llResult.getTx());
            telemetry.addData("ty: ", llResult.getTy());
            telemetry.addData("ta: ", llResult.getTa());


            for( double pythonResult: llResult.getPythonOutput()) {
                telemetry.addData("python output : ", pythonResult);
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
