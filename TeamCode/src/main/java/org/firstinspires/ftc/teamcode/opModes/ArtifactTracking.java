package org.firstinspires.ftc.teamcode.opModes;

import android.media.AudioMixerAttributes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;

@TeleOp
public class ArtifactTracking extends OpMode {

    double LLhight = 9.7;

    private Limelight3A limelight3A;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double ty = llResult.getTy();
            double dx = LLhight * Math.tan(Math.toRadians(ty+90));
            double tx = Math.toRadians(llResult.getTx());
            double dy = Math.tan(tx)/dx;
            telemetry.addData("Artifact X", dx);
            telemetry.addData("Artifact Y", dy);
            telemetry.update();
        }
    }
}

