package org.firstinspires.ftc.teamcode.opModes.testsVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class ApriltagDetection extends OpMode {
    Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // pipeline for apriltag!
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles orientation = result.getBotpose().getOrientation();


            telemetry.addData("AprilTag x", botpose.x);
            telemetry.addData("AprilTag y", botpose.y);
            telemetry.addData("AprilTag z", botpose.z);
            telemetry.addData("AprilTag heading", orientation.getYaw(AngleUnit.DEGREES));
        }

        telemetry.update();
    }
}
