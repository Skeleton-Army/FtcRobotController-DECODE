package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Zipi extends OpMode {

    DcMotorEx spinGreen;
    DcMotorEx spinNotGreen;
    boolean isRunningG = false;
    boolean isRunningNG = false;
    @Override
    public void init() {
        spinGreen = hardwareMap.get(DcMotorEx.class, "leftBack");
        spinNotGreen = hardwareMap.get(DcMotorEx.class, "leftFront");
    }

    @Override
    public void loop() {
        spinGreen.setPower(.35);
        spinNotGreen.setPower(1);
    }
}
