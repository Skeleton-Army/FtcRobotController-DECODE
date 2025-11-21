package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "test");
    }

    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()) {
            servo.setPosition(1);
        }

        if (gamepad1.squareWasPressed()) {
            servo.setPosition(0);
        }
    }
}
