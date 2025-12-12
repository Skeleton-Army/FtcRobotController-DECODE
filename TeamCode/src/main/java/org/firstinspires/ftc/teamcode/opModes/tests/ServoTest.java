package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;

@TeleOp
public class ServoTest extends OpMode {
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()) {
            servo.setPosition(ShooterConfig.HOOD_POSSIBLE_MAX);
        }

        if (gamepad1.squareWasPressed()) {
            servo.setPosition(ShooterConfig.HOOD_POSSIBLE_MIN);
        }
        telemetry.addData("Servo pos", servo.getPosition());
    }
}
