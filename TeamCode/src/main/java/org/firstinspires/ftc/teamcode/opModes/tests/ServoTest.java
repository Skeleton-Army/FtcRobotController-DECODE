package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.config.TransferConfig;

@TeleOp
@Config
public class ServoTest extends OpMode {
    private Servo servo;

    public static String name = "hood";
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, name);
    }

    @Override
    public void loop() {

        switch (name) {
            case "stopper": {
                if (gamepad1.crossWasPressed()) {
                    servo.setPosition(TransferConfig.STOPPER_MAX);
                }

                if (gamepad1.squareWasPressed()) {
                    servo.setPosition(TransferConfig.STOPPER_MIN);
                }
            }

            case "hood":
                if (gamepad1.crossWasPressed()) {
                    servo.setPosition(ShooterConfig.HOOD_POSSIBLE_MAX);
                }

                if (gamepad1.squareWasPressed()) {
                    servo.setPosition(ShooterConfig.HOOD_POSSIBLE_MIN);
                }
        }

        if (gamepad1.crossWasPressed()) {
            servo.setPosition(TransferConfig.STOPPER_MAX);
        }

        if (gamepad1.squareWasPressed()) {
            servo.setPosition(TransferConfig.STOPPER_MIN);
        }

        if (gamepad1.dpadUpWasPressed()) {
            servo.setPosition(servo.getPosition() + 0.1);
        }

        if (gamepad1.dpadDownWasPressed()) {
            servo.setPosition(servo.getPosition() - 0.1);
        }

        telemetry.addData("Servo pos", servo.getPosition());
    }
}
