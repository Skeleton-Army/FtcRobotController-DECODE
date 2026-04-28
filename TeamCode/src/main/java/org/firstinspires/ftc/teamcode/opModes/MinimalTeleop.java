package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="PushBot TeleOp")
public class MinimalTeleop extends OpMode {

    DcMotorEx leftBack;
    DcMotorEx leftFront;
    DcMotorEx rightBack;
    DcMotorEx rightFront;
    IMU imu;

    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
            drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        leftFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        rightFront.setPower(maxSpeed * (frontRightPower / maxPower));
        leftBack.setPower(maxSpeed * (backLeftPower / maxPower));
        rightBack.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
