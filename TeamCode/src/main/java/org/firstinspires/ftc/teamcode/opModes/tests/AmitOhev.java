package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Amit Ohev", group = "Tests")
public class AmitOhev extends OpMode {
    private DcMotorEx motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        // Optionally reset encoder if you want
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Get motor velocity in ticks per second
        double motorTPS = motor.getVelocity();

        // Example: NeveRest 60 motor has 1680 ticks per revolution
        int ticksPerRev = 28;

        // Convert to RPM
        double motorRPM = (motorTPS * 60.0) / ticksPerRev;

        telemetry.addData("Velocity (ticks/sec)", motorTPS);
        telemetry.addData("Motor RPM", motorRPM);
        telemetry.update();
    }
}

// iicyify