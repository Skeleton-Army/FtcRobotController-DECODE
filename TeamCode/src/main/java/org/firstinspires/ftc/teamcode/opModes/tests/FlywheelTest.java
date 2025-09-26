package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Flywheel Test", group = "Tests")
public class FlywheelTest extends OpMode {
    private MotorEx motor;

    private boolean isPID = true;
    public static double target;
    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 0.1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = new MotorEx(hardwareMap, "motor", Motor.GoBILDA.BARE);
        handlePID();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            isPID = !isPID;
            handlePID();
        }

        if (gamepad1.dpadUpWasPressed()) {
            target = MathUtils.clamp(target + (isPID ? 100 : 0.1), 0, (isPID ? 6000 : 1));
        }

        if (gamepad1.dpadDownWasPressed()) {
            target = MathUtils.clamp(target - (isPID ? 100 : 0.1), 0, (isPID ? 6000 : 1));
        }

        if (isPID) {
            double targetTPS = (target * motor.getCPR()) / 60.0;

            double maxTPS = motor.getCPR() * motor.getMaxRPM() / 60;
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Max RPM", motor.getMaxRPM());
            telemetry.addData("Max TPS", maxTPS);
            telemetry.addData("Set", targetTPS / maxTPS);

            motor.setVelocity(targetTPS);
        }
        else {
            motor.set(target);
        }

        // Get motor velocity in ticks per second
        double motorTPS = motor.getVelocity();

        // Convert to RPM
        double motorRPM = (motorTPS * 60.0) / motor.getCPR();

        telemetry.addData("Velocity (ticks/sec)", motorTPS);
        telemetry.addData("Motor RPM", motorRPM);
        telemetry.addData("Current (Amps)", motor.motorEx.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("PID On?", isPID);
        telemetry.addData("Target", target);

        telemetry.update();
    }

    private void handlePID() {
        if (isPID) {
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setVeloCoefficients(kP, kI, kD);
            target = 4000;
        }
        else {
            motor.setRunMode(Motor.RunMode.RawPower);
            target = 1;
        }
    }
}
