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
    public static double targetRPM = 4000;
    public static double targetPower = 1;
    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 275.00198;
    public static double kV = 1.130;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = new MotorEx(hardwareMap, "flywheel", Motor.GoBILDA.BARE);
        handlePID();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            isPID = !isPID;
            handlePID();
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (isPID) targetRPM = MathUtils.clamp(targetRPM + 100, 0, 6000);
            else targetPower = MathUtils.clamp(targetPower + 0.1, 0, 1);
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (isPID) targetRPM = MathUtils.clamp(targetRPM - 100, 0, 6000);
            else targetPower = MathUtils.clamp(targetPower - 0.1, 0, 1);
        }

        if (isPID) {
            double targetTPS = (targetRPM * motor.getCPR()) / 60.0;
            motor.setVelocity(targetTPS);

            telemetry.addData("Target", targetRPM);
        }
        else {
            motor.set(targetPower);

            telemetry.addData("Target", targetPower);
        }

        // Get motor velocity in ticks per second
        double motorTPS = motor.getCorrectedVelocity();

        // Convert to RPM
        double motorRPM = -(motorTPS * 60.0) / motor.getCPR();

        telemetry.addData("Velocity (ticks/sec)", motorTPS);
        telemetry.addData("Motor RPM", motorRPM);
        telemetry.addData("Current (Amps)", motor.motorEx.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("PID On?", isPID);

        telemetry.update();
    }

    private void handlePID() {
        if (isPID) {
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setVeloCoefficients(kP, kI, kD);
            motor.setFeedforwardCoefficients(kS, kV);
        }
        else {
            motor.setRunMode(Motor.RunMode.RawPower);
        }
    }
}
