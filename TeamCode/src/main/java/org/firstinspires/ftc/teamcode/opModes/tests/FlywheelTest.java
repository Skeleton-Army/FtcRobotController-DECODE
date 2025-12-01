package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Flywheel Test", group = "Tests")
public class FlywheelTest extends OpMode {
    private MotorEx motor;

    private boolean isPID = true;
    public static double targetPower = 1;
    private ServoEx kicker;

    private GamepadEx gamepadEx1;
    TimerEx timerEx;
    private double recoveryTime;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);

        motor = new MotorEx(hardwareMap, "flywheel", Motor.GoBILDA.BARE);
        motor.setInverted(true);
        handlePID();

        kicker = new ServoEx(hardwareMap, KICKER_NAME);
        kicker.set(0);

        timerEx = new TimerEx(TimeUnit.SECONDS);
        /*gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new InstantCommand(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> kicker.set(KICKER_MAX)),
                                new InstantCommand(() -> timerEx.restart()),
                                new InstantCommand(() -> timerEx.start()),
                                new WaitCommand(KICK_TIME),
                                new InstantCommand(() -> kicker.set(KICKER_MIN))
                        )
                )));*/
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            isPID = !isPID;
            handlePID();
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (isPID) FLYWHEEL_TARGET = MathUtils.clamp(FLYWHEEL_TARGET + 100, 0, 6000);
            else targetPower = MathUtils.clamp(targetPower + 0.1, 0, 1);
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (isPID) FLYWHEEL_TARGET = MathUtils.clamp(FLYWHEEL_TARGET - 100, 0, 6000);
            else targetPower = MathUtils.clamp(targetPower - 0.1, 0, 1);
        }
        if (gamepad1.triangleWasPressed()) {
            new InstantCommand(() -> CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> kicker.set(KICKER_MAX)),
                            new InstantCommand(() -> timerEx.restart()),
                            new InstantCommand(() -> timerEx.start()),
                            new WaitCommand(KICK_TIME),
                            new InstantCommand(() -> kicker.set(KICKER_MIN))
                    )
            ));

        }

        if (isPID) {
            double targetTPS = (FLYWHEEL_TARGET * motor.getCPR()) / 60.0;
            motor.setVelocity(targetTPS);
            telemetry.addData("Target", FLYWHEEL_TARGET);
        }
        else {
            motor.set(targetPower);

            telemetry.addData("Target", targetPower);
        }
        // Get motor velocity in ticks per second
        double motorTPS = motor.getCorrectedVelocity();

        // Convert to RPM
        double motorRPM = (motorTPS * 60.0) / motor.getCPR();

        if (FLYWHEEL_TARGET - motorRPM <= RPM_REACHED_THRESHOLD) {
            recoveryTime = timerEx.getElapsed();
            timerEx.pause();
        }

        telemetry.addData("Velocity (ticks/sec)", motorTPS);
        telemetry.addData("Motor RPM", motorRPM);
        telemetry.addData("recovery time (sec)",recoveryTime);
        telemetry.addData("Current (Amps)", motor.motorEx.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("PID On?", isPID);

        telemetry.update();
    }

    private void handlePID() {
        if (isPID) {
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
            motor.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV);
        }
        else {
            motor.setRunMode(Motor.RunMode.RawPower);
        }
    }
}
