package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.TimerEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;
    private boolean isCollecting = false;
    private final TimerEx runtime = new TimerEx();

    private double power;

    public Intake(final HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, INTAKE_NAME, INTAKE_MOTOR);
        runtime.start();
        runtime.pause();

        power = INTAKE_POWER;
    }

    public double getRPM() {
        double motorTPS = -intake.getCorrectedVelocity();
        return (motorTPS * 60.0) / intake.getCPR();
    }

    public void collect() {
        if (!isCollecting) {
            runtime.restart();
        }
        intake.set(-power);
        isCollecting = true;
    }

    public void setSlowMode(boolean enabled) {
        power = enabled ? SLOW_INTAKE_POWER : INTAKE_POWER;
    }

    public void release() {
        intake.set(INTAKE_POWER);
        isCollecting = false;
    }

    public void stop() {
        runtime.pause();
        intake.set(0);
        isCollecting = false;
    }

    public boolean isCollecting() {
        return isCollecting;
    }

    public boolean isStalled() {
        return isCollecting &&
                runtime.getElapsed() > STALL_STARTUP_DELAY &&
                getRPM() < STALL_THRESHOLD;
    }
}
