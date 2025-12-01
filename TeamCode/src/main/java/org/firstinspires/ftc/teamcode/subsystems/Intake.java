package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;

    public Intake(final HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, "intake");
    }

    public double getRPM() {
        double motorTPS = intake.getCorrectedVelocity();
        return (motorTPS * 60.0) / intake.getCPR();
    }

    public void collect() {
        intake.set(-0.6);
    }

    public void release() {
        intake.set(1);
    }

    public void stop() {
        intake.set(0);
    }
}
