package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;

    public Intake(final HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, INTAKE_NAME);
    }

    public double getRPM() {
        double motorTPS = intake.getCorrectedVelocity();
        return (motorTPS * 60.0) / intake.getCPR();
    }

    public void collect() {
        intake.set(-INTAKE_POWER);
    }

    public void transfer() {
        intake.set(-INTAKE_TRANSFER_POWER);
    }

    public void release() {
        intake.set(INTAKE_POWER);
    }

    public void stop() {
        intake.set(0);
    }
}
