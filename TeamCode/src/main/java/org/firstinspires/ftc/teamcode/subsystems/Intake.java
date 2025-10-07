package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;

    public Intake(final HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, "intake");
    }

    public void set(double output) {
        intake.set(output);
    }
}
