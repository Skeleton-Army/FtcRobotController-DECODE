package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.KickstandConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Kickstand extends SubsystemBase {
    private final ServoEx servo;

    public Kickstand(final HardwareMap hardwareMap) {
        servo = new ServoEx(hardwareMap, SERVO_NAME);
        drop();
    }

    public void raise() {
        servo.set(SERVO_MAX);
    }

    public void drop() {
        servo.set(SERVO_MIN);
    }
}
