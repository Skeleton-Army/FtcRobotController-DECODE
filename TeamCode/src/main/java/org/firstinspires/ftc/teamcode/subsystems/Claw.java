package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Claw extends SubsystemBase {

    private ServoEx clawPivot;
    private ServoEx clawHand;
    private boolean isClawOpen = false;

    private final double MIN_CLAW = 0.1;
    private final double MAX_CLAW = 0.6;

    public Claw(final HardwareMap hardwareMap) {
        clawPivot = new ServoEx(hardwareMap, "servo2");
        clawHand = new ServoEx(hardwareMap,  "servo3");
        clawHand.set(MIN_CLAW);
    }

    private void ClawOpen() {
        clawHand.set(MAX_CLAW);
        isClawOpen = true;
    }

    private void ClawClose() {
        clawHand.set(MIN_CLAW);
        isClawOpen = false;
    }

    public void useClaw() {
        if (isClawOpen) {
            ClawClose();
        } else {
            ClawOpen();
        }
    }





}
