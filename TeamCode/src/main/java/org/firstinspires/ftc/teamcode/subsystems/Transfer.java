package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer extends SubsystemBase {
    private final CRServoEx transfer;
    private final ServoEx kicker;
    private final SensorRevColorV3 colorSensor;

    public Transfer(final HardwareMap hardwareMap) {
        transfer = new CRServoEx(hardwareMap, TRANSFER_NAME);
        toggleTransfer(false);

        kicker = new ServoEx(hardwareMap, KICKER_NAME);
        kicker.set(0);

        colorSensor = new SensorRevColorV3(hardwareMap, SENSOR_NAME);
    }

    public void toggleTransfer(boolean isOn) {
        transfer.set(isOn ? TRANSFER_POWER : 0);
    }

    public void toggleTransfer(boolean isOn, boolean reversed) {
        transfer.set(isOn ? (reversed ? -1 : 0) * TRANSFER_REVERSE_POWER : 0);
    }

    public Command kick() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setKickerPosition(true)),
                new WaitCommand(KICK_TIME),
                new InstantCommand(() -> setKickerPosition(false))
        );
    }

    public void setKickerPosition(boolean isUp) {
        kicker.set(isUp ? KICKER_MAX : KICKER_MIN);
    }

    public boolean isArtifactDetected() {
        return getDistance() <= DISTANCE_THRESHOLD_CM;
    }

    public double getDistance() {
        return colorSensor.distance(DistanceUnit.CM);
    }
}
