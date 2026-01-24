package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;
import com.seattlesolvers.solverslib.hardware.SensorRevTOFDistance;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.BooleanSupplier;

public class Transfer extends SubsystemBase {
    private final ServoEx kicker;
    private final ServoEx stopper;
    private final SensorRevColorV3 colorSensor;
    private final SensorRevTOFDistance sensorDistance;

    public Transfer(final HardwareMap hardwareMap) {
        kicker = new ServoEx(hardwareMap, KICKER_NAME);
        kicker.set(KICKER_MIN);

        stopper = new ServoEx(hardwareMap, STOPPER_NAME);
        stopper.set(STOPPER_MAX);

        colorSensor = new SensorRevColorV3(hardwareMap, SENSOR_NAME);
        sensorDistance = new SensorRevTOFDistance(hardwareMap, DISTANCE_SENSOR_NAME);
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

    public void block() {
        stopper.set(STOPPER_MAX);
    }

    public void release() {
        stopper.set(STOPPER_MIN);
    }

    public boolean isArtifactDetected() {
        return getDistance() <= DISTANCE_THRESHOLD_CM;
    }

    public double getDistance() {
        return colorSensor.distance(DistanceUnit.CM);
    }

    public double getDistanceIntake() {
        return sensorDistance.getDistance(DistanceUnit.CM);
    }

    public boolean isArtifactInIntake()  {
        return getDistanceIntake() <= DISTANCE_INTAKE_CM;
    }

    public Trigger threeArtifactsDetected(long thresholdMs) {
        return new Trigger(new BooleanSupplier() {
            private long localDetectedTime = 0;

            @Override
            public boolean getAsBoolean() {
                boolean detected = isArtifactInIntake() && isArtifactDetected();

                if (localDetectedTime == 0) {
                    if (detected) {
                        localDetectedTime = System.currentTimeMillis();
                    }
                    return false;
                }

                long elapsed = System.currentTimeMillis() - localDetectedTime;

                if (elapsed >= thresholdMs) {
                    if (detected) {
                        return true;
                    } else {
                        localDetectedTime = 0;
                        return false;
                    }
                }

                return false;
            }
        });
    }
}
