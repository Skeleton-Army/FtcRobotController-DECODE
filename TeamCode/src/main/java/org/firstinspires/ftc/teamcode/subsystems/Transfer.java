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

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public boolean isArtifactDetected() {
        return getDistance() <= DISTANCE_THRESHOLD_CM;
    }

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public double getDistance() {
        return colorSensor.distance(DistanceUnit.CM);
    }

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public double getDistanceIntake() {
        return sensorDistance.getDistance(DistanceUnit.CM);
    }

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public boolean isArtifactInIntake()  {
        return getDistanceIntake() <= DISTANCE_INTAKE_CM;
    }

    public Trigger threeArtifactsDetected(BooleanSupplier isCollecting, long thresholdMs) {
        return new Trigger(new BooleanSupplier() {
            private long localDetectedTime = 0;
            private long lastEmptyCheckTime = 0;
            private final long EMPTY_POLL_COOLDOWN = 100;

            @Override
            public boolean getAsBoolean() {
                if (!isCollecting.getAsBoolean()) {
                    localDetectedTime = 0;
                    return false;
                }

                long currentTime = System.currentTimeMillis();

                if (localDetectedTime == 0) {
                    // Only poll I2C if the cooldown has expired
                    if (currentTime - lastEmptyCheckTime >= EMPTY_POLL_COOLDOWN) {
                        lastEmptyCheckTime = currentTime;

                        if (isArtifactInIntake() && isArtifactDetected()) {
                            localDetectedTime = currentTime;
                        }
                    }
                    return false;
                }

                long elapsed = currentTime - localDetectedTime;

                if (elapsed >= thresholdMs) {
                    if (isArtifactInIntake() && isArtifactDetected()) {
                        return true;
                    } else {
                        localDetectedTime = 0;
                        lastEmptyCheckTime = currentTime;
                        return false;
                    }
                }

                return false;
            }
        });
    }
}
