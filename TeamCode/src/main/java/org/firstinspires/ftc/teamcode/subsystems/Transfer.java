package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;
import com.seattlesolvers.solverslib.hardware.SensorRevTOFDistance;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.BooleanSupplier;

public class Transfer extends SubsystemBase {
    private static final int SENSOR_FAIL_THRESHOLD = 5;

    private final ServoEx kicker;
    private final ServoEx stopper;
    private final SensorRevColorV3 colorSensor;
    private final SensorRevTOFDistance sensorDistance;

    private boolean colorSensorDisabled = false;
    private boolean distanceSensorDisabled = false;
    private int colorSensorFailCount = 0;
    private int distanceSensorFailCount = 0;

    public Transfer(final HardwareMap hardwareMap) {
        kicker = new ServoEx(hardwareMap, KICKER_NAME);
        kicker.set(KICKER_MIN);

        stopper = new ServoEx(hardwareMap, STOPPER_NAME);
        stopper.set(STOPPER_STOP);

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
        stopper.set(STOPPER_STOP);
    }

    public void release() {
        stopper.set(STOPPER_RELEASE);
    }

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public boolean isArtifactDetected() {
        double distance = getDistance();
        if (distance == -1) return false;

        return distance <= DISTANCE_THRESHOLD_CM;
    }

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public double getDistance() {
        if (colorSensorDisabled) return -1;

        double dist = colorSensor.distance(DistanceUnit.CM);
        if (isSensorDisconnected(dist)) {
            if (++colorSensorFailCount >= SENSOR_FAIL_THRESHOLD) {
                RobotLog.addGlobalWarningMessage("TRANSFER SENSOR DISCONNECTED.");
                colorSensorDisabled = true;
            }
            return -1;
        }

        colorSensorFailCount = 0;
        return dist;
    }

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public double getDistanceIntake() {
        if (distanceSensorDisabled) return -1;

        double dist = sensorDistance.getDistance(DistanceUnit.CM);
        if (isSensorDisconnected(dist)) {
            if (++distanceSensorFailCount >= SENSOR_FAIL_THRESHOLD) {
                RobotLog.addGlobalWarningMessage("INTAKE SENSOR DISCONNECTED.");
                distanceSensorDisabled = true;
            }
            return -1;
        }

        distanceSensorFailCount = 0;
        return dist;
    }

    /* Do not call this function in a loop, as I2C calls reduce loop times greatly. */
    public boolean isArtifactInIntake()  {
        double distance = getDistanceIntake();
        if (distance == -1) return false;

        return distance <= DISTANCE_INTAKE_CM;
    }

    public BooleanSupplier threeArtifactsDetected(BooleanSupplier isCollecting, long thresholdMs) {
        return new BooleanSupplier() {
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
        };
    }

    private boolean isSensorDisconnected(double distanceCM) {
        return distanceCM >= 100 || distanceCM <= 0;
    }
}
