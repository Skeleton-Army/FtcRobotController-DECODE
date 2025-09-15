package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter extends SubsystemBase {
    private final PoseTracker poseTracker;

    private final Motor flywheel;
    private final Motor turret;
    private final SimpleServo hood;

    private double power;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker) {
        this.poseTracker = poseTracker;

        flywheel = new Motor(hardwareMap, "flywheel", Motor.GoBILDA.BARE);
        flywheel.setVeloCoefficients(1, 0, 0);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);

        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);
        turret.setPositionCoefficient(1);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        hood = new SimpleServo(hardwareMap, "hood", 0, Math.PI / 2, AngleUnit.RADIANS);
    }

    private void setPower(double power) {
        this.power = power;
    }

    private void setVerticalAngle(double angleRad) {
        hood.turnToAngle(angleRad);
    }

    private void setHorizontalAngle(int ticks) {
        turret.setTargetPosition(ticks);
    }

    public void updateHorizontalAngle() {
        // TODO: Do calculations and set horizontal angle to face GOAL
    }

    public void updateVerticalAngle() {
        // TODO: Do calculations and set vertical angle to GOAL
    }

    @Override
    public void periodic() {
        flywheel.set(power);

        if (!turret.atTargetPosition())
            turret.set(1);
        else
            turret.stopMotor();
    }
}
