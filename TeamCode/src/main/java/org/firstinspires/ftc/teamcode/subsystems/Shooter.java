package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter extends SubsystemBase {
    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    private final MotorEx turret;
    private final SimpleServo hood;

    private double velocity;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, "flywheel", MotorEx.GoBILDA.BARE);
        flywheel.setVeloCoefficients(1, 0, 0);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);

        turret = new MotorEx(hardwareMap, "turret", MotorEx.GoBILDA.RPM_312);
        turret.setPositionCoefficient(1);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        hood = new SimpleServo(hardwareMap, "hood", 0, Math.PI / 2, AngleUnit.RADIANS);
    }

    private void setVelocity(double velocity) {
        this.velocity = velocity;
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
        flywheel.setVelocity(velocity, AngleUnit.RADIANS);

        if (!turret.atTargetPosition())
            turret.set(1);
        else
            turret.stopMotor();
    }
}
