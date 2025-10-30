package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.consts.Alliance;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;

import java.io.IOException;

public class Shooter extends SubsystemBase {
    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    private final MotorEx turret;
    private final SimpleServo hood;
    private final IShooterCalculator shooterCalculator;
    private final Alliance alliance;
    private final Pose goalPose;
    /**
     * Flywheel velocity [ticks/second]
     */
    private double velocity;
    /**
     * Target hood angle in radians
     */
    private double hoodAngle;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker, IShooterCalculator shooterCalculator, Alliance alliance) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, "flywheel", MotorEx.GoBILDA.BARE);
        flywheel.setVeloCoefficients(1, 0, 0);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);

        turret = new MotorEx(hardwareMap, "turret", MotorEx.GoBILDA.RPM_312);
        turret.setPositionCoefficient(1);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        //TODO: Change 300 deg to correct max angle
        hood = new SimpleServo(hardwareMap, "hood", 0, 300, AngleUnit.DEGREES);

        this.shooterCalculator = shooterCalculator;

        this.alliance = alliance;

        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;
    }


    private void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    private void setVerticalAngle(double angleRad) {
        this.hoodAngle = angleRad;
    }


    @Override
    public void periodic() {
        ShootingSolution solution = shooterCalculator.getShootingSolution(poseTracker.getPose(), goalPose, poseTracker.getVelocity());

        setVerticalAngle(solution.getVerticalAngle());
        setVelocity(solution.getVelocity());

        flywheel.setVelocity(velocity, AngleUnit.RADIANS);
        hood.turnToAngle(this.hoodAngle, AngleUnit.RADIANS);

        if (!turret.atTargetPosition())
            turret.set(1);
        else
            turret.stopMotor();
    }
}
