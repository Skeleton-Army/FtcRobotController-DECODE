package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;

import java.io.IOException;

@Config
public class Shooter extends SubsystemBase {
    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    private final Motor turret;
    private final ServoEx hood;

    private final IShooterCalculator shooterCalculator;
    private final Alliance alliance;
    private final Pose goalPose;
    /**
     * Flywheel velocity [ticks/second]
     */
    private double velocity;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker, IShooterCalculator shooterCalculator, Alliance alliance) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, FLYWHEEL_NAME, FLYWHEEL_MOTOR);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);

        turret = new Motor(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setPositionCoefficient(TURRET_KP);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));

        hood = new ServoEx(hardwareMap, HOOD_NAME, HOOD_MIN, HOOD_MAX);

        this.shooterCalculator = shooterCalculator;
        this.alliance = alliance;
        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;
    }

    @Override
    public void periodic() {
        ShootingSolution solution = shooterCalculator.getShootingSolution(poseTracker.getPose(), goalPose, poseTracker.getVelocity());

        setVerticalAngle(solution.getVerticalAngle());
        setVelocity(solution.getVelocity());

        flywheel.setVelocity(velocity, AngleUnit.RADIANS);
        turret.set(1);
    }

    public void updateVerticalAngle() {
        // TODO: Do calculations and set vertical angle to GOAL
    }

    private void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    private void setVerticalAngle(double angleRad) {
        hood.set(angleRad);
    }

    private void setHorizontalAngle(double targetAngleRad) {
        double wrapped = IShooterCalculator.wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX);
        turret.setTargetDistance(wrapped);
    }
}
