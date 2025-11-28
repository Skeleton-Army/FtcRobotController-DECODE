package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorEx;

import java.util.concurrent.TimeUnit;

public class Shooter extends SubsystemBase {
    public ShootingSolution solution;

    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    private final ModifiedMotorEx turret;
    private final ServoEx hood;
    private final CRServoEx transfer;
    private final ServoEx kicker;

    private final IShooterCalculator shooterCalculator;
    private final Alliance alliance;
    private final Pose goalPose;

    private double targetTPS;

    private final TimerEx timerEx;
    public boolean calculatedRecovery = false;
    private double recoveryTime; // in seconds
    public double wrapped;
    public double shotHoodAngle; // in degrees
    public double shotTurretAngle; // in degrees
    public double shotFlywheelRPM;
    public double shotGoalDistance; // in meters
    final double inchesToMeters = 39.37;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker, IShooterCalculator shooterCalculator, Alliance alliance) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, FLYWHEEL_NAME, FLYWHEEL_MOTOR);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);
        flywheel.setInverted(FLYWHEEL_INVERTED);

        turret = new ModifiedMotorEx(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setPositionCoefficients(TURRET_KP, TURRET_KI, TURRET_KD, TURRET_KF);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));
        setHorizontalAngle(0);

        hood = new ServoEx(hardwareMap, HOOD_NAME);
        setVerticalAngle(0);

        transfer = new CRServoEx(hardwareMap, TRANSFER_NAME);
        toggleTransfer(false);

        kicker = new ServoEx(hardwareMap, KICKER_NAME);
        kicker.set(0);

        //setRPM(0);
        //spinUp();

        this.shooterCalculator = shooterCalculator;
        this.alliance = alliance;
        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;

        timerEx = new TimerEx(TimeUnit.SECONDS);
    }

    @Override
    public void periodic() {
        solution = shooterCalculator.getShootingSolution(poseTracker.getPose(), goalPose, poseTracker.getVelocity(), poseTracker.getAngularVelocity());

        setHorizontalAngle(solution.getHorizontalAngle());
        setVerticalAngle(solution.getVerticalAngle());
//        setVelocity(solution.getVelocity());

        //flywheel.setVelocity(MathUtils.clamp(solution.getVelocity(), 0,5000));
        setRPM(MathUtils.clamp(solution.getVelocity(), 0,5000));
        flywheel.setVelocity(targetTPS);

        calculateRecovery();

        turret.set(1);
    }

    public void spinUp() {
        setRPM(FLYWHEEL_TARGET);
    }

    public void spinDown() {
        setRPM(0);
    }

    public void toggleTransfer(boolean isOn) {
        transfer.set(isOn ? TRANSFER_POWER : 0);
    }

    public void kick() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> kicker.set(KICKER_MAX)),
                        new WaitCommand(KICK_TIME),
                        new InstantCommand(() -> kicker.set(KICKER_MIN))
                )
        );
    }

    public double getRPM() {
        double motorTPS = flywheel.getCorrectedVelocity();
        return (motorTPS * 60.0) / flywheel.getCPR();
    }

    public double getTargetRPM() {
        return (targetTPS * 60.0) / flywheel.getCPR();
    }

    public boolean reachedRPM() {
        return Math.abs(getTargetRPM() - getRPM()) <= RPM_REACHED_THRESHOLD;
    }

    public double getRawHoodPosition() {
        return hood.getRawPosition();
    }

    public double getTurretPosition() {
        return turret.getCurrentPosition();
    }

    public double getTurretAngle(AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(turret.getDistance()) : turret.getDistance();
    }

    public void setRawHoodPosition(double angle) {
        hood.set(angle + HOOD_POSSIBLE_MIN);
    }

    public double getRecoveryTime() {
        return recoveryTime;
    }

    /**
        Sets the hood angle relative to the ground

       * @param angle the hood angle given by the calculations (deg)
     **/
    public void setVerticalAngle(double angle) {
        double normalized = (angle - 62.5 * Math.PI / 180) / (-34.7 * Math.PI / 180); // normalized/converted to servo position
        setRawHoodPosition(MathFunctions.clamp(normalized, ShooterConfig.HOOD_POSSIBLE_MIN, 1));
    }

    public void setHorizontalAngle(double targetAngleRad) {
        wrapped = ShooterCalculator.wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP);
        turret.setTargetDistance(wrapped);

    }

    private void setRPM(double rpm) {
        this.targetTPS = (rpm * flywheel.getCPR()) / 60.0;
    }

    private void calculateRecovery() {
        if (FLYWHEEL_TARGET - getRPM() > RPM_REACHED_THRESHOLD) {
            if (!calculatedRecovery) {
                timerEx.restart();
                timerEx.start();
                calculatedRecovery = true;
                shotHoodAngle = Math.toDegrees(solution.getVerticalAngle());
                shotTurretAngle = Math.toDegrees(solution.getHorizontalAngle());
                shotFlywheelRPM = getRPM();
                shotGoalDistance = poseTracker.getPose().distanceFrom(goalPose) * inchesToMeters;
            }
        } else if (FLYWHEEL_TARGET - getRPM() <= RPM_REACHED_THRESHOLD && calculatedRecovery) {
            recoveryTime = timerEx.getElapsed();
            timerEx.pause();
            calculatedRecovery = false;
        }
    }
}
