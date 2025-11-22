package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

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
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;

import java.util.concurrent.TimeUnit;

@Config
public class Shooter extends SubsystemBase {
    public ShootingSolution solution;

    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    private final Motor turret;
    private final ServoEx hood;
    private final CRServoEx transfer;
    private final ServoEx kicker;

    private final ShooterCalculator shooterCalculator;
    private final Alliance alliance;
    private final Pose goalPose;

    /**
     * Flywheel velocity [ticks/second]
     */
    private double velocity;

    private final TimerEx timerEx;
    private boolean calculatedRecovery = false;
    private double recoveryTime;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker, ShooterCalculator shooterCalculator, Alliance alliance) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, FLYWHEEL_NAME, FLYWHEEL_MOTOR);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);
        flywheel.setInverted(FLYWHEEL_INVERTED);

        turret = new Motor(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setPositionCoefficient(TURRET_KP);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));
        setHorizontalAngle(0);

        hood = new ServoEx(hardwareMap, HOOD_NAME);
        setVerticalAngle(1);

        transfer = new CRServoEx(hardwareMap, TRANSFER_NAME);
        toggleTransfer(false);

        kicker = new ServoEx(hardwareMap, KICKER_NAME);
        kicker.set(0);

        setRPM(FLYWHEEL_TARGET);

        this.shooterCalculator = shooterCalculator;
        this.alliance = alliance;
        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;

        timerEx = new TimerEx(TimeUnit.MILLISECONDS);
    }

    @Override
    public void periodic() {
        solution = shooterCalculator.getShootingSolution(poseTracker.getPose(), goalPose, poseTracker.getVelocity());

        setHorizontalAngle(solution.getHorizontalAngle());
        setVerticalAngle(solution.getVerticalAngle());
//        setVelocity(solution.getVelocity());

        flywheel.setVelocity(velocity);

        calculateRecovery();

        turret.set(1);
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

    private void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getRPM() {
        double motorTPS = flywheel.getCorrectedVelocity();
        return (motorTPS * 60.0) / flywheel.getCPR();
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

    private void setRPM(double rpm) {
        this.velocity = (rpm * flywheel.getCPR()) / 60.0;
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
        double wrapped = ShooterCalculator.wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP);
        turret.setTargetDistance(wrapped);
    }

    private void calculateRecovery() {
        if (FLYWHEEL_TARGET - getRPM() > SHOOT_THRESHOLD) {
            if (!calculatedRecovery) {
                timerEx.restart();
                timerEx.start();
                calculatedRecovery = true;
            }
        } else if (FLYWHEEL_TARGET - getRPM() <= SHOOT_THRESHOLD && calculatedRecovery) {
            recoveryTime = timerEx.getElapsed();
            timerEx.pause();
            calculatedRecovery = false;
        }
    }
}
