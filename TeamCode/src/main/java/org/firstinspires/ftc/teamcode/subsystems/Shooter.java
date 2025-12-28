package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import androidx.core.math.MathUtils;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.utilities.Debugger;
import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorEx;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Transform3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.util.concurrent.TimeUnit;

public class Shooter extends SubsystemBase {
    public ShootingSolution solution;

    private final PoseTracker poseTracker;

    private final ModifiedMotorEx flywheel;
    private final ModifiedMotorEx turret;
    private final ServoEx hood;

    private final PIDController turretPID;
    private final SimpleMotorFeedforward turretFeedforward;

    private final VoltageSensor voltageSensor;

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

    private boolean horizontalManualMode;
    private boolean verticalManualMode;

    private double horizontalOffset = 0;
    private double verticalOffset = 0;
    private double lastshotRPM = getRPM();

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker, IShooterCalculator shooterCalculator, Alliance alliance) {
        this.poseTracker = poseTracker;

        flywheel = new ModifiedMotorEx(hardwareMap, FLYWHEEL_NAME, FLYWHEEL_MOTOR);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);
        flywheel.setInverted(FLYWHEEL_INVERTED);
        flywheel.setDelayCompensation(FLYWHEEL_DELAY_SEC);

        turret = new ModifiedMotorEx(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));

        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPID.setTolerance(ANGLE_REACHED_THRESHOLD);

        turretFeedforward = new SimpleMotorFeedforward(TURRET_KS, TURRET_KV, TURRET_KA);

        setHorizontalAngle(0);

        hood = new ServoEx(hardwareMap, HOOD_NAME);

        this.shooterCalculator = shooterCalculator;
        this.alliance = alliance;
        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;

        timerEx = new TimerEx(TimeUnit.SECONDS);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        lastshotRPM = getRPM();
    }

    @Override
    public void periodic() {
        solution = shooterCalculator.getShootingSolution(poseTracker.getPose(), goalPose, poseTracker.getVelocity(), poseTracker.getAngularVelocity(), (int)getRPM());

        if (!horizontalManualMode) setHorizontalAngle(solution.getHorizontalAngle() + horizontalOffset);
        if (!verticalManualMode) setVerticalAngle(solution.getVerticalAngle() + verticalOffset);
        setRPM(solution.getVelocity());

        calculateRecovery();

        double voltage = voltageSensor.getVoltage();
        flywheel.setVelocity(targetTPS, voltage);

        // Turret PIDF
        double pid = turretPID.calculate(turret.getDistance());
        double feedforward = turretFeedforward.calculate(-poseTracker.getAngularVelocity(), -poseTracker.getAcceleration().getTheta());
        double result = pid + feedforward;

        turret.set(result, voltage);
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

    public boolean reachedAngle() {
        return turretPID.atSetPoint();
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

    public void resetTurret() {
        turret.resetEncoder();
    }

    public void setHoodPosition(double angle) {
        hood.set(MathFunctions.clamp(angle, HOOD_POSSIBLE_MIN, HOOD_POSSIBLE_MAX));
    }

    // get hood angle in deg
    public double getHoodAngle() {
        return (-34.7) * getRawHoodPosition() + 62.5;
    }


    public void setHorizontalManualMode(boolean enabled) {
        horizontalManualMode = enabled;
    }

    public void setVerticalManualMode(boolean enabled) {
        verticalManualMode = enabled;
    }

    public boolean getHorizontalManualMode() {
        return horizontalManualMode;
    }

    public boolean getVerticalManualMode() {
        return verticalManualMode;
    }

    public double getRecoveryTime() {
        return recoveryTime;
    }

    public void setHorizontalOffset(double offset) {
        horizontalOffset = offset;
    }

    public void setVerticalOffset(double offset) {
        verticalOffset = offset;
    }

    public double getHorizontalOffset() {
        return horizontalOffset;
    }

    public double getVerticalOffset() {
        return verticalOffset;
    }

    /**
        Sets the hood angle relative to the ground

       * @param angleRad the hood angle given by the calculations (radians)
     **/
    public void setVerticalAngle(double angleRad) {
        double normalized = (angleRad - HOOD_MIN) / (HOOD_MAX - HOOD_MIN); // Normalized/converted to servo position
        double invertedNormalized = 1 - normalized;

        setHoodPosition(invertedNormalized);
    }

    public void setHorizontalAngle(double targetAngleRad) {
        wrapped = IShooterCalculator.wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP);
        turretPID.setSetPoint(wrapped);
        turret.setTargetDistance(wrapped);
    }

    private void setRPM(double rpm) {
        rpm = MathUtils.clamp(rpm, 0, flywheel.getMaxRPM());
        this.targetTPS = (rpm * flywheel.getCPR()) / 60.0;
    }

    // returns true if we just shot, otherwise false
    private boolean wasBallshot() {

        if ((Math.abs(getTargetRPM() - getRPM()) >  SHOT_RPM_DROP) && (Math.abs(lastshotRPM - getRPM()) > SHOT_RPM_DROP)) {
            return true;
        }

        lastshotRPM = getRPM();
        return false;
    }

    // checks how much time it took for the flywheel to reach the setpoint
    // NOTE: the setpoint may CHANGE, so this function checks the time it took for the flywheel to follow the targetRPM
    private void calculateRecovery() {
        if (wasBallshot()) {
            if (!calculatedRecovery) {
                timerEx.restart();
                timerEx.start();
                calculatedRecovery = true;
                shotHoodAngle = Math.toDegrees(solution.getVerticalAngle());
                shotTurretAngle = Math.toDegrees(solution.getHorizontalAngle());
                shotFlywheelRPM = getRPM();
                shotGoalDistance = poseTracker.getPose().distanceFrom(goalPose) / inchesToMeters;
                Logger.recordOutput("Shot/trajectory",Debugger.generateTrajectory(new Translation3d(poseTracker.getPose().getX(), poseTracker.getPose().getY(), SHOOT_HEIGHT), solution.getVelocityMetersPerSec() * inchesToMeters, shotHoodAngle, 2, 0.2));
            }
        } else if (getTargetRPM() - getRPM() <= RPM_REACHED_THRESHOLD && calculatedRecovery) {
            recoveryTime = timerEx.getElapsed();
            timerEx.pause();
            calculatedRecovery = false;
        }
    }
}
