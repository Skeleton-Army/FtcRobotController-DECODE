package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import androidx.core.math.MathUtils;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.OpModeManager;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.utilities.Kinematics;
import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorEx;
import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorGroup;

import java.util.concurrent.TimeUnit;

public class Shooter extends SubsystemBase {
    public ShootingSolution solution;

    private final PoseTracker poseTracker;

    private final ModifiedMotorEx flywheel1;
    private final ModifiedMotorEx flywheel2;
    private final ModifiedMotorGroup flywheel;
    private final ModifiedMotorEx turret;
    private final ServoEx hood;

    private final PIDController turretPID;

    private final VoltageSensor voltageSensor;

    private final IShooterCalculator shooterCalculator;
    private final Alliance alliance;
    public Pose goalPose;

    private double targetTPS;

    private final TimerEx recoveryTimer;
    private final TimerEx stallTimer;
    private final Kinematics kinematics;

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
    private double lastshotRPM;

    private boolean canShoot;
    private double filteredRPM = 0;

    private boolean emergencyStop = false;
    private boolean updateHood = true;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker, IShooterCalculator shooterCalculator, Alliance alliance) {
        this.poseTracker = poseTracker;
        this.kinematics = new Kinematics();

        flywheel1 = new ModifiedMotorEx(hardwareMap, FLYWHEEL1_NAME, FLYWHEEL_MOTOR);
        flywheel1.setInverted(FLYWHEEL1_INVERTED);

        flywheel2 = new ModifiedMotorEx(hardwareMap, FLYWHEEL2_NAME, FLYWHEEL_MOTOR);
        flywheel2.setInverted(FLYWHEEL2_INVERTED);

        flywheel = new ModifiedMotorGroup(flywheel1, flywheel2);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);
        flywheel.setDelayCompensation(FLYWHEEL_DELAY_SEC);
        flywheel.setCurrentAlert(CURRENT_THRESHOLD, CurrentUnit.AMPS);

        turret = new ModifiedMotorEx(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));

        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPID.setTolerance(TURRET_POSITION_TOLERANCE, TURRET_VELOCITY_TOLERANCE);

        setHorizontalAngle(0);

        hood = new ServoEx(hardwareMap, HOOD_NAME);

        this.shooterCalculator = shooterCalculator;
        this.alliance = alliance;
        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;

        recoveryTimer = new TimerEx(TimeUnit.SECONDS);
        stallTimer = new TimerEx(TimeUnit.SECONDS);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        if (poseTracker == null) return;

        // --- SAFETY CHECK ---
        if (emergencyStop) return;

        if (isFlywheelDamaged() && !emergencyStop) {
            flywheel.stopMotor();
            turret.stopMotor();
            emergencyStop = true;
            canShoot = false;
            return;
        }
        // --------------------

        calculateGoalPose(); // calculates the current goalpose by the robot pose

        kinematics.update(poseTracker, ACCELERATION_SMOOTHING_GAIN);

        solution = shooterCalculator.getShootingSolution(poseTracker.getPose(), goalPose, poseTracker.getVelocity(), poseTracker.getAngularVelocity(), (int)getFilteredRPM());
        canShoot = solution.getCanShoot();

        if (!horizontalManualMode) setHorizontalAngle(solution.getHorizontalAngle() + horizontalOffset);
        if (!verticalManualMode && updateHood) setVerticalAngle(solution.getVerticalAngle() + verticalOffset);
        setRPM(solution.getRPM());

        calculateRecovery();

        double voltage = voltageSensor.getVoltage();
        flywheel.setVelocity(targetTPS, voltage);

        // ----- TURRET PIDF -----
        double pid = turretPID.calculate(turret.getDistance());
        double error = turretPID.getPositionError();

        // Only apply kS if we aren't at the target to prevent high-frequency oscillation
        double staticComp = 0;
        if (!turretPID.atSetPoint()) {
            staticComp = Math.signum(error) * TURRET_KS;
        }

        // If we are more than X degrees away, clear the integral sum
        // This prevents it from building up during the high-speed part of the move
        if (Math.abs(error) > TURRET_IZONE) {
            turretPID.clearTotalError();
        }

        // Robot rotation compensation
        double robotVel = poseTracker.getAngularVelocity();
        double robotAcc = kinematics.getAngularAcceleration();

        double feedforward = staticComp + (-robotVel * TURRET_KV) + (-robotAcc * TURRET_KA);
        double result = pid + feedforward;

        turret.set(result, voltage);
    }

    public boolean isFlywheelDamaged() {
        double currentRPM = Math.abs(getRPM());
        double targetRPM = Math.abs(getTargetRPM());
        boolean isOverCurrent = flywheel.isOverCurrent();

        // 1. Encoder Direction Check
        if ((getRPM() < -50  && targetRPM > 0) || (getRPM() > 50 && targetRPM < 0)) {
            RobotLog.addGlobalWarningMessage("FLYWHEEL IS SPINNING IN THE WRONG DIRECTION.");
            return true;
        }

        // 2. Conflict/Stall Check
        // Trying to spin fast, but barely moving and drawing high current
        boolean isStalling = targetRPM > 1000 && currentRPM < 200 && isOverCurrent;

        if (isStalling) {
            stallTimer.start();
            stallTimer.resume();

            if (stallTimer.getElapsed() > STALL_TIMEOUT) {
                RobotLog.addGlobalWarningMessage("FLYWHEEL STALL DETECTED. ONE OF THE FLYWHEEL MOTORS IS PROBABLY REVERSED.");
                return true;
            }
        } else {
            stallTimer.restart();
            stallTimer.pause();
        }

        return false;
    }

    public double getRPM() {
        double motorTPS = flywheel.getVelocity();
        return (motorTPS * 60.0) / flywheel.getCPR();
    }

    public double getTargetRPM() {
        return (targetTPS * 60.0) / flywheel.getCPR();
    }

    public double getFilteredRPM() {
        filteredRPM = Kinematics.lowPassFilter(getRPM(), filteredRPM, RPM_SMOOTHING_FACTOR);
        return filteredRPM;
    }

    public boolean getCanShoot() {
        return canShoot;
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

    public double getHoodAngle() {
        double pos = getRawHoodPosition();
        double normalizedPos = HOOD_INVERTED ? (1.0 - pos) : pos;

        return HOOD_MIN + (normalizedPos * (HOOD_MAX - HOOD_MIN));
    }

    public double getHoodAngleDegrees() {
        return Math.toDegrees(getHoodAngle());
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

        setHoodPosition(HOOD_INVERTED ? invertedNormalized : normalized);
    }

    public void setHorizontalAngle(double targetAngleRad) {
        wrapped = IShooterCalculator.wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP);
        turretPID.setSetPoint(wrapped);
        turret.setTargetDistance(wrapped);
    }

    public void setUpdateHood(boolean enabled) {
        updateHood = enabled;
    }

    private void setRPM(double rpm) {
        rpm = MathUtils.clamp(rpm, 0, flywheel.getMaxRPM());
        this.targetTPS = (rpm * flywheel.getCPR()) / 60.0;
    }

    // returns true if we just shot, otherwise false
    private boolean wasBallshot() {
        if ((Math.abs(getTargetRPM() - getRPM()) > RPM_REACHED_THRESHOLD) && (Math.abs(lastshotRPM - getRPM()) > RPM_REACHED_THRESHOLD)) {
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
                recoveryTimer.restart();
                recoveryTimer.start();
                calculatedRecovery = true;
                shotHoodAngle = Math.toDegrees(solution.getVerticalAngle());
                shotTurretAngle = Math.toDegrees(solution.getHorizontalAngle());
                shotFlywheelRPM = getRPM();
                shotGoalDistance = poseTracker.getPose().distanceFrom(goalPose) / inchesToMeters;
//                Logger.recordOutput("Shot/trajectory",Debugger.generateTrajectory(new Translation3d(poseTracker.getPose().getX(), poseTracker.getPose().getY(), SHOOT_HEIGHT), solution.getVelocityMetersPerSec() * inchesToMeters, shotHoodAngle, 2, 0.2));
            }
        } else if (getTargetRPM() - getRPM() <= RPM_REACHED_THRESHOLD && calculatedRecovery) {
            recoveryTime = recoveryTimer.getElapsed();
            recoveryTimer.pause();
            calculatedRecovery = false;
        }
    }

    public void calculateGoalPose() {
        double rx = poseTracker.getPose().getX();
        double ry = poseTracker.getPose().getY();

        // The "Static" Centroid (the middle of the blue triangle)
        double centroidX = (GoalPositions.BLUE_GOAL_TOP.getX() + GoalPositions.BLUE_GOAL_BUTTOM.getX() + GoalPositions.BLUE_GOAL_CORNER.getX()) / 3.0;
        double centroidY = (GoalPositions.BLUE_GOAL_TOP.getY() + GoalPositions.BLUE_GOAL_BUTTOM.getY() + GoalPositions.BLUE_GOAL_CORNER.getY()) / 3.0;

        if (alliance == Alliance.BLUE) {
        /* DYNAMIC SHIFT:
           If the robot is near the left wall (low X),
           the "Ideal X" should track the robot's X to keep the shot straight.
        */
            double idealX = Math.max(4.0, Math.min(rx, 24.0)); // Keeps target within the goal's width

            // Calculate distance to the corner to determine how much to "slide" the target
            double distToCorner = Math.hypot(rx - GoalPositions.BLUE_GOAL_CORNER.getX(), ry - GoalPositions.BLUE_GOAL_CORNER.getY());

            // Interpolation factor (t): 1.0 when far away (use centroid), 0.0 when close (use idealX)
            double t = Math.max(0, Math.min((distToCorner - 15.0) / 30.0, 1.0));

            // Set the Absolute Field Coordinates
            goalPose = new Pose(
                    (idealX * (1 - t)) + (centroidX * t),
                    (142.0 * (1 - t)) + (centroidY * t) // Slide Y closer to the back wall as you get near
            );
        }
        else if (alliance == Alliance.RED) {
            // Mirror the logic for the Red side (High X)
            double idealX = Math.max(120.0, Math.min(rx, 140.0));
            double distToCorner = Math.hypot(rx - GoalPositions.RED_GOAL_CORNER.getX(), ry - GoalPositions.RED_GOAL_CORNER.getY());
            double t = Math.max(0, Math.min((distToCorner - 15.0) / 30.0, 1.0));

            goalPose = new Pose(
                    (idealX * (1 - t)) + (centroidX * t),
                    (142.0 * (1 - t)) + (centroidY * t)
            );
        }
    }

}
