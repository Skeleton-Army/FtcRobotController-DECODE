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
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
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
    public final ModifiedMotorGroup flywheel;
    private final ModifiedMotorEx turret;
    private final ServoEx hood;

    private final PIDController turretPID;

    private final VoltageSensor voltageSensor;

    private final IShooterCalculator shooterCalculator;
    private final Alliance alliance;
    public Pose goalPose;
    public Pose turretGoalPose;

    private double targetTPS;

    private final TimerEx recoveryTimer;
    private final TimerEx stallTimer;
    private final TimerEx rampTimer;
    private final TimerEx startupTimer;
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

    private boolean emergencyStop = false;
    private boolean updateHood = true;
    public boolean disabled = false;

    public Pose currentPose;

    private final double[] rpmBuffer = new double[RPM_WINDOW_SIZE];
    private int bufferIndex = 0;
    private double runningRpmSum = 0;
    public double filteredRPM;
    public double filteredRPMPredicted;

    private final PIDController flywheelPID;
    private double lastSpeedFlywheel = 0;
    private double lastSpeedFlywheelFiltered = 0;

    // Add this as a class variable
    private long lastLoopTime = 0;

    private double lastTargetAngle = 0;
    private long lastTargetUpdateTime = 0;
    private double targetVel = 0;
    private double targetAccel = 0;
    private double filteredTargetVel = 0;
    private double filteredTargetAccel = 0;

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
        flywheel.setRunMode(MotorEx.RunMode.RawPower);
        flywheel.setDelayCompensation(FLYWHEEL_DELAY_SEC);
        flywheel.setCurrentAlert(CURRENT_THRESHOLD, CurrentUnit.AMPS);

        flywheelPID = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);

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
        this.turretGoalPose = alliance == Alliance.BLUE ? GoalPositions.TURRET_BLUE_GOAL : GoalPositions.TURRET_RED_GOAL;

        recoveryTimer = new TimerEx(TimeUnit.SECONDS);
        stallTimer = new TimerEx(TimeUnit.SECONDS);
        rampTimer = new TimerEx(TimeUnit.SECONDS);
        rampTimer.start();
        startupTimer = new TimerEx(TimeUnit.SECONDS);
        startupTimer.start();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        if (poseTracker == null) return;
        currentPose = poseTracker.getPose();

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

        kinematics.update(poseTracker, ACCELERATION_SMOOTHING_GAIN);

        filteredRPM = getFilteredRPM(getRPM());
        filteredRPMPredicted = getFilteredRPM(getRPMCorrectedTiming());
        solution = shooterCalculator.getShootingSolution(currentPose == null ? poseTracker.getPose() : currentPose, goalPose, turretGoalPose , poseTracker.getVelocity(), poseTracker.getAngularVelocity(), (int)filteredRPM);
        //solution = shooterCalculator.getShootingSolution(currentPose == null ? poseTracker.getPose() : currentPose, goalPose, turretGoalPose , poseTracker.getVelocity(), poseTracker.getAngularVelocity(), (int)filteredRPMPredicted);
        canShoot = solution.getCanShoot();

        if (!horizontalManualMode && !disabled) setHorizontalAngle(solution.getHorizontalAngle() + horizontalOffset);
        if (!verticalManualMode && updateHood && !disabled) setVerticalAngle(solution.getVerticalAngle() + verticalOffset);
        setRPM(solution.getRPM());

        double voltage = voltageSensor.getVoltage();
        updateFlywheelPID(voltage);
        updateTurretPID();
    }

    public void updateFlywheelPID(double voltage) {
        if (disabled || emergencyStop) {
            flywheel.set(0);
            rampTimer.restart();
            rampTimer.pause();
            lastSpeedFlywheel = 0; // Reset this so we don't get a huge spike on restart
            lastLoopTime = System.nanoTime(); // Reset time
            return;
        }

        // 1. Calculate real Delta Time (dt) in seconds
        long currentTime = System.nanoTime();
        if (lastLoopTime == 0) lastLoopTime = currentTime; // Safety for first run
        double dt = (currentTime - lastLoopTime) / 1.0e9; // Convert nanoseconds to seconds
        lastLoopTime = currentTime;

        // Safety: Prevent division by zero if loops run instantly (unlikely but safe)
        if (dt < 0.001) dt = 0.001;

        if (!rampTimer.isOn()) rampTimer.resume();
        double rampMultiplier = Math.min(rampTimer.getElapsed() / INITIAL_RAMP_DURATION, 1.0);

        double speed = (0.9 * targetTPS) * rampMultiplier;

        // 2. Calculate Acceleration using real dt
        double targetAcceleration = (speed - lastSpeedFlywheel) / dt;
        double currentAcceleration = (filteredRPM - lastSpeedFlywheelFiltered) / dt;
        OpModeManager.getTelemetry().addData("flywheel target acceleration", targetAcceleration);
        OpModeManager.getTelemetry().addData("flywheel acceleration", currentAcceleration);

        // Choose kA based on the direction of acceleration
        double currentKA = (targetAcceleration >= 0) ? FLYWHEEL_KA : FLYWHEEL_KA_DOWN;

        double pid = flywheelPID.calculate(flywheel.getCorrectedVelocity(), speed);
        double ff = (FLYWHEEL_KS * Math.signum(speed)) +
                (FLYWHEEL_KV * speed) +
                (currentKA * targetAcceleration);

        lastSpeedFlywheel = speed;
        lastSpeedFlywheelFiltered = filteredRPM;

        double velocityCmd = pid + ff;
        double finalPower = velocityCmd / flywheel1.ACHIEVABLE_MAX_TICKS_PER_SECOND;

        flywheel.set(finalPower, voltage);
    }

    public void updateTurretPID() {
        if (disabled || emergencyStop) {
            turret.set(0);
            return;
        }

        // 1. PID Calculation
        double pid = turretPID.calculate(turret.getDistance());
        double error = turretPID.getPositionError();

        // If we are more than X degrees away, clear the integral sum
        // This prevents it from building up during the high-speed part of the move
        if (Math.abs(error) > TURRET_IZONE) {
            turretPID.clearTotalError();
        }

        // 2. Robot Motion Compensation
        double[] netKinematics = getNetTargetKinematics();
        double netVel = netKinematics[0];
        double netAccel = netKinematics[1];

        double ffBase = (netVel * TURRET_KV) + (netAccel * TURRET_KA);
        double totalRequest = pid + ffBase;

        // Only apply kS if the baseRequest is actually trying to move the motor
        // and apply it in the direction of that motion.
        double staticComp = 0;

        if (Math.abs(error) > 0.01) {
            if (-error > 0) {
                // Clockwise movement (Positive)
                staticComp = -TURRET_KS_CW;
            } else {
                // Counter-Clockwise movement (Negative)
                staticComp = TURRET_KS_CCW;
            }
        }

        double voltageScale = 12.0 / voltageSensor.getVoltage();
        double finalOutput = (totalRequest + staticComp) * voltageScale;

        turret.set(finalOutput);
    }

    /**
     * Calculates the required velocity and acceleration for the turret to track the moving target.
     * @return an array where [0] is Net Velocity and [1] is Net Acceleration
     */
    private double[] getNetTargetKinematics() {
        long currentTime = System.nanoTime();
        double currentTargetAngle = solution.getHorizontalAngle() + horizontalOffset;

        if (lastTargetUpdateTime == 0) {
            lastTargetUpdateTime = currentTime;
            lastTargetAngle = currentTargetAngle;
            return new double[] {0, 0};
        }

        double dt = (currentTime - lastTargetUpdateTime) / 1e9;

        // This removes the jitter and jumps at the start of the OpMode
        if (dt <= 0.005 || startupTimer.getElapsed() < 0.5) {
            lastTargetAngle = currentTargetAngle;
            lastTargetUpdateTime = currentTime;
            targetVel = 0;
            return new double[] {0, 0};
        }

        // 1. Calculate Raw Derivatives
        double deltaAngle = MathFunctions.normalizeAngleSigned(currentTargetAngle - lastTargetAngle);
        double rawTargetVel = deltaAngle / dt;
        rawTargetVel = MathUtils.clamp(rawTargetVel, -20.0, 20.0);

        double rawTargetAccel = (rawTargetVel - targetVel) / dt;
        rawTargetAccel = MathUtils.clamp(rawTargetAccel, -40.0, 40.0);

        // 2. Apply Low Pass Filter using your Kinematics utility
        filteredTargetVel = Kinematics.lowPassFilter(rawTargetVel, filteredTargetVel, TURRET_DERIVATIVE_GAIN);
        filteredTargetAccel = Kinematics.lowPassFilter(rawTargetAccel, filteredTargetAccel, TURRET_DERIVATIVE_GAIN);

        targetVel = rawTargetVel;
        lastTargetAngle = currentTargetAngle;
        lastTargetUpdateTime = currentTime;

        // 3. Compute Net Motion (Filtered Target - Filtered/Measured Robot)
        double netVel = filteredTargetVel;
        double netAccel = filteredTargetAccel;

        return new double[] {netVel, netAccel};
    }

    public boolean isFlywheelDamaged() {
        double currentRPM = Math.abs(getRPM());
        double targetRPM = Math.abs(getTargetRPM());
        boolean isOverCurrent = flywheel.isOverCurrent();

        // 1. Encoder Direction Check
        if ((getRPM() < -500  && targetRPM > 0) || (getRPM() > 500 && targetRPM < 0)) {
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

    public double getRPMCorrectedTiming() {
        double motorTPS = flywheel.getVelocity();
        if (!Double.isNaN(flywheel1.getAcceleration())) {
             motorTPS += flywheel1.getAcceleration() * FLYWHEEL_SHOOTING_DIFFRENCE;
            return (motorTPS * 60.0) / flywheel.getCPR();
        }

        return (motorTPS * 60.0) / flywheel.getCPR();
    }


    public double getTargetRPM() {
        return (targetTPS * 60.0) / flywheel.getCPR();
    }

    private double getFilteredRPM(double currentRPM) {
        // Subtract the oldest sample from the sum
        runningRpmSum -= rpmBuffer[bufferIndex];

        // Add the new sample to the buffer and the sum
        rpmBuffer[bufferIndex] = currentRPM;
        runningRpmSum += currentRPM;

        // Advance the index (circular buffer)
        bufferIndex = (bufferIndex + 1) % RPM_WINDOW_SIZE;

        // Calculate the average
        double movingAverageRPM = runningRpmSum / RPM_WINDOW_SIZE;
        return movingAverageRPM;
    }

    public boolean getCanShoot() {
        return canShoot;
    }

    public boolean reachedRPM() {
        return Math.abs(getTargetRPM() - getRPM()) <= RPM_REACHED_THRESHOLD;
    }

    public double getTurretWindow() {
        // Map distance to a tolerance: closer is wider, farther is tighter
        double distance = poseTracker.getPose().distanceFrom(goalPose);
        double t = (distance - DIST_CLOSE) / (DIST_FAR - DIST_CLOSE);
        t = MathUtils.clamp(t, 0.0, 1.0);

        double baseWindow = WINDOW_CLOSE + t * (WINDOW_FAR - WINDOW_CLOSE);

        // If we are moving, we open the window slightly to account for latency
        double robotVel = poseTracker.getVelocity().getMagnitude();
        double velocityGrace = robotVel * VELOCITY_WINDOW_GAIN;

        return Math.min(baseWindow + velocityGrace, MAX_WINDOW_SIZE);
    }

    public boolean reachedAngle() {
        double posError = Math.abs(turretPID.getPositionError());
        boolean posReached = posError <= getTurretWindow();

        double velError = Math.abs(turretPID.getVelocityError());
        boolean velReached = velError <= TURRET_VELOCITY_WINDOW;

        return posReached && velReached;
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

    /**
     * Calculates the current physical angle of the hood based on the servo's position.
     * @return physical angle in radians
     */
    public double getHoodAngle() {
        double pos = getRawHoodPosition();

        // Normalize position relative to the possible range
        double normalizedPos = (pos - HOOD_POSSIBLE_MIN) / (HOOD_POSSIBLE_MAX - HOOD_POSSIBLE_MIN);

        if (HOOD_INVERTED) {
            normalizedPos = 1.0 - normalizedPos;
        }

        // Interpolate back to radians
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
     * Sets the hood angle by interpolating the target radians between the
     * physical min/max angles and the servo's possible position range.
     *
     * @param angleRad the target hood angle in radians
     */
    public void setVerticalAngle(double angleRad) {
        // 1. Clamp the input to ensure it stays within physical hardware limits
        double clampedAngle = MathUtils.clamp(angleRad, HOOD_MIN, HOOD_MAX);

        // 2. Calculate the interpolation factor (t) from 0.0 to 1.0
        double t = (clampedAngle - HOOD_MIN) / (HOOD_MAX - HOOD_MIN);

        // 3. Map that factor to the servo's possible position range
        double targetPos = HOOD_POSSIBLE_MIN + (t * (HOOD_POSSIBLE_MAX - HOOD_POSSIBLE_MIN));

        // 4. Handle inversion if the servo rotates opposite to the angle increase
        if (HOOD_INVERTED) {
            targetPos = HOOD_POSSIBLE_MAX - (targetPos - HOOD_POSSIBLE_MIN);
        }

        setHoodPosition(targetPos);
    }

    public void setHorizontalAngle(double targetAngleRad) {
        wrapped = IShooterCalculator.wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP);
        turretPID.setSetPoint(wrapped);
        turret.setTargetDistance(wrapped);
    }

    public void setUpdateHood(boolean enabled) {
        updateHood = enabled;
    }

    public void disable() {
        disabled = true;
        flywheel.stopMotor();
    }

    public void enable() {
        disabled = false;
    }

    public void setTargetPose(Pose pose) {
        currentPose = pose;
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
}
