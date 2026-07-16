package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import androidx.core.math.MathUtils;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.LaunchZone;
import org.firstinspires.ftc.teamcode.opModes.TeleOpApp;
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

    private final IShooterCalculator shooterCalculatorClose;
    private final IShooterCalculator shooterCalculatorFar;
    private final Alliance alliance;
    private Pose goalPose;
    private Pose goalPoseFar;

    private boolean isConfiguredCW = true;
    private double targetTPS;

    private final TimerEx recoveryTimer;
    private final TimerEx stallTimer;
    private final TimerEx rampTimer;
    private final TimerEx startupTimer;
    private final Kinematics kinematics;

    public double wrapped;
    public double shotHoodAngle; // in degrees
    public double shotTurretAngle; // in degrees
    public double shotFlywheelRPM;
    public double shotGoalDistance; // in meters

    private boolean horizontalManualMode;
    private boolean verticalManualMode;

    private double horizontalOffset = 0;
    private double verticalOffset = 0;
    private double lastShotRPM;

    private boolean canShootRPMCalc;

    private boolean emergencyStop = false;
    private boolean updateHood = true;
    private boolean updateFlywheel = true;
    public boolean disabled = false;
    public boolean turretDisabled = false;

    public Pose currentPose;

    private final double[] rpmBuffer = new double[RPM_WINDOW_SIZE];
    private int bufferIndex = 0;
    private double runningRpmSum = 0;
    public double filteredRPM;

    private final PIDController flywheelPID;
    private double lastSpeedFlywheel = 0;

    private long lastLoopTime = 0;

    private double lastTargetAngle = 0;
    private long lastTargetUpdateTime = 0;
    private double targetVel = 0;
    private double targetAccel = 0;
    private double filteredTargetVel = 0;
    private double filteredTargetAccel = 0;
    private double filteredTargetAccelFlywheel = 0;
    private double lastFilteredTargetVel = 0;

    // --- Gyroscopic coupling (turret <-> flywheel speed change) ---
    private double lastFilteredRPM = 0;
    private long lastFilteredRPMTime = 0;
    private double filteredRPMAccel = 0; // RPM/s, derived from filteredRPM (already smoothed)
    private double filteredVelocity = 0; // flywheel velocity
    private final double VELOCITY_FILTER_ALPHA = 0.7; // Adjust (0.1 = heavy filter, 0.9 = light)
    private boolean isBraking = false;

    private double lastWrappedTarget = 0;
    private boolean hasWrappedTarget = false;
    private boolean justWrapped = false;

    private double voltage = 12;
    private boolean voltageExternallySupplied = false;
    private LaunchZone zoneCalculator = LaunchZone.CLOSE;
    private boolean sotmEnabled = true;
    double gyroFF = 0;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker, IShooterCalculator shooterCalculatorClose,IShooterCalculator shooterCalculatorFar, Alliance alliance) {
        this.poseTracker = poseTracker;
        this.kinematics = new Kinematics();

        flywheel1 = new ModifiedMotorEx(hardwareMap, FLYWHEEL1_NAME,
                FLYWHEEL_MOTOR.getCPR() * FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MOTOR.getRPM() / FLYWHEEL_GEAR_RATIO);
        flywheel1.setInverted(FLYWHEEL1_INVERTED);

        flywheel2 = new ModifiedMotorEx(hardwareMap, FLYWHEEL2_NAME,
                FLYWHEEL_MOTOR.getCPR() * FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MOTOR.getRPM() / FLYWHEEL_GEAR_RATIO);
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
        turret.setInverted(TURRET_INVERTED);

        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPID.setTolerance(TURRET_POSITION_TOLERANCE, TURRET_VELOCITY_TOLERANCE);

        setHorizontalAngle(0);

        hood = new ServoEx(hardwareMap, HOOD_NAME);

        this.shooterCalculatorClose = shooterCalculatorClose;
        this.shooterCalculatorFar = shooterCalculatorFar;
        this.alliance = alliance;
        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;
        this.goalPoseFar = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL_FAR : GoalPositions.RED_GOAL_FAR;

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
            canShootRPMCalc = false;
            return;
        }
        // --------------------

        if (!voltageExternallySupplied) {
            this.voltage = voltageSensor.getVoltage();
        }

        kinematics.update(poseTracker, ACCELERATION_SMOOTHING_GAIN);

        Vector shotVelocity = sotmEnabled ? poseTracker.getVelocity() : new Vector();

        filteredRPM = getFilteredRPM(getRPM());
        updateFilteredRPMAccel();
        if (zoneCalculator == LaunchZone.CLOSE)
            solution = shooterCalculatorClose.getShootingSolution(currentPose == null ? poseTracker.getPose() : currentPose, goalPose, shotVelocity, poseTracker.getAngularVelocity(), (int)filteredRPM);
        else if (zoneCalculator == LaunchZone.FAR) {
            solution = shooterCalculatorFar.getShootingSolution(currentPose == null ? poseTracker.getPose() : currentPose, goalPoseFar, shotVelocity, poseTracker.getAngularVelocity(), (int)filteredRPM);
        }

        //solution = shooterCalculator.getShootingSolution(currentPose == null ? poseTracker.getPose() : currentPose, goalPose, turretGoalPose , poseTracker.getVelocity(), poseTracker.getAngularVelocity(), (int)filteredRPMPredicted);
        canShootRPMCalc = solution.getCanShoot();

        if (!horizontalManualMode && !disabled) setHorizontalAngle(solution.getHorizontalAngle() + horizontalOffset);
        if (!verticalManualMode && updateHood && !disabled) setVerticalAngle(solution.getVerticalAngle() + verticalOffset);
        if (updateFlywheel) setRPM(solution.getRPM());

        //updateFlywheelPID(false);
        updateFlywheelPIDFiltered(false);
        //updateFlywheelPID(false);
        //updateTurretPID(true);
        if (!turretDisabled) updateTurretPID(false);

        voltageExternallySupplied = false;
    }

    private void updateFlywheelPIDFiltered(boolean useDelayCompensation) {
        if (disabled || emergencyStop) {
            flywheel.set(0);
            rampTimer.restart();
            rampTimer.pause();
            lastSpeedFlywheel = 0;
            lastLoopTime = System.nanoTime();
            return;
        }

        // 1. Calculate real Delta Time (dt)
        long currentTime = System.nanoTime();
        if (lastLoopTime == 0) lastLoopTime = currentTime;
        double dt = (currentTime - lastLoopTime) / 1.0e9;
        lastLoopTime = currentTime;
        if (dt < 0.001) dt = 0.001;

        if (!rampTimer.isOn()) rampTimer.resume();
        double rampMultiplier = Math.min(rampTimer.getElapsed() / INITIAL_RAMP_DURATION, 1.0);

        // 'speed' is our Target Velocity
        double speed = targetTPS * rampMultiplier;

        // Get raw measurement and apply Low Pass Filter to mitigate sensor noise
        //double rawVelocity = flywheel1.getCorrectedVelocity();
        //filteredVelocity = (VELOCITY_FILTER_ALPHA * rawVelocity) + (1.0 - VELOCITY_FILTER_ALPHA) * filteredVelocity;

        double tpsPerRPM = flywheel.getCPR() / 60.0;
        double processVariable = filteredRPM * tpsPerRPM;

        // 2. Delay Compensation (Lead Compensation)
        // We add predicted velocity gain to offset the phase lag of the filter and mechanical latency
        if (useDelayCompensation) {
            double accel = Kinematics.lowPassFilter(flywheel1.getAcceleration(), filteredTargetAccelFlywheel, 0.3);
            processVariable += accel * FLYWHEEL_DELAY_SEC;
        }

        // --- ASYMMETRIC P-GAIN LOGIC ---
        double error = speed - processVariable;

        if (!isBraking && error < BRAKE_ENTRY_THRESHOLD * tpsPerRPM) {
            isBraking = true;
        } else if (isBraking && error > BRAKE_EXIT_THRESHOLD * tpsPerRPM) {
            isBraking = false;
        }

        if (isBraking) {
            flywheelPID.setPID(FLYWHEEL_KP_DOWN, FLYWHEEL_KI, FLYWHEEL_KD);
        } else {
            flywheelPID.setPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        }

        // 3. Calculate Acceleration for Feedforward
        double targetAcceleration = (speed - lastSpeedFlywheel) / dt;
        double currentKA = (targetAcceleration >= 0) ? FLYWHEEL_KA : FLYWHEEL_KA_DOWN;

        // PID and FF now output volts directly
        double pid = flywheelPID.calculate(processVariable, speed);

        // Feedforward calculates based on TARGET (speed), not measurement
        double ff = (FLYWHEEL_KS * Math.signum(speed)) +
                (FLYWHEEL_KV * speed) +
                (currentKA * targetAcceleration);

        lastSpeedFlywheel = speed;

        double desiredVoltage = pid + ff;
        desiredVoltage = Math.max(-voltage, Math.min(voltage, desiredVoltage));

        flywheel.set(desiredVoltage / voltage);
    }

    private void updateTurretPID(boolean useDelayCompensation) {
        if ((disabled || emergencyStop || filteredRPM < 200) && !horizontalManualMode) {
            turret.set(0);
            return;
        }

        double measuredPos = turret.getDistance();

        if (useDelayCompensation) {
            double measuredVel = turret.getVelocity();
            measuredPos += (measuredVel * TURRET_DELAY);
        }

        double pid = turretPID.calculate(measuredPos);
        double error = turretPID.getPositionError();

        if (Math.abs(error) > TURRET_IZONE) {
            turretPID.clearTotalError();
        }

        double[] netKinematics = getNetTargetKinematics();
        double netVel = netKinematics[0];
        double netAccel = netKinematics[1];

        double ffBase = (netVel * TURRET_KV) + (netAccel * TURRET_KA);

        // --- Gyroscopic coupling compensation ---
        // A spinning flywheel changing speed while the turret is also rotating (or
        // accelerating) couples a reaction torque into the turret axis. Both candidate
        // models are wired in with independent gains, so either or both can be tuned
        // in by setting the corresponding TURRET_KGYRO_* constant — set to 0 to disable.
        if (useGyroCompensatoin) {
            gyroFF = (TURRET_KGYRO_VEL * netVel * filteredRPMAccel)
                    + (TURRET_KGYRO_ACCEL * netAccel * filteredRPMAccel);
        }
        else if (!useGyroCompensatoin) gyroFF = 0;

        double totalRequest = pid + ffBase + gyroFF;

        double[] ks = getBandedTurretKS();
        double ks_cw  = ks[0];
        double ks_ccw = ks[1];
        double staticComp = 0;

        if (Math.abs(totalRequest) > TURRET_MIN_VOLTAGE) {
            staticComp = (totalRequest > 0) ? TURRET_KS : -TURRET_KS;
        }

        double desiredVoltage = totalRequest + staticComp;
        desiredVoltage = Math.max(-voltage, Math.min(voltage, desiredVoltage));

        turret.set(desiredVoltage / voltage);
    }

    /**
     * Calculates the required velocity and acceleration for the turret to track the moving target.
     * @return an array where [0] is Net Velocity and [1] is Net Acceleration
     */
    public double[] getNetTargetKinematics() {
        long currentTime = System.nanoTime();
        double currentTargetAngle = turretPID.getSetPoint();

        if (lastTargetUpdateTime == 0) {
            lastTargetUpdateTime = currentTime;
            lastTargetAngle = currentTargetAngle;
            return new double[] {0, 0};
        }

        double dt = (currentTime - lastTargetUpdateTime) / 1e9;

        // Skip on startup, tiny dt, or a turret wrap event — a wrap causes a
        // near-instant jump in the raw setpoint that isn't real motion, and
        // letting it into the derivative chain spikes filteredTargetAccel.
        if (dt <= 0.005 || startupTimer.getElapsed() < 0.5 || justWrapped) {
            lastTargetAngle = currentTargetAngle;
            lastTargetUpdateTime = currentTime;
            targetVel = 0;
            return new double[] {0, 0};
        }

        // 1. Calculate Raw Velocity
        double deltaAngle = MathFunctions.normalizeAngleSigned(currentTargetAngle - lastTargetAngle);
        double rawTargetVel = deltaAngle / dt;
        rawTargetVel = MathUtils.clamp(rawTargetVel, -4, 4);

        targetVel = rawTargetVel;
        lastTargetAngle = currentTargetAngle;
        lastTargetUpdateTime = currentTime;

        // 2. Filter velocity FIRST
        filteredTargetVel = Kinematics.lowPassFilter(rawTargetVel, filteredTargetVel, TURRET_DERIVATIVE_GAIN);

        // 3. Derive acceleration from the FILTERED velocity, not the raw one —
        // differencing raw velocity twice compounds noise before any smoothing
        // has a chance to act on it.
        double rawTargetAccel = (filteredTargetVel - lastFilteredTargetVel) / dt;
        rawTargetAccel = MathUtils.clamp(rawTargetAccel, -4, 4);
        lastFilteredTargetVel = filteredTargetVel;

        filteredTargetAccel = Kinematics.lowPassFilter(rawTargetAccel, filteredTargetAccel, TURRET_SECOND_DERIVATIVE_GAIN);

        // 4. Compute Net Motion
        double netVel = filteredTargetVel;
        double netAccel = filteredTargetAccel;

        return new double[] {netVel, netAccel};
    }

    private double[] getBandedTurretKS() {
        double rpm = Math.abs(filteredRPM);
        if      (rpm < 1000) return new double[]{ TURRET_KS_CW_0,    TURRET_KS_CCW_0    };
        else if (rpm < 2000) return new double[]{ TURRET_KS_CW_1000, TURRET_KS_CCW_1000 };
        else if (rpm < 3000) return new double[]{ TURRET_KS_CW_2000, TURRET_KS_CCW_2000 };
        else                 return new double[]{ TURRET_KS_CW_3000, TURRET_KS_CCW_3000 };
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

    /**
     * Derives flywheel angular acceleration from the already-averaged filteredRPM signal.
     * Since filteredRPM is a moving average, its derivative is far less noisy than
     * differentiating the raw encoder velocity directly.
     */
    private double updateFilteredRPMAccel() {
        long currentTime = System.nanoTime();

        if (lastFilteredRPMTime == 0) {
            lastFilteredRPMTime = currentTime;
            lastFilteredRPM = filteredRPM;
            return 0;
        }

        double dt = (currentTime - lastFilteredRPMTime) / 1e9;
        if (dt < 0.001) dt = 0.001;

        double rawAccel = (filteredRPM - lastFilteredRPM) / dt; // RPM/s

        // Light additional smoothing — this is a derivative of a signal that's
        // already averaged, so a small alpha here mostly just kills quantization noise.
        filteredRPMAccel = Kinematics.lowPassFilter(rawAccel, filteredRPMAccel, FLYWHEEL_ACCEL_FILTER_ALPHA);

        lastFilteredRPM = filteredRPM;
        lastFilteredRPMTime = currentTime;

        return filteredRPMAccel;
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

    public boolean getCanShootRPMCalc() {
        return canShootRPMCalc;
    }

    // checking if the flywheel and the turret is ready to shoot
    public boolean getCanShoot() {
        return canShootRPMCalc && reachedAngle();
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

    public double getTurretAngleVel() {
        return turret.getVelocity();
    }

    public double getTurretAngleAccel() {
        return turret.getAcceleration();
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

    public void setHorizontalOffset(double offset) {
        horizontalOffset = offset;
    }

    public void setVerticalOffset(double offset) {
        verticalOffset = offset;
    }
    public void setZoneCalculator (LaunchZone zone) { zoneCalculator = zone;}

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
        double clampedAngle = MathUtils.clamp(angleRad, HOOD_USABLE_MIN, HOOD_USABLE_MAX);

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
        double lastCmd = hasWrappedTarget ? lastWrappedTarget : turret.getDistance();

        wrapped = IShooterCalculator.wrapToTarget(lastCmd, targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP, lastCmd);

        // A real wrap shows up as a big discontinuous jump in the commanded setpoint
        justWrapped = hasWrappedTarget && Math.abs(wrapped - lastWrappedTarget) > TURRET_WRAP_JUMP_THRESHOLD;

        lastWrappedTarget = wrapped;
        hasWrappedTarget = true;

        turretPID.setSetPoint(wrapped);
        turret.setTargetDistance(wrapped);
    }

    public void setUpdateHood(boolean enabled) {
        updateHood = enabled;
    }

    public void setUpdateFlywheel(boolean enabled) {
        updateFlywheel = enabled;
    }

    public void updateVoltage(double voltage) {
        this.voltage = voltage;
        voltageExternallySupplied = true;
    }

    public boolean getJustWrapped() {
        return justWrapped;
    }

    public void disable() {
        disabled = true;
        flywheel.stopMotor();
    }

    public void disableTurret() {
        turretDisabled = true;
    }

    public void enable() {
        disabled = false;
    }

    public void enableTurret() {
        turretDisabled = false;
    }

    public void setTargetPose(Pose pose) {
        currentPose = pose;
    }

    public void setRPM(double rpm) {
        rpm = MathUtils.clamp(rpm, 0, flywheel.getMaxRPM());
        this.targetTPS = (rpm * flywheel.getCPR()) / 60.0;
    }

    public void setSOTMEnabled(boolean enabled) {
        sotmEnabled = enabled;
    }

    public boolean getSOTMEnabled() {
        return sotmEnabled;
    }

    public void setGoalPose(Pose bluePose, Pose redPose) {
        this.goalPose = alliance == Alliance.BLUE ? bluePose : redPose;
    }

    public void setGoalPoseFar(Pose bluePose, Pose redPose) {
        this.goalPoseFar = alliance == Alliance.BLUE ? bluePose : redPose;
    }

    public Pose getGoalPose() {
        return goalPose;
    }

    public Pose getGoalPoseFar() {
        return goalPoseFar;
    }

    public boolean justShot() {
        if ((Math.abs(getTargetRPM() - getRPM()) > RPM_REACHED_THRESHOLD) && (Math.abs(lastShotRPM - getRPM()) > RPM_REACHED_THRESHOLD)) {
            return true;
        }

        lastShotRPM = getRPM();
        return false;
    }
}
