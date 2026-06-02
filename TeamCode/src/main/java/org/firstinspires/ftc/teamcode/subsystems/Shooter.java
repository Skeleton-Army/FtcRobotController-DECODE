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
import org.firstinspires.ftc.teamcode.enums.LaunchZone;
import org.firstinspires.ftc.teamcode.utilities.Kinematics;
import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorEx;
import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorGroup;

import java.util.concurrent.TimeUnit;

/**
 * Shooter subsystem with full 5-level motor control stack:
 *
 *  Level 1 – Voltage control          (all motors normalised by live battery V)
 *  Level 2 – Current limiting flywheel (driving + regen, via ModifiedMotorEx.applyVoltage)
 *  Level 3 – Current limiting turret   (driving + regen, via ModifiedMotorEx.applyVoltage)
 *  Level 4 – Torque-current control    (flywheel uses back-EMF feedforward)
 *  Level 5 – PID on top of torque-current (stable, voltage-normalised velocity loop)
 *
 * Current limits and physics constants are configured on each motor instance
 * in the constructor via setMotorPhysicsConstants() + setCurrentLimits().
 * After that, every applyVoltage() call automatically enforces:
 *   - Driving current ceiling  (protects fuses and gears on hard pushes)
 *   - Regen current ceiling    (protects Control Hub bus from decel voltage spikes)
 *   - Battery voltage normalisation  (consistent feel as battery sags)
 *
 * No subsystem code needs to call getCurrentLimitedVoltage() manually.
 * Any other subsystem in the robot that uses ModifiedMotorEx + setCurrentLimits()
 * gets the same protection automatically.
 *
 * Add the following constants to ShooterConfig (see ShooterConfig_additions.java):
 *
 *   FLYWHEEL_STALL_CURRENT_A, FLYWHEEL_NO_LOAD_RPM,
 *   FLYWHEEL_STALL_TORQUE_NM, FLYWHEEL_NO_LOAD_CURRENT_A,
 *   FLYWHEEL_MAX_DRIVING_CURRENT_A, FLYWHEEL_MAX_REGEN_CURRENT_A
 *
 *   TURRET_STALL_CURRENT_A,  TURRET_NO_LOAD_RPM,
 *   TURRET_STALL_TORQUE_NM,  TURRET_NO_LOAD_CURRENT_A,
 *   TURRET_MAX_DRIVING_CURRENT_A, TURRET_MAX_REGEN_CURRENT_A
 */
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

    private final IShooterCalculator shooterCalculatorClose;
    private final IShooterCalculator shooterCalculatorFar;
    private final Alliance alliance;
    public Pose goalPose;
    public Pose goalPoseFar;

    public boolean isConfiguredCW = true;
    private double targetTPS;

    private final TimerEx recoveryTimer;
    private final TimerEx stallTimer;
    private final TimerEx rampTimer;
    private final TimerEx startupTimer;
    private final Kinematics kinematics;

    public boolean calculatedRecovery = false;
    private double recoveryTime;
    public double wrapped;
    public double shotHoodAngle;
    public double shotTurretAngle;
    public double shotFlywheelRPM;
    public double shotGoalDistance;
    final double inchesToMeters = 39.37;

    private boolean horizontalManualMode;
    private boolean verticalManualMode;

    private double horizontalOffset = 0;
    private double verticalOffset   = 0;
    private double lastShotRPM;

    private boolean canShootRPMCalc;

    private boolean emergencyStop   = false;
    private boolean updateHood      = true;
    private boolean updateFlywheel  = true;
    public  boolean disabled        = false;
    public  boolean turretDisabled  = false;

    public Pose currentPose;

    // ── RPM filter ───────────────────────────────────────────────────────────
    private final double[] rpmBuffer = new double[RPM_WINDOW_SIZE];
    private int    bufferIndex    = 0;
    private double runningRpmSum  = 0;
    public  double filteredRPM;
    public  double filteredRPMPredicted;

    // ── Flywheel PID ─────────────────────────────────────────────────────────
    private final PIDController flywheelPID;
    private double lastSpeedFlywheel = 0;
    private boolean isBraking        = false;

    private long   lastLoopTime = 0;

    // ── Turret motion tracking ────────────────────────────────────────────────
    private double lastTargetAngle      = 0;
    private long   lastTargetUpdateTime = 0;
    private double targetVel            = 0;
    private double targetAccel          = 0;
    private double filteredTargetVel    = 0;
    public  double filteredTargetAccel  = 0;
    public  double filteredTargetAccelFlywheel = 0;

    // ── Level 1 – live battery voltage ───────────────────────────────────────
    private double  voltage                 = 12.0;
    private boolean voltageExternallySupplied = false;

    // ── Level 3/4 – last applied voltages (needed for encoder-less current est.)
    private double lastFlywheelAppliedVoltage = 0.0;
    private double lastTurretAppliedVoltage   = 0.0;

    private LaunchZone zoneCalculator = LaunchZone.CLOSE;

    // ────────────────────────────────────────────────────────────────────────
    // Constructor
    // ────────────────────────────────────────────────────────────────────────

    public Shooter(final HardwareMap hardwareMap,
                   final PoseTracker poseTracker,
                   IShooterCalculator shooterCalculatorClose,
                   IShooterCalculator shooterCalculatorFar,
                   Alliance alliance) {
        this.poseTracker = poseTracker;
        this.kinematics  = new Kinematics();

        // ── Flywheel motors ──────────────────────────────────────────────────
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

        // ── Level 4 – configure flywheel physics model ───────────────────────
        flywheel1.setMotorPhysicsConstants(
                12.0,
                FLYWHEEL_STALL_CURRENT_A,
                FLYWHEEL_NO_LOAD_RPM,
                FLYWHEEL_STALL_TORQUE_NM,
                FLYWHEEL_NO_LOAD_CURRENT_A
        );
        // Level 2 – driving and regen current limits baked into the motor.
        // applyVoltage() will enforce these automatically every call.
        flywheel1.setCurrentLimits(FLYWHEEL_MAX_DRIVING_CURRENT_A, FLYWHEEL_MAX_REGEN_CURRENT_A);

        flywheel2.setMotorPhysicsConstants(
                12.0,
                FLYWHEEL_STALL_CURRENT_A,
                FLYWHEEL_NO_LOAD_RPM,
                FLYWHEEL_STALL_TORQUE_NM,
                FLYWHEEL_NO_LOAD_CURRENT_A
        );
        flywheel2.setCurrentLimits(FLYWHEEL_MAX_DRIVING_CURRENT_A, FLYWHEEL_MAX_REGEN_CURRENT_A);

        flywheelPID = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);

        // ── Turret motor ─────────────────────────────────────────────────────
        turret = new ModifiedMotorEx(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));
        turret.setInverted(TURRET_INVERTED);

        // ── Level 3 – configure turret physics model ─────────────────────────
        turret.setMotorPhysicsConstants(
                12.0,
                TURRET_STALL_CURRENT_A,
                TURRET_NO_LOAD_RPM,
                TURRET_STALL_TORQUE_NM,
                TURRET_NO_LOAD_CURRENT_A
        );
        // Level 3 – turret current limits. Regen limit protects bus on hard
        // direction reversals; driving limit protects gears on hard stops.
        turret.setCurrentLimits(TURRET_MAX_DRIVING_CURRENT_A, TURRET_MAX_REGEN_CURRENT_A);

        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPID.setTolerance(TURRET_POSITION_TOLERANCE, TURRET_VELOCITY_TOLERANCE);
        setHorizontalAngle(0);

        hood = new ServoEx(hardwareMap, HOOD_NAME);

        this.shooterCalculatorClose = shooterCalculatorClose;
        this.shooterCalculatorFar   = shooterCalculatorFar;
        this.alliance               = alliance;
        this.goalPose    = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL     : GoalPositions.RED_GOAL;
        this.goalPoseFar = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL_FAR : GoalPositions.RED_GOAL_FAR;

        recoveryTimer = new TimerEx(TimeUnit.SECONDS);
        stallTimer    = new TimerEx(TimeUnit.SECONDS);
        rampTimer     = new TimerEx(TimeUnit.SECONDS);
        rampTimer.start();
        startupTimer  = new TimerEx(TimeUnit.SECONDS);
        startupTimer.start();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    // ────────────────────────────────────────────────────────────────────────
    // Periodic
    // ────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        if (poseTracker == null) return;
        currentPose = poseTracker.getPose();

        if (emergencyStop) return;

        if (isFlywheelDamaged() && !emergencyStop) {
            flywheel.stopMotor();
            turret.stopMotor();
            emergencyStop    = true;
            canShootRPMCalc  = false;
            return;
        }

        // ── Level 1 – Read live battery voltage every loop ───────────────────
        if (!voltageExternallySupplied) {
            this.voltage = voltageSensor.getVoltage();
        }

        kinematics.update(poseTracker, ACCELERATION_SMOOTHING_GAIN);

        filteredRPM = getFilteredRPM(getRPM());

        if (zoneCalculator == LaunchZone.CLOSE) {
            solution = shooterCalculatorClose.getShootingSolution(
                    currentPose == null ? poseTracker.getPose() : currentPose,
                    goalPose,
                    poseTracker.getVelocity(),
                    poseTracker.getAngularVelocity(),
                    (int) filteredRPM);
        } else if (zoneCalculator == LaunchZone.FAR) {
            solution = shooterCalculatorFar.getShootingSolution(
                    currentPose == null ? poseTracker.getPose() : currentPose,
                    goalPoseFar,
                    poseTracker.getVelocity(),
                    poseTracker.getAngularVelocity(),
                    (int) filteredRPM);
        }

        canShootRPMCalc = solution.getCanShoot();

        if (!horizontalManualMode && !disabled)              setHorizontalAngle(solution.getHorizontalAngle() + horizontalOffset);
        if (!verticalManualMode && updateHood && !disabled)  setVerticalAngle(solution.getVerticalAngle() + verticalOffset);
        if (updateFlywheel)                                   setRPM(solution.getRPM());

        // ── Run control loops ────────────────────────────────────────────────
        updateFlywheelPIDFiltered();
        if (!turretDisabled) updateTurretPID(false);

        voltageExternallySupplied = false;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Level 4+5 – Flywheel: Torque-Current Control with PID on top
    // ────────────────────────────────────────────────────────────────────────

    /**
     * Flywheel control loop combining:
     *   - Level 1: all voltages normalised to live battery V
     *   - Level 4: back-EMF feedforward (torque-current model) replaces raw Kv/Ka
     *   - Level 5: PID velocity error correction layered on top
     *   - Asymmetric gain scheduling (braking vs accelerating)
     *   - Ramp limiter on startup to protect hardware
     *   - Current ceiling clamp to protect fuses and motor windings
     *
     * All gains and process variables are in rad/s (or V/(rad/s) for Kv, etc.)
     * so they are physically meaningful and independent of encoder resolution.
     *
     * Unit convention:
     *   targetSpeed, processVariable, error  →  rad/s
     *   targetAcceleration                   →  rad/s²
     *   FLYWHEEL_KP / KP_DOWN / KI / KD      →  V / (rad/s)
     *   FLYWHEEL_KV                          →  V / (rad/s)     (fallback only)
     *   FLYWHEEL_KA / KA_DOWN                →  V / (rad/s²)
     *   FLYWHEEL_KS                          →  V               (static offset)
     *   BRAKE_ENTRY/EXIT_THRESHOLD           →  rad/s
     */
    private void updateFlywheelPIDFiltered() {
        if (disabled || emergencyStop) {
            flywheel.set(0);
            rampTimer.restart();
            rampTimer.pause();
            lastSpeedFlywheel          = 0;
            lastFlywheelAppliedVoltage = 0;
            lastLoopTime               = System.nanoTime();
            return;
        }

        // ── 0. Unit conversion factor ─────────────────────────────────────────
        // All control math happens in rad/s. Convert at the boundary only.
        //   ticks/s  →  rad/s  :  multiply by  (2π / CPR)
        //   rad/s    →  ticks/s:  multiply by  (CPR / 2π)
        final double CPR              = flywheel.getCPR();
        final double ticksToRadPerSec = (2.0 * Math.PI) / CPR;  // rad/s per tick/s
        final double radToTicksPerSec = CPR / (2.0 * Math.PI);  // tick/s per rad/s

        // ── 1. dt ────────────────────────────────────────────────────────────
        long currentTime = System.nanoTime();
        if (lastLoopTime == 0) lastLoopTime = currentTime;
        double dt = (currentTime - lastLoopTime) / 1.0e9;
        lastLoopTime = currentTime;
        if (dt < 0.001) dt = 0.001;

        // ── 2. Ramp limiter ───────────────────────────────────────────────────
        if (!rampTimer.isOn()) rampTimer.resume();
        double rampMultiplier = Math.min(rampTimer.getElapsed() / INITIAL_RAMP_DURATION, 1.0);

        // targetTPS is stored internally in ticks/s; convert to rad/s here
        double targetSpeed = targetTPS * ticksToRadPerSec * rampMultiplier;  // rad/s

        // ── 3. Process variable: filtered RPM → rad/s ────────────────────────
        // RPM × (2π / 60) = rad/s
        double processVariable = filteredRPM * (2.0 * Math.PI / 60.0);      // rad/s

        // ── 4. Asymmetric P-gain (braking vs normal) ─────────────────────────
        // BRAKE_ENTRY_THRESHOLD and BRAKE_EXIT_THRESHOLD are now in rad/s in config
        double error = targetSpeed - processVariable;
        if      (!isBraking && error < -BRAKE_ENTRY_THRESHOLD) isBraking = true;
        else if ( isBraking && error > -BRAKE_EXIT_THRESHOLD)  isBraking = false;

        flywheelPID.setPID(
                isBraking ? FLYWHEEL_KP_DOWN : FLYWHEEL_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD
        );

        // ── 5. PID (operates fully in rad/s) ─────────────────────────────────
        // Kp units: V / (rad/s)  →  output: V
        // Ki units: V / (rad)    →  output: V
        // Kd units: V / (rad/s²) →  output: V
        double pidOutput = flywheelPID.calculate(processVariable, targetSpeed);

        // ── 6. Level 4 – Torque-current feedforward ──────────────────────────
        // targetAcceleration is in rad/s²; Ka is in V/(rad/s²)  →  output: V
        double targetAcceleration = (targetSpeed - lastSpeedFlywheel) / dt;  // rad/s²
        double currentKA          = (targetAcceleration >= 0) ? FLYWHEEL_KA : FLYWHEEL_KA_DOWN;

        double ffVoltage;
        if (flywheel1.isPhysicsConfigured()) {
            // Physics-based feedforward (Level 4).
            // Back-EMF from the motor model is already in volts;
            // it implicitly captures the Kv term — no separate Kv needed.
            double backEmf         = flywheel1.getBackEmfVoltage();
            double staticFrictionV = FLYWHEEL_KS * Math.signum(targetSpeed);
            double inertialV       = currentKA * targetAcceleration;         // V/(rad/s²) × rad/s²
            ffVoltage = backEmf + staticFrictionV + inertialV;
        } else {
            // Fallback: classic Kv/Ks/Ka  (all gains now in rad/s-based units)
            // FLYWHEEL_KV : V / (rad/s)
            ffVoltage = (FLYWHEEL_KS * Math.signum(targetSpeed))
                    + (FLYWHEEL_KV * targetSpeed)
                    + (currentKA   * targetAcceleration);
        }

        lastSpeedFlywheel = targetSpeed;   // rad/s — used to compute accel next loop

        // ── 7. Combine PID + FF, then apply ──────────────────────────────────
        // applyVoltage() handles the full pipeline internally:
        //   regen-aware current clamp → battery clamp → normalise → setPower
        // No manual getCurrentLimitedVoltage() needed here.
        double desiredVoltage = pidOutput + ffVoltage;
        lastFlywheelAppliedVoltage = desiredVoltage;

        OpModeManager.getTelemetry().addData("Flywheel measured current: ", flywheel.getCurrent(CurrentUnit.AMPS));
        OpModeManager.getTelemetry().addData("Flywheel calculated current: ", flywheel.getEstimatedCurrent(lastFlywheelAppliedVoltage));
        flywheel1.applyVoltage(desiredVoltage, voltage);
        flywheel2.applyVoltage(desiredVoltage, voltage);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Level 1+3 – Turret: Voltage control + stall current protection
    // ────────────────────────────────────────────────────────────────────────

    /**
     * Turret PID loop with:
     *   - Level 1: voltage-normalised output
     *   - Level 3: current limiting to prevent gear strip on hard stops
     *   - Robot motion compensation feedforward (velocity + acceleration)
     *   - Static friction compensation (banded by flywheel RPM for vibration)
     *   - Optional delay compensation
     */
    private void updateTurretPID(boolean useDelayCompensation) {
        if ((disabled || emergencyStop) && !horizontalManualMode) {
            turret.set(0);
            return;
        }

        // ── 1. Sensor reading with optional delay compensation ────────────────
        double measuredPos = turret.getDistance();
        if (useDelayCompensation) {
            measuredPos += turret.getVelocity() * TURRET_DELAY;
        }

        // ── 2. PID ────────────────────────────────────────────────────────────
        double pid   = turretPID.calculate(measuredPos);
        double error = turretPID.getPositionError();

        if (Math.abs(error) > TURRET_IZONE) {
            turretPID.clearTotalError();
        }

        // ── 3. Robot motion compensation FF ──────────────────────────────────
        double[] netKinematics = getNetTargetKinematics();
        double netVel   = netKinematics[0];
        double netAccel = netKinematics[1];
        double ffBase   = (netVel * TURRET_KV) + (netAccel * TURRET_KA);

        double totalRequest = pid + ffBase;

        // ── 4. Static friction compensation (banded by flywheel RPM) ─────────
        double[] ks         = getBandedTurretKS();
        double staticComp   = 0;
        if (Math.abs(totalRequest) > TURRET_MIN_VOLTAGE) {
            staticComp = (totalRequest > 0) ? TURRET_KS : -TURRET_KS;
        }

        double desiredVoltage = totalRequest + staticComp;

        // ── 6. Apply — applyVoltage() runs the full pipeline internally:
        //   regen-aware current clamp → battery clamp → normalise → setPower

        OpModeManager.getTelemetry().addData("Turret measured current", turret.getCurrent(CurrentUnit.AMPS));
        OpModeManager.getTelemetry().addData("Turret calculated current", turret.getEstimatedCurrent(lastTurretAppliedVoltage));
        lastTurretAppliedVoltage = desiredVoltage;
        turret.applyVoltage(desiredVoltage, voltage);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Turret – two-sided gain scheduling (kept for reference / A-B testing)
    // ────────────────────────────────────────────────────────────────────────

    private void updateTurretPIDTwoSides() {
        if (disabled || emergencyStop) {
            turret.set(0);
            return;
        }

        double currentPos = turret.getDistance();
        double setpoint   = turretPID.getSetPoint();
        double error      = setpoint - currentPos;

        if (error > 0.5 && !isConfiguredCW) {
            turretPID.setPID(TURRET_KP_CW, TURRET_KI_CW, TURRET_KD_CW);
            isConfiguredCW = true;
        } else if (error < -0.5 && isConfiguredCW) {
            turretPID.setPID(TURRET_KP_CCW, TURRET_KI_CCW, TURRET_KD_CCW);
            isConfiguredCW = false;
        }

        double pidOutput = turretPID.calculate(currentPos);

        if (Math.abs(error) > TURRET_IZONE) turretPID.clearTotalError();

        double[] netKinematics = getNetTargetKinematics();
        double ffBase = (netKinematics[0] * TURRET_KV) + (netKinematics[1] * TURRET_KA);
        double totalRequest = pidOutput + ffBase;

        double[] ks         = getBandedTurretKS();
        double staticComp   = 0;
        if (Math.abs(totalRequest) > 0.05) {
            staticComp = (totalRequest > 0) ? ks[1] : -ks[0];
        }

        double desiredVoltage = totalRequest + staticComp;

        // applyVoltage() handles current limiting and voltage normalisation.
        turret.applyVoltage(desiredVoltage, voltage);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Target kinematics (turret motion compensation)
    // ────────────────────────────────────────────────────────────────────────

    public double[] getNetTargetKinematics() {
        long   currentTime        = System.nanoTime();
        double currentTargetAngle = turretPID.getSetPoint();

        if (lastTargetUpdateTime == 0) {
            lastTargetUpdateTime = currentTime;
            lastTargetAngle      = currentTargetAngle;
            return new double[] {0, 0};
        }

        double dt = (currentTime - lastTargetUpdateTime) / 1e9;

        if (dt <= 0.005 || startupTimer.getElapsed() < 0.5) {
            lastTargetAngle      = currentTargetAngle;
            lastTargetUpdateTime = currentTime;
            targetVel            = 0;
            return new double[] {0, 0};
        }

        double deltaAngle   = MathFunctions.normalizeAngleSigned(currentTargetAngle - lastTargetAngle);
        double rawTargetVel = MathUtils.clamp(deltaAngle / dt, -4, 4);
        double rawTargetAccel = MathUtils.clamp((rawTargetVel - targetVel) / dt, -4, 4);

        filteredTargetVel   = Kinematics.lowPassFilter(rawTargetVel,   filteredTargetVel,   TURRET_DERIVATIVE_GAIN);
        filteredTargetAccel = Kinematics.lowPassFilter(rawTargetAccel, filteredTargetAccel, TURRET_SECOND_DERIVATIVE_GAIN);

        targetVel            = rawTargetVel;
        lastTargetAngle      = currentTargetAngle;
        lastTargetUpdateTime = currentTime;

        return new double[] {filteredTargetVel, filteredTargetAccel};
    }

    // ────────────────────────────────────────────────────────────────────────
    // Banded static friction (depends on flywheel vibration level)
    // ────────────────────────────────────────────────────────────────────────

    private double[] getBandedTurretKS() {
        double rpm = Math.abs(filteredRPM);
        if      (rpm < 1000) return new double[]{ TURRET_KS_CW_0,    TURRET_KS_CCW_0    };
        else if (rpm < 2000) return new double[]{ TURRET_KS_CW_1000, TURRET_KS_CCW_1000 };
        else if (rpm < 3000) return new double[]{ TURRET_KS_CW_2000, TURRET_KS_CCW_2000 };
        else                 return new double[]{ TURRET_KS_CW_3000, TURRET_KS_CCW_3000 };
    }

    // ────────────────────────────────────────────────────────────────────────
    // Safety
    // ────────────────────────────────────────────────────────────────────────

    public boolean isFlywheelDamaged() {
        double currentRPM = Math.abs(getRPM());
        double targetRPM  = Math.abs(getTargetRPM());
        boolean isOverCurrent = flywheel.isOverCurrent();

        if ((getRPM() < -500 && targetRPM > 0) || (getRPM() > 500 && targetRPM < 0)) {
            RobotLog.addGlobalWarningMessage("FLYWHEEL IS SPINNING IN THE WRONG DIRECTION.");
            return true;
        }

        boolean isStalling = targetRPM > 1000 && currentRPM < 200 && isOverCurrent;
        if (isStalling) {
            stallTimer.start();
            stallTimer.resume();
            if (stallTimer.getElapsed() > STALL_TIMEOUT) {
                RobotLog.addGlobalWarningMessage("FLYWHEEL STALL DETECTED.");
                return true;
            }
        } else {
            stallTimer.restart();
            stallTimer.pause();
        }
        return false;
    }

    // ────────────────────────────────────────────────────────────────────────
    // RPM helpers
    // ────────────────────────────────────────────────────────────────────────

    public double getRPM() {
        return (flywheel.getVelocity() * 60.0) / flywheel.getCPR();
    }

    public double getRPMCorrectedTiming() {
        double motorTPS = flywheel.getVelocity();
        if (!Double.isNaN(flywheel1.getAcceleration())) {
            motorTPS += flywheel1.getAcceleration() * FLYWHEEL_SHOOTING_DIFFRENCE;
        }
        return (motorTPS * 60.0) / flywheel.getCPR();
    }

    public double getTargetRPM() {
        return (targetTPS * 60.0) / flywheel.getCPR();
    }

    private double getFilteredRPM(double currentRPM) {
        runningRpmSum          -= rpmBuffer[bufferIndex];
        rpmBuffer[bufferIndex]  = currentRPM;
        runningRpmSum          += currentRPM;
        bufferIndex             = (bufferIndex + 1) % RPM_WINDOW_SIZE;
        return runningRpmSum / RPM_WINDOW_SIZE;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Shoot-readiness
    // ────────────────────────────────────────────────────────────────────────

    public boolean getCanShootRPMCalc() { return canShootRPMCalc; }

    public boolean getCanShoot() { return canShootRPMCalc && reachedAngle(); }

    public double getTurretWindow() {
        double distance     = poseTracker.getPose().distanceFrom(goalPose);
        double t            = MathUtils.clamp((distance - DIST_CLOSE) / (DIST_FAR - DIST_CLOSE), 0.0, 1.0);
        double baseWindow   = WINDOW_CLOSE + t * (WINDOW_FAR - WINDOW_CLOSE);
        double velocityGrace = poseTracker.getVelocity().getMagnitude() * VELOCITY_WINDOW_GAIN;
        return Math.min(baseWindow + velocityGrace, MAX_WINDOW_SIZE);
    }

    public boolean reachedAngle() {
        return Math.abs(turretPID.getPositionError()) <= getTurretWindow()
                && Math.abs(turretPID.getVelocityError()) <= TURRET_VELOCITY_WINDOW;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Hood / servo helpers
    // ────────────────────────────────────────────────────────────────────────

    public double getRawHoodPosition() { return hood.getRawPosition(); }

    public void setHoodPosition(double angle) {
        hood.set(MathFunctions.clamp(angle, HOOD_POSSIBLE_MIN, HOOD_POSSIBLE_MAX));
    }

    public double getHoodAngle() {
        double pos           = getRawHoodPosition();
        double normalizedPos = (pos - HOOD_POSSIBLE_MIN) / (HOOD_POSSIBLE_MAX - HOOD_POSSIBLE_MIN);
        if (HOOD_INVERTED) normalizedPos = 1.0 - normalizedPos;
        return HOOD_MIN + (normalizedPos * (HOOD_MAX - HOOD_MIN));
    }

    public double getHoodAngleDegrees() { return Math.toDegrees(getHoodAngle()); }

    public void setVerticalAngle(double angleRad) {
        double clampedAngle = MathUtils.clamp(angleRad, HOOD_MIN, HOOD_MAX);
        double t            = (clampedAngle - HOOD_MIN) / (HOOD_MAX - HOOD_MIN);
        double targetPos    = HOOD_POSSIBLE_MIN + (t * (HOOD_POSSIBLE_MAX - HOOD_POSSIBLE_MIN));
        if (HOOD_INVERTED) targetPos = HOOD_POSSIBLE_MAX - (targetPos - HOOD_POSSIBLE_MIN);
        setHoodPosition(targetPos);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Turret helpers
    // ────────────────────────────────────────────────────────────────────────

    public double getTurretPosition()                          { return turret.getCurrentPosition(); }
    public double getTurretAngle(AngleUnit angleUnit)          { return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(turret.getDistance()) : turret.getDistance(); }
    public double getTurretAngleVel()                          { return turret.getVelocity(); }
    public double getTurretAngleAccel()                        { return turret.getAcceleration(); }
    public void   resetTurret()                                { turret.resetEncoder(); }

    public void setHorizontalAngle(double targetAngleRad) {
        wrapped = IShooterCalculator.wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP);
        turretPID.setSetPoint(wrapped);
        turret.setTargetDistance(wrapped);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Diagnostics – useful for telemetry while tuning physics constants
    // ────────────────────────────────────────────────────────────────────────

    /** Model-estimated flywheel current (A). Compare with flywheel.getCurrent() for validation. */
    public double getEstimatedFlywheelCurrent() {
        return flywheel1.getEstimatedCurrent(lastFlywheelAppliedVoltage);
    }

    /** Model-estimated turret current (A). */
    public double getEstimatedTurretCurrent() {
        return turret.getEstimatedCurrent(lastTurretAppliedVoltage);
    }

    /** Back-EMF voltage of flywheel1 at current speed. */
    public double getFlywheelBackEmf() {
        return flywheel1.getBackEmfVoltage();
    }

    /** Back-EMF voltage of turret at current speed. */
    public double getTurretBackEmf() {
        return turret.getBackEmfVoltage();
    }

    // ────────────────────────────────────────────────────────────────────────
    // State setters / getters
    // ────────────────────────────────────────────────────────────────────────

    public void setHorizontalManualMode(boolean enabled)  { horizontalManualMode = enabled; }
    public void setVerticalManualMode(boolean enabled)    { verticalManualMode   = enabled; }
    public boolean getHorizontalManualMode()              { return horizontalManualMode; }
    public boolean getVerticalManualMode()                { return verticalManualMode; }

    public void   setHorizontalOffset(double offset) { horizontalOffset = offset; }
    public void   setVerticalOffset(double offset)   { verticalOffset   = offset; }
    public double getHorizontalOffset()              { return horizontalOffset; }
    public double getVerticalOffset()                { return verticalOffset; }

    public void setZoneCalculator(LaunchZone zone) { zoneCalculator = zone; }

    public void setUpdateHood(boolean enabled)     { updateHood     = enabled; }
    public void setUpdateFlywheel(boolean enabled) { updateFlywheel = enabled; }

    public void updateVoltage(double voltage) {
        this.voltage                = voltage;
        voltageExternallySupplied   = true;
    }

    public void disable()        { disabled = true;  flywheel.stopMotor(); }
    public void disableTurret()  { turretDisabled = true; }
    public void enable()         { disabled = false; }
    public void enableTurret()   { turretDisabled = false; }

    public void setTargetPose(Pose pose) { currentPose = pose; }

    public void setRPM(double rpm) {
        rpm           = MathUtils.clamp(rpm, 0, flywheel.getMaxRPM());
        this.targetTPS = (rpm * flywheel.getCPR()) / 60.0;
    }

    public double getRecoveryTime() { return recoveryTime; }

    public boolean justShot() {
        if ((Math.abs(getTargetRPM() - getRPM()) > RPM_REACHED_THRESHOLD)
                && (Math.abs(lastShotRPM - getRPM()) > RPM_REACHED_THRESHOLD)) {
            return true;
        }
        lastShotRPM = getRPM();
        return false;
    }
}