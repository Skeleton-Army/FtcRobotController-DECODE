package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * An extended motor class with latency-compensated velocity control AND a full
 * brushed-motor physics model, enabling:
 *
 *  Level 1 – Voltage control:       applyVoltage() normalises by live battery V
 *  Level 2/3 – Current limiting:    applyVoltage() clamps driving AND regen current
 *  Level 4 – Torque-current:        getTorqueCurrentVoltage() computes exact V for desired I
 *
 * ── Motor model ──────────────────────────────────────────────────────────────
 *   V_applied = V_backEMF + I * R
 *   V_backEMF = ω * Ke          (ω in rad/s,  Ke in V/(rad/s))
 *   Torque    = I * Kt          (Kt in N·m/A)
 *
 * ── Key method: applyVoltage(desiredVolts, batteryVolts) ─────────────────────
 *   This is the ONE method every subsystem should call to drive a motor.
 *   It runs the full safety pipeline internally:
 *
 *     1. If physics model is configured, clamp desiredVolts to respect the
 *        per-motor current limits set via setCurrentLimits():
 *          - Driving  (V > backEMF): clamp to  backEMF + maxDrivingCurrent × R
 *          - Regen    (V < backEMF): clamp to  backEMF − maxRegenCurrent   × R
 *        The asymmetric limits matter because regen current spikes back into
 *        the Control Hub bus and can corrupt encoders or crash the hub.
 *     2. Hard-clamp to [-batteryVolts, +batteryVolts]
 *     3. Normalise: dutyCycle = clampedVolts / batteryVolts
 *     4. Write to motor hardware via the caching setPower()
 *
 *   Result: any subsystem using ModifiedMotorEx automatically gets fuse
 *   protection, gear protection, and bus-voltage spike protection on every
 *   motor in the robot — not just the shooter.
 *
 * ── Getting motor constants (example: GoBilda 5000-series 12 V bare motor) ──
 *   R  = 12 / stall_current_A                                    (Ω)
 *   Ke = (12 − noLoad_current_A × R) / (noLoad_RPM × 2π / 60)   (V/(rad/s))
 *   Kt = stall_torque_Nm / stall_current_A                       (N·m/A)
 *
 * If you don't have a datasheet yet, don't call setMotorPhysicsConstants().
 * physicsConfigured stays false and applyVoltage() skips the current clamp,
 * behaving identically to a plain voltage-normalised setPower().
 *
 * @author Jackson, Saket, extended by team
 */
public class ModifiedMotorEx extends Motor {

    public DcMotorEx motorEx;

    private final PIDFController positionController =
            new PIDFController(1, 0, 0, 0);

    // ── Latency compensation ─────────────────────────────────────────────────
    private double delaySec = 0.0;

    // ── Power write caching ──────────────────────────────────────────────────
    private double cachingTolerance = 0.0001;

    // ── Motor physics constants ──────────────────────────────────────────────

    /** Winding resistance (Ω). 0 = physics model not configured. */
    private double motorResistanceOhms = 0.0;

    /** Back-EMF constant (V / (rad/s)).  V_backEMF = ω * Ke */
    private double Ke = 0.0;

    /** Torque constant (N·m / A).  Torque = I * Kt */
    private double Kt = 0.0;

    private boolean physicsConfigured = false;

    // ── Per-motor current limits used inside applyVoltage() ──────────────────

    /**
     * Maximum driving current (A).
     * Applied when the requested voltage would push current INTO the motor
     * (normal acceleration / hold).
     * Default: Double.MAX_VALUE = no limit until setCurrentLimits() is called.
     */
    private double maxDrivingCurrentA = Double.MAX_VALUE;

    /**
     * Maximum regenerative current magnitude (A).
     * Applied when the requested voltage is BELOW back-EMF (motor is decelerating
     * and current is flowing back toward the bus).
     * Keeping this lower than maxDrivingCurrentA protects the Control Hub bus
     * from voltage spikes during hard decelerations or direction reversals.
     * Default: Double.MAX_VALUE = no limit.
     */
    private double maxRegenCurrentA = Double.MAX_VALUE;

    private boolean targetIsSet = false;

    // ────────────────────────────────────────────────────────────────────────
    // Constructors
    // ────────────────────────────────────────────────────────────────────────

    public ModifiedMotorEx() {}

    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND =
                motorEx.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id,
                           @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
        motorEx = (DcMotorEx) super.motor;
    }

    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id,
                           double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
        motorEx = (DcMotorEx) super.motor;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Physics model configuration
    // ────────────────────────────────────────────────────────────────────────

    /**
     * Configure the brushed-motor physics model from datasheet values.
     *
     * This must be called before setCurrentLimits() has any effect, because
     * the limits are expressed in Amps which require R and Ke to enforce.
     *
     * @param ratedVoltage   V   – nominal supply voltage (usually 12)
     * @param stallCurrentA  A   – stall current from spec sheet
     * @param noLoadRPM      RPM – free-running speed at rated voltage
     * @param stallTorqueNm  N·m – stall torque from spec sheet
     * @param noLoadCurrentA A   – no-load current draw (for accurate Ke)
     */
    public void setMotorPhysicsConstants(double ratedVoltage,
                                         double stallCurrentA,
                                         double noLoadRPM,
                                         double stallTorqueNm,
                                         double noLoadCurrentA) {
        // R = V / I_stall  (back-EMF is zero at stall, so all voltage drops on R)
        motorResistanceOhms = ratedVoltage / stallCurrentA;

        // Ke: back-EMF at no-load conditions, divided by shaft speed in rad/s
        double noLoadRadPerSec = noLoadRPM * (2.0 * Math.PI / 60.0);
        double backEmfNoLoad   = ratedVoltage - noLoadCurrentA * motorResistanceOhms;
        Ke = (noLoadRadPerSec > 0) ? (backEmfNoLoad / noLoadRadPerSec) : 0.0;

        // Kt
        Kt = (stallCurrentA > 0) ? (stallTorqueNm / stallCurrentA) : 0.0;

        physicsConfigured = true;
    }

    /**
     * Configure physics constants directly (if you have already calculated R, Ke, Kt).
     *
     * @param resistanceOhms  motor winding resistance (Ω)
     * @param Ke              back-EMF constant (V / (rad/s))
     * @param Kt              torque constant (N·m / A)
     */
    public void setMotorPhysicsConstantsDirect(double resistanceOhms, double Ke, double Kt) {
        this.motorResistanceOhms = resistanceOhms;
        this.Ke  = Ke;
        this.Kt  = Kt;
        physicsConfigured = true;
    }

    /**
     * Set asymmetric current limits enforced by every applyVoltage() call.
     *
     * Call this after setMotorPhysicsConstants(). If physicsConfigured is false,
     * these limits are stored but silently ignored until the physics model is set.
     *
     * Choosing values:
     *   maxDrivingCurrentA – how hard the motor can push. Start at 70–80% of
     *       stall current. For a drivetrain motor: ~6–8 A prevents wheel slip
     *       and protects fuses. For a turret: 2–4 A protects gears.
     *
     *   maxRegenCurrentA – how hard the motor can brake. Should be ≤ maxDrivingCurrentA.
     *       A value of 1–2 A prevents voltage spikes on the Control Hub bus during
     *       hard decelerations. For flywheels, this smooths post-shot recovery.
     *       For turrets, it prevents bus spikes on hard direction reversals.
     *
     * @param maxDrivingCurrentA  max current during acceleration/hold (A, positive)
     * @param maxRegenCurrentA    max regenerative current during deceleration (A, positive)
     */
    public void setCurrentLimits(double maxDrivingCurrentA, double maxRegenCurrentA) {
        this.maxDrivingCurrentA = maxDrivingCurrentA;
        this.maxRegenCurrentA   = maxRegenCurrentA;
    }

    /**
     * Convenience overload: sets the same limit for both driving and regen.
     * Use when you only care about total current magnitude, not direction.
     */
    public void setCurrentLimits(double maxCurrentA) {
        setCurrentLimits(maxCurrentA, maxCurrentA);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Physics helpers
    // ────────────────────────────────────────────────────────────────────────

    public boolean isPhysicsConfigured() { return physicsConfigured; }

    /**
     * Back-EMF voltage estimated from the current encoder velocity.
     *   V_backEMF = (ticks/s × 2π/CPR) × Ke
     *
     * @return back-EMF in volts, or 0 if physics model not configured
     */
    public double getBackEmfVoltage() {
        if (!physicsConfigured || Ke == 0) return 0.0;
        double radPerSec = getCorrectedVelocity() * (2.0 * Math.PI / getCPR());
        return radPerSec * Ke;
    }

    /**
     * Model-estimated current draw.
     *   I = (V_applied − V_backEMF) / R
     * Positive = driving current; negative = regenerative.
     *
     * @param appliedVoltage voltage currently sent to the motor (V)
     * @return estimated current in Amps
     */
    public double getEstimatedCurrent(double appliedVoltage) {
        if (!physicsConfigured || motorResistanceOhms == 0) return 0.0;
        return (appliedVoltage - getBackEmfVoltage()) / motorResistanceOhms;
    }

    /**
     * Model-estimated torque.
     *   Torque = I * Kt
     *
     * @param appliedVoltage voltage currently sent to the motor (V)
     * @return estimated torque in N·m
     */
    public double getEstimatedTorque(double appliedVoltage) {
        if (!physicsConfigured) return 0.0;
        return getEstimatedCurrent(appliedVoltage) * Kt;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Level 4 – Compute voltage for a desired torque / current
    // ────────────────────────────────────────────────────────────────────────

    /**
     * Compute the voltage that produces a specific current (and therefore torque)
     * at the motor's current speed.
     *
     *   V = V_backEMF + I_desired × R
     *
     * Use this when you want to command an exact torque (e.g. constant-force hold
     * on an arm, or a precise flywheel acceleration ramp). Pass the result into
     * applyVoltage().
     *
     * @param desiredCurrentA  target current (A). Positive = forward torque.
     * @return voltage to apply (V)
     */
    public double getTorqueCurrentVoltage(double desiredCurrentA) {
        if (!physicsConfigured) return desiredCurrentA; // safe no-op
        return getBackEmfVoltage() + desiredCurrentA * motorResistanceOhms;
    }

    // ────────────────────────────────────────────────────────────────────────
    // applyVoltage — the single canonical motor-drive method
    // ────────────────────────────────────────────────────────────────────────

    /**
     * Apply a desired voltage to the motor, running the full safety pipeline:
     *
     *   Step 1 – Regen-aware current limiting (if physics model is configured):
     *
     *     The allowed voltage window is asymmetric:
     *       Upper bound (driving):  backEMF + maxDrivingCurrentA × R
     *       Lower bound (regen):    backEMF − maxRegenCurrentA   × R
     *
     *     Why asymmetric? When the motor decelerates, current flows *back* toward
     *     the Control Hub bus (regenerative braking). If unclamped, this causes a
     *     voltage spike on the 12 V bus that can crash the hub or corrupt encoder
     *     reads. A separate, typically tighter regen limit prevents this without
     *     affecting normal driving performance.
     *
     *     If setCurrentLimits() was never called, limits default to MAX_VALUE and
     *     this step is a no-op. If setMotorPhysicsConstants() was never called,
     *     this entire step is skipped.
     *
     *   Step 2 – Hard clamp to [−batteryVolts, +batteryVolts]
     *
     *   Step 3 – Voltage normalisation: dutyCycle = clampedVolts / batteryVolts
     *     This is the Level 1 voltage control. By dividing by the live battery
     *     voltage every loop, output stays consistent as the battery sags.
     *
     *   Step 4 – Write via caching setPower() (skips hardware write if unchanged)
     *
     * This method is the only one subsystems should call to drive a motor.
     * Every motor in the robot using ModifiedMotorEx gets all safety for free —
     * no manual current-limiting logic needed in the subsystem code.
     *
     * @param desiredVolts   voltage the controller wants to apply (V).
     *                       Computed by PID+FF, joystick scaling, or getTorqueCurrentVoltage().
     * @param batteryVolts   live battery voltage from VoltageSensor (V).
     *                       Pass 12.0 if no live reading is available.
     */
    public void applyVoltage(double desiredVolts, double batteryVolts) {
        // ── Step 1: Regen-aware current limiting ─────────────────────────────
        if (physicsConfigured && motorResistanceOhms != 0) {
            double backEmf = getBackEmfVoltage();

            // Upper limit: how much voltage can we add on top of back-EMF before
            // driving current exceeds maxDrivingCurrentA?
            double vMax = backEmf + maxDrivingCurrentA * motorResistanceOhms;

            // Lower limit: how far below back-EMF can we go before regenerative
            // current (flowing back into the bus) exceeds maxRegenCurrentA?
            double vMin = backEmf - maxRegenCurrentA * motorResistanceOhms;

            desiredVolts = Math.max(vMin, Math.min(vMax, desiredVolts));
        }

        // ── Step 2: Hard clamp to battery voltage ────────────────────────────
        desiredVolts = Math.max(-batteryVolts, Math.min(batteryVolts, desiredVolts));

        // ── Step 3: Voltage-normalise → duty cycle ───────────────────────────
        double dutyCycle = desiredVolts / batteryVolts;

        // ── Step 4: Write to hardware (cached) ───────────────────────────────
        setPower(dutyCycle);
    }

    /**
     * Convenience overload assuming a nominal 12 V battery.
     * Prefer {@link #applyVoltage(double, double)} with a live voltage reading.
     */
    public void applyVoltage(double desiredVolts) {
        applyVoltage(desiredVolts, 12.0);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Legacy set() — kept for FTCLib VelocityControl / PositionControl modes
    // ────────────────────────────────────────────────────────────────────────

    /**
     * Legacy entry point kept for compatibility with FTCLib VelocityControl and
     * PositionControl run-modes. For RawPower mode use {@link #applyVoltage}.
     */
    @Override
    public void set(double output) {
        set(output, 12.0);
    }

    /** @deprecated Use {@link #applyVoltage(double, double)} for RawPower voltage control. */
    public void set(double output, double voltage) {
        if (runmode == RunMode.VelocityControl) {
            double speed             = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double predictedVelocity = getPredictedVelocity();
            double futureSpeed       = speed + getAcceleration() * delaySec;

            double pid         = veloController.calculate(predictedVelocity, speed);
            double ff          = feedforward.calculate(futureSpeed, getAcceleration());
            double velocityCmd = pid + ff;

            setPower(velocityCmd / ACHIEVABLE_MAX_TICKS_PER_SECOND * (12.0 / voltage));

        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            setPower(output * error * (12.0 / voltage));

        } else {
            // RawPower legacy: caller passes a normalised fraction [-1, 1]
            setPower(output);
        }
    }

    // ────────────────────────────────────────────────────────────────────────
    // Latency compensation
    // ────────────────────────────────────────────────────────────────────────

    private double getPredictedVelocity() {
        return getCorrectedVelocity() + getAcceleration() * delaySec;
    }

    public void setDelayCompensation(double delaySec) {
        this.delaySec = delaySec;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Position control
    // ────────────────────────────────────────────────────────────────────────

    public void setPositionCoefficients(double kp, double ki, double kd, double kf) {
        positionController.setPIDF(kp, ki, kd, kf);
    }

    @Override
    public void setPositionCoefficient(double kp) {
        positionController.setP(kp);
    }

    @Override
    public boolean atTargetPosition() {
        return positionController.atSetPoint();
    }

    public double[] getPositionCoefficients() {
        return positionController.getCoefficients();
    }

    @Override
    public void setRunMode(RunMode runmode) {
        this.runmode = runmode;
        veloController.reset();
        positionController.reset();
        if (runmode == RunMode.PositionControl && !targetIsSet) {
            setTargetPosition(getCurrentPosition());
            targetIsSet = false;
        }
    }

    @Override
    public void setTargetDistance(double target) {
        targetIsSet = true;
        positionController.setSetPoint(target);
    }

    @Override
    public void setPositionTolerance(double tolerance) {
        positionController.setTolerance(tolerance);
    }

    // ────────────────────────────────────────────────────────────────────────
    // Velocity helpers
    // ────────────────────────────────────────────────────────────────────────

    public void setVelocity(double velocity) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    public void setVelocity(double velocity, double voltage) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND, voltage);
    }

    public void setVelocity(double velocity, AngleUnit angleUnit) {
        setVelocity(
                getCPR() * AngleUnit.RADIANS.fromUnit(angleUnit, velocity) / (2 * Math.PI)
        );
    }

    @Override
    public double getVelocity() {
        return motorEx.getVelocity();
    }

    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    // ────────────────────────────────────────────────────────────────────────
    // Power write caching
    // ────────────────────────────────────────────────────────────────────────

    private void setPower(double power) {
        if ((Math.abs(power - motorEx.getPower()) > cachingTolerance)
                || (power == 0 && motorEx.getPower() != 0)) {
            motorEx.setPower(power);
        }
    }

    public double getCachingTolerance() { return cachingTolerance; }

    public ModifiedMotorEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Diagnostics / telemetry
    // ────────────────────────────────────────────────────────────────────────

    @Override
    public String getDeviceType() { return "Extended " + super.getDeviceType(); }

    public double getCurrent(CurrentUnit currentUnit) {
        return motorEx.getCurrent(currentUnit);
    }

    public double getCurrentAlert(CurrentUnit currentUnit) {
        return motorEx.getCurrentAlert(currentUnit);
    }

    public void setCurrentAlert(double current, CurrentUnit unit) {
        motorEx.setCurrentAlert(current, unit);
    }

    public boolean isOverCurrent() { return motorEx.isOverCurrent(); }

    public double getMotorResistance()    { return motorResistanceOhms; }
    public double getKe()                 { return Ke; }
    public double getKt()                 { return Kt; }
    public double getMaxDrivingCurrent()  { return maxDrivingCurrentA; }
    public double getMaxRegenCurrent()    { return maxRegenCurrentA; }
}