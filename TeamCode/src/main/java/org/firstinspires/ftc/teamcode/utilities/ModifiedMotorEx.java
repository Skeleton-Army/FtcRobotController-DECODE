package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * An extended motor class that utilizes more features than the
 * regular motor.
 *
 * @author Jackson and Saket
 */
public class ModifiedMotorEx extends Motor {
    public DcMotorEx motorEx;

    private final PIDFController positionController = new PIDFController(1, 0, 0, 0);

    // The minimum difference between the current and requested motor power between motor writes
    private double cachingTolerance = 0.0001;

    private boolean targetIsSet = false;

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motorEx.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
        motorEx = (DcMotorEx) super.motor;
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
        motorEx = (DcMotorEx) super.motor;
    }

    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed, getAcceleration());
            setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            setPower(output * error);
        } else {
            setPower(output);
        }
    }

    public void set(double output, double voltage) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed, getAcceleration());
            setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND * (12.0 / voltage));
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            setPower(output * error * (12.0 / voltage));
        } else {
            setPower(output * (12.0 / voltage));
        }
    }

    public void setPositionCoefficients(double kp, double ki, double kd, double kf) {
        positionController.setPIDF(kp, ki, kd, kf);
    }

    /**
     * Set the proportional gain for the position controller.
     *
     * @param kp the proportional gain
     */
    @Override
    public void setPositionCoefficient(double kp) {
        positionController.setP(kp);
    }

    /**
     * @return if the motor is at the target position or distance
     */
    @Override
    public boolean atTargetPosition() {
        return positionController.atSetPoint();
    }

    public double[] getPositionCoefficients() {
        return positionController.getCoefficients();
    }

    /**
     * Sets the {@link RunMode} of the motor
     *
     * @param runmode the desired runmode
     */
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

    /**
     * Sets the target distance for the motor to the desired target.
     * Once {@link #set(double)} is called, the motor will attempt to move in the direction
     * of said target.
     *
     * @param target the target position in units of distance
     */
    @Override
    public void setTargetDistance(double target) {
        targetIsSet = true;
        positionController.setSetPoint(target);
    }

    /**
     * Sets the target tolerance
     *
     * @param tolerance the specified tolerance
     */
    @Override
    public void setPositionTolerance(double tolerance) {
        positionController.setTolerance(tolerance);
    }

    /**
     * @param velocity the velocity in ticks per second
     */
    public void setVelocity(double velocity) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    /**
     * @param velocity the velocity in ticks per second
     * @param voltage the current battery voltage
     */
    public void setVelocity(double velocity, double voltage) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND, voltage);
    }

    /**
     * Sets the velocity of the motor to an angular rate
     *
     * @param velocity  the angular rate
     * @param angleUnit radians or degrees
     */
    public void setVelocity(double velocity, AngleUnit angleUnit) {
        setVelocity(getCPR() * AngleUnit.RADIANS.fromUnit(angleUnit, velocity) / (2 * Math.PI));
    }

    /**
     * @return the velocity of the motor in ticks per second
     */
    @Override
    public double getVelocity() {
        return motorEx.getVelocity();
    }

    /**
     * @return the acceleration of the motor in ticks per second squared
     */
    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    @Override
    public String getDeviceType() {
        return "Extended " + super.getDeviceType();
    }

    /**
     * @return the caching tolerance of the motor before it writes a new power to the motor
     */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    /**
     * @param cachingTolerance the new caching tolerance between motor writes
     * @return this object for chaining purposes
     */
    public ModifiedMotorEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    /**
     * @param power power to be assigned to the motor if difference is greater than caching tolerance or if power is exactly 0
     */
    private void setPower(double power) {
        if ((Math.abs(power - motorEx.getPower()) > cachingTolerance) || (power == 0 && motorEx.getPower() != 0)) {
            motorEx.setPower(power);
        }
    }

    /**
     * Gets the current in the specified unit
     * @param currentUnit the unit to get the current in
     * @return the current in the specified unit
     */
    public double getCurrent(CurrentUnit currentUnit) {
        return motorEx.getCurrent(currentUnit);
    }

    /**
     * Gets the current alert in the specified unit
     * @param currentUnit the unit to get the current alert in
     * @return the current alert in the specified unit
     */
    public double getCurrentAlert(CurrentUnit currentUnit) {
        return motorEx.getCurrentAlert(currentUnit);
    }

    /**
     * Sets the current alert in the specified unit
     * @param current the current alert to set
     * @param unit the unit to set the current alert in
     */
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motorEx.setCurrentAlert(current, unit);
    }

    /**
     * Checks if the motor is over the current limit
     * @return true if the motor is over the current limit, false otherwise
     */
    public boolean isOverCurrent() {
        return motorEx.isOverCurrent();
    }
}