package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * An extended motor class that utilizes more features than the
 * regular motor, with latency-compensated velocity control.
 *
 * @author Jackson and Saket
 */
public class ModifiedMotorEx extends Motor {
    public DcMotorEx motorEx;

    private final PIDFController positionController =
            new PIDFController(1, 0, 0, 0);

    // Estimated total control latency (seconds)
    private double delaySec;

    // The minimum difference between the current and requested motor power
    private double cachingTolerance = 0.0001;

    private boolean targetIsSet = false;

//    private double lastSpeed = 0;
//    private double lastTime = 0;

    public ModifiedMotorEx() {}

    /**
     * Constructs the instance motor for the wrapper
     */
    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND =
                motorEx.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    /**
     * Constructs the instance motor for the wrapper
     */
    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id,
                           @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
        motorEx = (DcMotorEx) super.motor;
    }

    /**
     * Constructs an instance motor for the wrapper
     */
    public ModifiedMotorEx(@NonNull HardwareMap hMap, String id,
                           double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
        motorEx = (DcMotorEx) super.motor;
    }

    /* ---------------------------------------------------------------------- */
    /* Latency compensation helpers                                            */
    /* ---------------------------------------------------------------------- */

    private double getPredictedVelocity() {
        // Predict where the motor velocity is NOW, not where encoder says it was
        return getCorrectedVelocity() + getAcceleration() * delaySec;
    }

    public void setDelayCompensation(double delaySec) {
        this.delaySec = delaySec;
    }

    /* ---------------------------------------------------------------------- */
    /* Control logic                                                           */
    /* ---------------------------------------------------------------------- */

    @Override
    public void set(double output) {
        set(output, 12.0);
    }

    public void set(double output, double voltage) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;

            double predictedVelocity = getPredictedVelocity();
            double futureSpeed = speed + getAcceleration() * delaySec;

            double pid =
                    veloController.calculate(predictedVelocity, speed);

            double ff =
                    feedforward.calculate(futureSpeed, getAcceleration());

            double velocityCmd = pid + ff;

            OpModeManager.getTelemetry().addData("Flywheel/PowerCommand", velocityCmd / ACHIEVABLE_MAX_TICKS_PER_SECOND
                    * (12.0 / voltage));

            OpModeManager.getTelemetry().addData("Flywheel/VoltageCommand", velocityCmd / ACHIEVABLE_MAX_TICKS_PER_SECOND);
            setPower(
                    velocityCmd / ACHIEVABLE_MAX_TICKS_PER_SECOND
                            * (12.0 / voltage)
            );
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            setPower(output * error * (12.0 / voltage));

        } else {
            setPower(output * (12.0 / voltage));
        }
    }

//    public void set(double output, double voltage) {
//        if (runmode == RunMode.VelocityControl) {
//            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
//
//            long currentTime = System.currentTimeMillis();
//            double dt = (currentTime - lastTime) / 1000;
//            double targetAcceleration = 0;
//
//            if (dt > 0)
//                targetAcceleration = (speed - lastSpeed) / dt;
//
//            double predictedVelocity = getPredictedVelocity();
//            double futureSpeed = speed + targetAcceleration * delaySec;
//
//            double pid =
//                    veloController.calculate(predictedVelocity, futureSpeed);
//
//            double ff =
//                    feedforward.calculate(futureSpeed, targetAcceleration);
//
//            double velocityCmd = pid + ff;
//
//            setPower(
//                    velocityCmd / voltage
//            );
//
//            lastSpeed = speed;
//            lastTime = currentTime;
//        } else if (runmode == RunMode.PositionControl) {
//            double error = positionController.calculate(getDistance());
//            setPower(output * error * (12.0 / voltage));
//
//        } else {
//            setPower(output * (12.0 / voltage));
//        }
//    }

    /* ---------------------------------------------------------------------- */
    /* Position control                                                        */
    /* ---------------------------------------------------------------------- */

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

    /* ---------------------------------------------------------------------- */
    /* Velocity helpers                                                        */
    /* ---------------------------------------------------------------------- */

    public void setVelocity(double velocity) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    public void setVelocity(double velocity, double voltage) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND, voltage);
    }

    public void setVelocity(double velocity, AngleUnit angleUnit) {
        setVelocity(
                getCPR() * AngleUnit.RADIANS.fromUnit(angleUnit, velocity)
                        / (2 * Math.PI)
        );
    }

    @Override
    public double getVelocity() {
        return motorEx.getVelocity();
    }

    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    /* ---------------------------------------------------------------------- */
    /* Power write caching                                                     */
    /* ---------------------------------------------------------------------- */

    private void setPower(double power) {
        if ((Math.abs(power - motorEx.getPower()) > cachingTolerance)
                || (power == 0 && motorEx.getPower() != 0)) {
            motorEx.setPower(power);
        }
    }

    public double getCachingTolerance() {
        return cachingTolerance;
    }

    public ModifiedMotorEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    /* ---------------------------------------------------------------------- */
    /* Diagnostics                                                             */
    /* ---------------------------------------------------------------------- */

    @Override
    public String getDeviceType() {
        return "Extended " + super.getDeviceType();
    }

    public double getCurrent(CurrentUnit currentUnit) {
        return motorEx.getCurrent(currentUnit);
    }

    public double getCurrentAlert(CurrentUnit currentUnit) {
        return motorEx.getCurrentAlert(currentUnit);
    }

    public void setCurrentAlert(double current, CurrentUnit unit) {
        motorEx.setCurrentAlert(current, unit);
    }

    public boolean isOverCurrent() {
        return motorEx.isOverCurrent();
    }
}
