package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Allows multiple {@link ModifiedMotorEx} objects to be linked together
 * as a single group. Multiple motors will act together.
 *
 * @author Jackson
 */
public class ModifiedMotorGroup extends ModifiedMotorEx implements Iterable<ModifiedMotorEx> {

    private final ModifiedMotorEx[] group;

    /**
     * Create a new MotorGroup with the provided Motors.
     *
     * @param leader    The leader motor.
     * @param followers The follower motors which follow the leader motor's protocols.
     */
    public ModifiedMotorGroup(@NonNull ModifiedMotorEx leader, ModifiedMotorEx... followers) {
        group = new ModifiedMotorEx[followers.length + 1];
        group[0] = leader;
        System.arraycopy(followers, 0, group, 1, followers.length);
    }

    /**
     * Set the speed for each motor in the group
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    @Override
    public void set(double speed) {
        group[0].set(speed);
        for (int i = 1; i < group.length; i++) {
            group[i].set(group[0].get());
        }
    }

    @Override
    public void set(double speed, double voltage) {
        group[0].set(speed, voltage);
        for (int i = 1; i < group.length; i++) {
            group[i].set(group[0].get(), voltage);
        }
    }

    /**
     * @return The speed as a percentage of output
     */
    @Override
    public double get() {
        return group[0].get();
    }

    /**
     * @return All motor target speeds as a percentage of output
     */
    public List<Double> getSpeeds() {
        return Arrays.stream(group)
                .map(ModifiedMotorEx::get)
                .collect(Collectors.toList());
    }

    @Override
    public double getVelocity() {
        return group[0].getCorrectedVelocity();
    }

    /**
     * @return All current velocities of the motors in the group in units of distance
     * per second which is by default ticks / second
     */
    public List<Double> getVelocities() {
        return Arrays.stream(group)
                .map(ModifiedMotorEx::getRate)
                .collect(Collectors.toList());
    }

    @NonNull
    @Override
    public Iterator<ModifiedMotorEx> iterator() {
        return Arrays.asList(group).iterator();
    }

    @Override
    public Encoder setDistancePerPulse(double distancePerPulse) {
        Encoder leaderEncoder = group[0].setDistancePerPulse(distancePerPulse);
        for (int i = 1; i < group.length; i++) {
            group[i].setDistancePerPulse(distancePerPulse);
        }
        return leaderEncoder;
    }

    /**
     * @return The position of every motor in the group in units of distance
     * which is by default ticks
     */
    public List<Double> getPositions() {
        return Arrays.stream(group)
                .map(ModifiedMotorEx::getDistance)
                .collect(Collectors.toList());
    }

    @Override
    public void setRunMode(RunMode runmode) {
        group[0].setRunMode(runmode);
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        for (ModifiedMotorEx motor : group) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    @Override
    public void resetEncoder() {
        group[0].resetEncoder();
    }

    @Override
    public void stopAndResetEncoder() {
        group[0].stopAndResetEncoder();
    }

    @Override
    public void setPositionCoefficient(double kp) {
        group[0].setPositionCoefficient(kp);
    }

    @Override
    public boolean atTargetPosition() {
        return group[0].atTargetPosition();
    }

    @Override
    public void setTargetPosition(int target) {
        group[0].setTargetPosition(target);
    }

    @Override
    public void setTargetDistance(double target) {
        group[0].setTargetDistance(target);
    }

    @Override
    public void setPositionTolerance(double tolerance) {
        group[0].setPositionTolerance(tolerance);
    }

    @Override
    public void setVeloCoefficients(double kp, double ki, double kd) {
        group[0].setVeloCoefficients(kp, ki, kd);
    }

    @Override
    public void setFeedforwardCoefficients(double ks, double kv) {
        group[0].setFeedforwardCoefficients(ks, kv);
    }

    @Override
    public void setFeedforwardCoefficients(double ks, double kv, double ka) {
        group[0].setFeedforwardCoefficients(ks, kv, ka);
    }

    /**
     * @return true if the motor group is inverted
     */
    @Override
    public boolean getInverted() {
        return group[0].getInverted();
    }

    /**
     * Set the motor group to the inverted direction or forward direction.
     * This directly affects the speed rather than the direction.
     *
     * @param isInverted The state of inversion true is inverted.
     * @return This object for chaining purposes.
     */
    @Override
    public ModifiedMotorGroup setInverted(boolean isInverted) {
        for (ModifiedMotorEx motor : group) {
            motor.setInverted(isInverted);
        }
        return this;
    }

    /**
     * Disables all the motor devices.
     */
    @Override
    public void disable() {
        for (ModifiedMotorEx x : group) {
            x.disable();
        }
    }

    /**
     * @return a string characterizing the device type
     */
    @Override
    public String getDeviceType() {
        return "ModifiedMotorEx Group";
    }

    /**
     * Stops all motors in the group.
     */
    @Override
    public void stopMotor() {
        for (ModifiedMotorEx x : group) {
            x.stopMotor();
        }
    }

    @Override
    public double getCPR() {
        return group[0].getCPR();
    }

    @Override
    public double getMaxRPM() {
        return group[0].getMaxRPM();
    }

    public void setDelayCompensation(double delaySec) {
        for (ModifiedMotorEx x : group) {
            x.setDelayCompensation(delaySec);
        } 
    }
}
