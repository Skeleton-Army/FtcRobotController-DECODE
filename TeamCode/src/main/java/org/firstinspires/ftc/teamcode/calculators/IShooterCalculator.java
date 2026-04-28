package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.util.MathUtils;

public interface IShooterCalculator {
    ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector robotVel, double angularVel, int flywheelRPM);

    /**
     * Compute the equivalent of {@code target} (mod 2π) that is nearest to {@code current},
     * but force the result into the physical turret limits [min, max].
     *
     * @param current current angle in radians (must satisfy min <= current <= max)
     * @param target  desired target angle in radians
     * @param min     minimal allowed angle (radians)
     * @param max     maximal allowed angle (radians)
     * @return the target equivalent nearest to current, clamped to [min, max]
     */
    static double wrapToTarget(double current, double target, double min, double max, boolean wrap) {
        double delta = Math.atan2(Math.sin(target - current), Math.cos(target - current));
        double closest = current + delta;

        if (!wrap) return MathUtils.clamp(closest, min, max);

        // If within limits, use it
        if (closest >= min && closest <= max) return closest;

        // Check if wrapping to the other side puts us within limits
        double wrapped = (closest < min) ? closest + 2 * Math.PI : closest - 2 * Math.PI;
        if (wrapped >= min && wrapped <= max) return wrapped;

        // If both are outside limits, the target is in a deadzone
        // Clamp to the nearest limit to stop the side-to-side jumping
        return (closest < min) ? min : max;
    }
}
