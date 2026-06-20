package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
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
    static double wrapToTarget(double current, double target, double min, double max, boolean wrap, double lastCommanded) {
        double delta = Math.atan2(Math.sin(target - current), Math.cos(target - current));
        double closest = current + delta;

        if (!wrap) return MathUtils.clamp(closest, min, max);

        boolean closestInRange = closest >= min && closest <= max;

        double wrapped = (closest < min) ? closest + 2 * Math.PI : closest - 2 * Math.PI;
        boolean wrappedInRange = wrapped >= min && wrapped <= max;

        // Only one side is valid -> unambiguous
        if (closestInRange && !wrappedInRange) return closest;
        if (!closestInRange && wrappedInRange) return wrapped;

        // From here on, only two cases remain: both valid (overlap band) or neither valid (deadzone).
        if (closestInRange) {
            // Overlap band: both sides are technically legal. Stick with
            // whichever is closer to what we last commanded, to avoid flip-flop.
            double distToClosest = Math.abs(MathFunctions.normalizeAngleSigned(closest - lastCommanded));
            double distToWrapped = Math.abs(MathFunctions.normalizeAngleSigned(wrapped - lastCommanded));
            return (distToClosest <= distToWrapped) ? closest : wrapped;
        }

        // Neither side reachable -> true deadzone, clamp to nearest limit
        return (closest < min) ? min : max;
    }
}
