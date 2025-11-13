package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public interface IShooterCalculator {
    /**
     * Compute the equivalent of {@code target} (mod 2π) that is nearest to {@code current},
     * but force the result into the physical turret limits [min, max].
     *
     * <p><b>Assumption:</b> {@code current} is inside [min, max]. {@code target} is assumed
     * to be in [0, 2π).
     *
     * @param current current angle in radians (must satisfy min <= current <= max)
     * @param target  desired target angle in radians (0 <= target < 2π)
     * @param min     minimal allowed angle (radians)
     * @param max     maximal allowed angle (radians)
     * @return the target equivalent nearest to current, clamped to [min, max]
     */
    public static double wrapToTarget(double current, double target, double min, double max) {
        double closestEquiv = target + 2 * Math.PI * Math.round((current - target) / (2 * Math.PI));
        if (closestEquiv < min) {
            return closestEquiv + 2 * Math.PI;
        }
        if (closestEquiv > max) {
            return closestEquiv - 2 * Math.PI;
        }
        return closestEquiv;
    }
    ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector goalVelocity);
}
