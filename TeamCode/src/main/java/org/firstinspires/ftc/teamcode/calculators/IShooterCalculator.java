package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.util.MathUtils;

public interface IShooterCalculator {
    ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Pose turretGoalPose,Vector robotVel, double angularVel, int flywheelRPM);

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
        // Normalize target to [0, 2π]
        target = target % (2 * Math.PI);
        if (target < 0) target += 2 * Math.PI;

        double closestEquiv = target + 2 * Math.PI * Math.round((current - target) / (2 * Math.PI));

        if (wrap) {
            if (closestEquiv < min) {
                return closestEquiv + 2 * Math.PI;
            }
            if (closestEquiv > max) {
                return closestEquiv - 2 * Math.PI;
            }
            return closestEquiv;
        } else {
            return MathUtils.clamp(closestEquiv, min, max);
        }
    }
}
