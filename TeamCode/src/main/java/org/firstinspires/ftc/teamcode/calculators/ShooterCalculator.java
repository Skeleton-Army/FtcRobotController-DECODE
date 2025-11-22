package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;

import com.pedropathing.geometry.Pose;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class ShooterCalculator {
    private final double SHOOTER_MIN_VELOCITY = 11;
    private final double SHOOTER_MAX_VELOCITY = 7;
    private final double MIN_DISTANCE = 0.7;
    private final double MAX_DISTANCE = 3.5;

    private final double[] hoodCoeffs;
    private final double[] velCoeffs;

    public ShooterCalculator(double[] hoodCoeffs, double[] velCoeffs) {
        this.hoodCoeffs = hoodCoeffs.clone();
        this.velCoeffs = velCoeffs.clone();
    }

    private double calculateVerticalAngle(double distanceFromGoal) {
        double result = 0.0;
        for (int exponent = 0; exponent < hoodCoeffs.length; exponent++) {
            result += hoodCoeffs[exponent] * Math.pow(distanceFromGoal, hoodCoeffs.length - exponent - 1);
        }
        return result;
    }

    /**
     * @param velocity Starting velocity in meters/second
     * @return Motor RPM
     */
    public int velocityToRPM(double velocity) {
        double result = 0.0;
        for (int exponent = 0; exponent < velCoeffs.length; exponent++) {
            result += velCoeffs[exponent] * Math.pow(velocity, exponent);
        }
        return (int)result;
    }

    private double shooterVelocity(double distance) {
        return SHOOTER_MIN_VELOCITY + (distance - MIN_DISTANCE) * (SHOOTER_MAX_VELOCITY - SHOOTER_MIN_VELOCITY) / (MAX_DISTANCE - MIN_DISTANCE);
    }

    public double calculateTurretAngle(Pose targetPose, double x, double y, double heading) {
        double turretX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        double angle = Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);
        double target = angle - heading;

        return MathFunctions.normalizeAngle(target);
    }


    /**
     * @param robotPose Robot position (pedro field coordinates)
     * @param goalPose Goal position (pedro field coordinates)
     * @param vel Robot velocity vector in meters per second
     * @return ShootingSolution at field coordinates (e.g. horizontal angle at relative to the field and not the robot)
     */
    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector vel) {
        Vector3D robotVel = new Vector3D(vel.getXComponent(), vel.getYComponent(), 0);
        double distance = robotPose.distanceFrom(goalPose);

        double inchesToMeters = 39.37;
        double verticalAngle = calculateVerticalAngle(distance / inchesToMeters);
        double horizontalAngle = calculateTurretAngle(goalPose, robotPose.getX(), robotPose.getY(), robotPose.getHeading());

        double speed = shooterVelocity(distance);

        // Stationary launch vector
        double vx = speed * Math.cos(verticalAngle) * Math.cos(horizontalAngle);
        double vy = speed * Math.cos(verticalAngle) * Math.sin(horizontalAngle);
        double vz = speed * Math.sin(verticalAngle);

        Vector3D vLaunch = new Vector3D(vx, vy, vz);

        // Relative to field (robot motion subtraction)
        Vector3D v0 = vLaunch.subtract(robotVel);

        // Extract new angles
        double newSpeed = v0.getNorm();
        double newHorizontalAngle = Math.atan2(v0.getY(), v0.getX());
        double newVerticalAngle = Math.asin(v0.getZ() / newSpeed);

        return new ShootingSolution(
                newHorizontalAngle,
                newVerticalAngle,
                velocityToRPM(newSpeed)
        );
    }

    /**
     * Compute the equivalent of {@code target} (mod 2π) that is nearest to {@code current},
     * but force the result into the physical turret limits [min, max].
     *
     * <p><b>Assumption:</b> {@code current} is inside [min, max]. {@code target} is assumed
     * to be in [0, 2π].
     *
     * @param current current angle in radians (must satisfy min <= current <= max)
     * @param target  desired target angle in radians (0 <= target < 2π)
     * @param min     minimal allowed angle (radians)
     * @param max     maximal allowed angle (radians)
     * @return the target equivalent nearest to current, clamped to [min, max]
     */
    public static double wrapToTarget(double current, double target, double min, double max, boolean wrap) {
        if (!wrap) {
            // No wrapping – just clamp
            if (target < min) return min;
            if (target > max) return max;
            return target;
        }

        double closestEquiv = target + 2 * Math.PI * Math.round((current - target) / (2 * Math.PI));
        if (closestEquiv < min) {
            return closestEquiv + 2 * Math.PI;
        }
        if (closestEquiv > max) {
            return closestEquiv - 2 * Math.PI;
        }
        return closestEquiv;
    }
}
