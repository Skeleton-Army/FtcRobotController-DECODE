package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;

import com.pedropathing.geometry.Pose;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class ShooterCalculator implements IShooterCalculator {
    private static final double INCH_TO_METERS = 0.0254;
    private static final double SHOT_LATENCY = 0.1;
    private final double shooterMinVelocity = 5;
    private final double shooterMaxVelocity = 8;
    private final double minDistance = 0.7;
    private final double maxDistance = 3.5;
    private double[] hoodCoeffs;
    private double[] velCoeffs;

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
        return shooterMinVelocity + (distance - minDistance) * (shooterMaxVelocity - shooterMinVelocity) / (maxDistance - minDistance);
    }

    public double calculateTurretAngle(Pose targetPose, double x, double y, double heading) {
        double turretX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        return Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);
    }


    /**
     * @param robotPose Robot position (pedro field coordinates)
     * @param goalPose Goal position (pedro field coordinates)
     * @param robotVel Robot velocity vector in meters per second
     * @return ShootingSolution at field coordinates (e.g. horizontal angle at relative to the field and not the robot)
     */
    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector robotVel, double angularVel) {
        // predict where robot will be at firing time
        Pose robotPoseMeters = robotPose.scale(INCH_TO_METERS);
        Vector robotVelMeters = robotVel.times(INCH_TO_METERS);
        double predX = robotPoseMeters.getX() + robotVelMeters.getXComponent() * SHOT_LATENCY;
        double predY = robotPoseMeters.getY() + robotVelMeters.getYComponent() * SHOT_LATENCY;
        double predHeading = robotPoseMeters.getHeading() + angularVel * SHOT_LATENCY;
        Pose predictedPose = new Pose(predX, predY, predHeading);

        // compute distance/angles from predicted pose
        double distance = predictedPose.distanceFrom(goalPose);
        double verticalAngle = calculateVerticalAngle(distance);
        double horizontalAngle = calculateTurretAngle(goalPose, predictedPose.getX(), predictedPose.getY(), predictedPose.getHeading());

        // compute stationary launch vector in field coords (as before)
        double speed = shooterVelocity(distance);
        double vx = speed * Math.cos(verticalAngle) * Math.cos(horizontalAngle);
        double vy = speed * Math.cos(verticalAngle) * Math.sin(horizontalAngle);
        double vz = speed * Math.sin(verticalAngle);
        Vector3D vLaunch = new Vector3D(vx, vy, vz);

        // robot velocity at firing instant (we assume constant)
        Vector3D velocity = new Vector3D(robotVelMeters.getXComponent(), robotVelMeters.getYComponent(), 0);

        // relative launch vector (what the shooter must produce)
        Vector3D v0 = vLaunch.subtract(velocity);

        double newSpeed = v0.getNorm();
        double newHorizontalAngle = v0.getAlpha() - predHeading;
        double newVerticalAngle = v0.getDelta();
        return new ShootingSolution(MathFunctions.normalizeAngle(newHorizontalAngle), newVerticalAngle, velocityToRPM(newSpeed));
    }

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
    public static double wrapToTarget(double current, double target, double min, double max, boolean wrap) {
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
