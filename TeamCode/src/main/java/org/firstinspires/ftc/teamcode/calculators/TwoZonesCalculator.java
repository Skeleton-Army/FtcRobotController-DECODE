package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.CLOSE_MAX_DISTANCE;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.*;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.SHOT_LATENCY;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class TwoZonesCalculator implements IShooterCalculator {
    private static final double INCH_TO_METERS = 0.0254;
    private final double[] closeHoodCoeffs;
    private final double[] farHoodCoeffs;
    private final double[] velCoeffs;

    public TwoZonesCalculator(double[] closeHoodCoeffs, double[] farHoodCoeffs, double[] velCoeffs) {
        this.closeHoodCoeffs = closeHoodCoeffs.clone();
        this.farHoodCoeffs = farHoodCoeffs.clone();
        this.velCoeffs = velCoeffs;
    }

    private double calculateVerticalAngle(double distanceFromGoal) {
        double[] hoodCoeffs = distanceFromGoal <= CLOSE_MAX_DISTANCE ? closeHoodCoeffs : farHoodCoeffs;
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
            result += velCoeffs[exponent] * Math.pow(velocity, velCoeffs.length - exponent - 1);
        }
        return (int)result;
    }

    private double shooterVelocity(double distance) {
        if (distance <= CLOSE_MAX_DISTANCE) {
            return CLOSE_SHOOTER_MIN_VELOCITY + (distance - CLOSE_MIN_DISTANCE)
                    * (CLOSE_SHOOTER_MAX_VELOCITY - CLOSE_SHOOTER_MIN_VELOCITY)
                    / (CLOSE_MAX_DISTANCE - CLOSE_MIN_DISTANCE);
        }
        return FAR_SHOOTER_MIN_VELOCITY + (distance - CLOSE_MAX_DISTANCE)
                * (FAR_SHOOTER_MAX_VELOCITY - FAR_SHOOTER_MIN_VELOCITY)
                / (FAR_MAX_DISTANCE - CLOSE_MAX_DISTANCE);
    }
    public double calculateTurretAngle(Pose targetPose, double x, double y, double heading) {
        double turretX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        return Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);
    }
    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector robotVel, double angularVel) {
        Pose robotPoseMeters = robotPose.scale(INCH_TO_METERS);
        Vector robotVelMeters = robotVel.times(INCH_TO_METERS);
        Pose goalPoseMeters = goalPose.scale(INCH_TO_METERS);

        double dx = goalPoseMeters.getX() - robotPoseMeters.getX();
        double dy = goalPoseMeters.getY() - robotPoseMeters.getY();
        double r  = Math.hypot(dx, dy);

        // Predict where robot will be at firing time
        double predX = robotPoseMeters.getX() + robotVelMeters.getXComponent() * SHOT_LATENCY * r;
        double predY = robotPoseMeters.getY() + robotVelMeters.getYComponent() * SHOT_LATENCY * r;
        double predHeading = robotPoseMeters.getHeading();
        Pose predictedPose = new Pose(predX, predY, predHeading);

        // Compute distance/angles from predicted pose
        double distance = predictedPose.distanceFrom(goalPoseMeters);
        double verticalAngle = calculateVerticalAngle(distance);
        double horizontalAngle = calculateTurretAngle(goalPose, predX / INCH_TO_METERS, predY / INCH_TO_METERS, predHeading);

        // Compute stationary launch vector in field coordinates
        double speed = shooterVelocity(distance);
        double vx = speed * Math.cos(verticalAngle) * Math.cos(horizontalAngle);
        double vy = speed * Math.cos(verticalAngle) * Math.sin(horizontalAngle);
        double vz = speed * Math.sin(verticalAngle);
        Vector3D vLaunch = new Vector3D(vx, vy, vz);

        // Robot velocity at firing instant (we assume constant)
        Vector3D velocity = new Vector3D(robotVelMeters.getXComponent(), robotVelMeters.getYComponent(), 0);

        // Relative launch vector (what the shooter must produce)
        Vector3D v0 = vLaunch.subtract(velocity);

        double newSpeed = v0.getNorm();
        double newHorizontalAngle = v0.getAlpha() - predHeading;
        double newVerticalAngle = v0.getDelta();

        return new ShootingSolution(
                MathFunctions.normalizeAngle(newHorizontalAngle),
                newVerticalAngle,
                velocityToRPM(newSpeed)
        );
    }
}
