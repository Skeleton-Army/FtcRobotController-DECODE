package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.SHOT_LATENCY;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public abstract class OnTheMoveCalculator implements IShooterCalculator {
    private static final double INCH_TO_METERS = 0.0254;

    protected abstract double calculateVerticalAngle(double distance);
    protected abstract double shooterVelocity(double distance);

    /**
     * @param velocity Starting velocity in meters/second
     * @return Motor RPM
     */
    protected abstract int velocityToRPM(double velocity);

    public double calculateTurretAngle(Pose targetPose, double x, double y, double heading) {
        double turretX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        return Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);
    }

    /**
     * @param robotPose Robot position (pedro field coordinates)
     * @param goalPose Goal position (pedro field coordinates)
     * @param robotVel Robot velocity vector in inches per second
     * @return ShootingSolution at field coordinates (e.g. horizontal angle at relative to the field and not the robot)
     */
    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Pose turretGoalPose, Vector robotVel, double angularVel, int flywheelVel) {
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
