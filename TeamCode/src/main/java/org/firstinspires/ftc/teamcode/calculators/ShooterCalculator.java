package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.GOAL_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.GOAL_Y;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;

import com.pedropathing.geometry.Pose;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class ShooterCalculator implements IShooterCalculator {
    private final double shooterMinVelocity = 11;
    private final double shooterMaxVelocity = 11;
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
            result += hoodCoeffs[exponent] * Math.pow(distanceFromGoal, exponent);
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

    public double calculateTurretAngle(double x, double y, double heading) {
        double turretX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        double angle = Math.atan2(GOAL_Y - turretY, GOAL_X - turretX);
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
        Vector3D velocity = new Vector3D(vel.getXComponent(), vel.getYComponent(), 0);
        double distanceFromGoal = robotPose.distanceFrom(goalPose);
        double staticVerticalAngle = calculateVerticalAngle(distanceFromGoal);
        double horizontalAngle = calculateTurretAngle(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        Vector3D targetVelocityBase = new Vector3D(staticVerticalAngle, horizontalAngle);
        Vector3D targetVelocity = new Vector3D(shooterVelocity(distanceFromGoal), targetVelocityBase);
        Vector3D v0 = targetVelocity.add(velocity.negate());

        return new ShootingSolution(v0.getDelta(), v0.getAlpha(), velocityToRPM(v0.getNorm())); //might be mistake from wrong placement of alpha and delta
    }
}
