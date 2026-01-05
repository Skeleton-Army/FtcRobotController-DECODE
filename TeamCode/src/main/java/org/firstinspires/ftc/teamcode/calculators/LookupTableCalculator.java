package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.*;
import static org.firstinspires.ftc.teamcode.consts.ShooterLookupTable.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.consts.ShooterLookupTable;
import org.psilynx.psikit.core.Logger;

public class LookupTableCalculator implements IShooterCalculator {
    private static final double INCH_TO_METERS = 0.0254;
    private static final double MOVEMENT_COMPENSATION = 0.2;
    private final double[] velCoeffs;

    public LookupTableCalculator(double[] velCoeffs) {
        this.velCoeffs = velCoeffs;
    }

    private double RPMToVelocity(int rpm) {
        // THIS IS SPECIFICALLY FOR 2 COEFFICIENTS, IF THERE ARE MORE THIS NEEDS TO CHANGE
        return (rpm - velCoeffs[1]) / velCoeffs[0];
    }

    /**
     * @param velocity Starting velocity in meters/second
     * @return Motor RPM
     */
    protected int velocityToRPM(double velocity) {
        double result = 0.0;
        for (int exponent = 0; exponent < velCoeffs.length; exponent++) {
            result += velCoeffs[exponent] * Math.pow(velocity, velCoeffs.length - exponent - 1);
        }
        return (int)result;
    }

    protected double shooterVelocity(double distance) {
        double clampedDist = Math.max(MIN_DIST, Math.min(MAX_DIST, distance));

        double dIdx = (clampedDist - MIN_DIST) / (MAX_DIST - MIN_DIST) * (DIST_STEPS - 1);
        int r0 = (int) dIdx;
        int r1 = Math.min(r0 + 1, DIST_STEPS - 1);

        // 1. Collect all valid velocity indices
        int[] validCols = new int[VEL_STEPS];
        int count = 0;

        for (int c = 0; c < VEL_STEPS; c++) {
            if (ANGLE_TABLE[r0][c] != -1 || ANGLE_TABLE[r1][c] != -1) {
                validCols[count++] = c;
            }
        }

        // 2. If none found, fallback
        if (count == 0) {
            return MIN_VEL;
        }

        // 3. Pick index based on bias
        double bias = Math.max(0.0, Math.min(1.0, VELOCITY_BIAS));
        int chosenIndex = (int) Math.round(bias * (count - 1));

        int c = validCols[chosenIndex];

        // 4. Convert column index → velocity
        return MIN_VEL + ((double) c / (VEL_STEPS - 1)) * (MAX_VEL - MIN_VEL);
    }

    public double calculateTurretAngle(Pose targetPose, double x, double y, double heading) {
        double turretX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        return Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);
    }

    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector robotVel, double angularVel, int flywheelVel) {
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
        double speed = RPMToVelocity(flywheelVel);
        HoodSolution hoodSolution = lookup(distance, speed);
        double verticalAngle = hoodSolution.getHoodAngle();
        double horizontalAngle = calculateTurretAngle(goalPose, predX / INCH_TO_METERS, predY / INCH_TO_METERS, predHeading);

        boolean canShoot = hoodSolution.isValid();

        // Compute stationary launch vector in field coordinates
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
                velocityToRPM(shooterVelocity(distance)) * (1 - MOVEMENT_COMPENSATION) + newSpeed * MOVEMENT_COMPENSATION,
                canShoot
        );
    }
}