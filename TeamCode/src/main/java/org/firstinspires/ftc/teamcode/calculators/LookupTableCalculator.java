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

        // 1. Find the first valid column (Left Boundary)
        int firstValid = findBoundary(r0, r1, true);

        // 2. Find the last valid column (Right Boundary)
        int lastValid = findBoundary(r0, r1, false);

        // 3. If no valid overlap found, return a safe fallback
        if (firstValid == -1 || lastValid == -1) {
            return MIN_VEL;
        }

        // 4. Apply bias between the min and max valid velocities
        double bias = Math.max(0.0, Math.min(1.0, VELOCITY_BIAS));
        double chosenColIdx = firstValid + bias * (lastValid - firstValid);

        // 5. Convert column index to velocity
        return MIN_VEL + (chosenColIdx / (VEL_STEPS - 1)) * (MAX_VEL - MIN_VEL);
    }

    /**
     * Binary search to find the start or end of the 'true' block in the VALIDITY_TABLE.
     * @param findFirst If true, searches for the leftmost valid index. If false, rightmost.
     */
    private int findBoundary(int r0, int r1, boolean findFirst) {
        int low = 0, high = VEL_STEPS - 1;
        int result = -1;

        while (low <= high) {
            int mid = low + (high - low) / 2;

            boolean isValid = VALIDITY_TABLE[r0][mid] || VALIDITY_TABLE[r1][mid];

            if (isValid) {
                result = mid;
                if (findFirst) high = mid - 1; // Keep looking left
                else low = mid + 1;           // Keep looking right
            } else {
                // This part is tricky: valid regions in ballistics are usually one
                // continuous hump. If invalid, we need to know which way to move.
                // Usually, low velocity at long distance is invalid, so:
                if (findFirst) low = mid + 1;
                else high = mid - 1;
            }
        }
        return result;
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