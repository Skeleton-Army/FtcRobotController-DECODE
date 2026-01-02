package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.*;
import static org.firstinspires.ftc.teamcode.consts.ShooterLookupTable.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.psilynx.psikit.core.Logger;

public class LookupTableCalculator implements IShooterCalculator {
    private static final double INCH_TO_METERS = 0.0254;
    private final double[] velCoeffs;

    public LookupTableCalculator(double[] velCoeffs) {
        this.velCoeffs = velCoeffs;
    }

    private double calculateVerticalAngle(double distance, double velocity) {
        if (distance < MIN_DIST || distance > MAX_DIST || velocity < MIN_VEL || velocity > MAX_VEL) {
            return Double.NaN;
        }

        double dIdx = (distance - MIN_DIST) / (MAX_DIST - MIN_DIST) * (DIST_STEPS - 1);
        double vIdx = (velocity - MIN_VEL) / (MAX_VEL - MIN_VEL) * (VEL_STEPS - 1);

        int r0 = (int) dIdx;
        int r1 = Math.min(r0 + 1, DIST_STEPS - 1);
        int c0 = (int) vIdx;
        int c1 = Math.min(c0 + 1, VEL_STEPS - 1);

        // Get values from the four corners
        double v00 = ANGLE_TABLE[r0][c0];
        double v01 = ANGLE_TABLE[r0][c1];
        double v10 = ANGLE_TABLE[r1][c0];
        double v11 = ANGLE_TABLE[r1][c1];

        double dr = dIdx - r0;
        double dc = vIdx - c0;

        // Calculate weights for each corner
        double w00 = (1 - dr) * (1 - dc);
        double w01 = (1 - dr) * dc;
        double w10 = dr * (1 - dc);
        double w11 = dr * dc;

        double weightedSum = 0;
        double totalWeight = 0;

        // Only include valid points (!= -1) in the result
        if (v00 != -1) { weightedSum += v00 * w00; totalWeight += w00; }
        if (v01 != -1) { weightedSum += v01 * w01; totalWeight += w01; }
        if (v10 != -1) { weightedSum += v10 * w10; totalWeight += w10; }
        if (v11 != -1) { weightedSum += v11 * w11; totalWeight += w11; }

        // If no valid points were found in the bounding box, return -1
        if (totalWeight == 0) return -1;

        // Normalize by the sum of weights used
        return weightedSum / totalWeight;
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
        if (distance < MIN_DIST || distance > MAX_DIST) return MIN_VEL;

        double dIdx = (distance - MIN_DIST) / (MAX_DIST - MIN_DIST) * (DIST_STEPS - 1);
        int r0 = (int) dIdx;
        int r1 = Math.min(r0 + 1, DIST_STEPS - 1); // Check the next distance step too

        for (int c = VEL_STEPS - 1; c >= 0; c--) {
            // A velocity is only truly "highest valid" if it's valid for
            // the bounding distance steps used in interpolation.
            if (ANGLE_TABLE[r0][c] != -1 && ANGLE_TABLE[r1][c] != -1) {
                return MIN_VEL + ((double) c / (VEL_STEPS - 1)) * (MAX_VEL - MIN_VEL);
            }
        }

        return MIN_VEL;
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
        double verticalAngle = calculateVerticalAngle(distance, RPMToVelocity(flywheelVel));
        double horizontalAngle = calculateTurretAngle(goalPose, predX / INCH_TO_METERS, predY / INCH_TO_METERS, predHeading);

        // Compute stationary launch vector in field coordinates
        double speed = RPMToVelocity(flywheelVel);
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

        if (verticalAngle == -1 || Double.isNaN(verticalAngle)) {
            newVerticalAngle = -1;
            newHorizontalAngle = horizontalAngle - predHeading;
        }

        return new ShootingSolution(
                MathFunctions.normalizeAngle(newHorizontalAngle),
                newVerticalAngle,
                velocityToRPM(shooterVelocity(distance))
        );
    }
}