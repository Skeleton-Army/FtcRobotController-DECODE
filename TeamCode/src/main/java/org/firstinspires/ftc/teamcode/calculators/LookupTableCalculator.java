package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.*;
import static org.firstinspires.ftc.teamcode.consts.ShooterLookupTable.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

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

        double v00 = ANGLE_TABLE[r0][c0];
        double v01 = ANGLE_TABLE[r0][c1];
        double v10 = ANGLE_TABLE[r1][c0];
        double v11 = ANGLE_TABLE[r1][c1];

        if (v00 == -1 || v01  == -1 || v10 == -1 || v11 == -1) {
            return -1;
        }

        double dr = dIdx - r0;
        double dc = vIdx - c0;

        return (1 - dr) * (1 - dc) * v00 +
                (1 - dr) * dc * v01 +
                dr * (1 - dc) * v10 +
                dr * dc * v11;
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

    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector robotVel, double angularVel, int flywheelRPM) {
        Pose robotPoseMeters = robotPose.scale(INCH_TO_METERS);
        Vector robotVelMeters = robotVel.times(INCH_TO_METERS);
        Pose goalPoseMeters = goalPose.scale(INCH_TO_METERS);

        // Compute distance/angles
        double currentVelocity = RPMToVelocity(flywheelRPM);
        double distance = robotPoseMeters.distanceFrom(goalPoseMeters);
        double verticalAngle = calculateVerticalAngle(distance, currentVelocity);
        double horizontalAngle = calculateTurretAngle(goalPose, robotPose.getX(), robotPose.getY(), robotPose.getHeading());

        // Compute stationary launch vector in field coordinates
        double vx = currentVelocity * Math.cos(verticalAngle) * Math.cos(horizontalAngle);
        double vy = currentVelocity * Math.cos(verticalAngle) * Math.sin(horizontalAngle);
        double vz = currentVelocity * Math.sin(verticalAngle);
        Vector3D vLaunch = new Vector3D(vx, vy, vz);

        // Robot velocity at firing instant (we assume constant)
        Vector3D velocity = new Vector3D(robotVelMeters.getXComponent(), robotVelMeters.getYComponent(), 0);

        // Relative launch vector (what the shooter must produce)
        Vector3D v0 = vLaunch.subtract(velocity);

        double newHorizontalAngle = v0.getAlpha() - robotPose.getHeading();
        double newVerticalAngle = (verticalAngle == -1 || Double.isNaN(verticalAngle)) ? -1 : v0.getDelta();

        return new ShootingSolution(
                MathFunctions.normalizeAngle(newHorizontalAngle),
                newVerticalAngle,
                velocityToRPM(shooterVelocity(distance))
        );
    }
}