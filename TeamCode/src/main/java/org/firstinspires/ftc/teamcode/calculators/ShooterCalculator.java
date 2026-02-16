package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.RPM_INTERPOLATION;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.VELOCITY_INTERPOLATION;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.DISTANCE_LOOKUP;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.MIN_VEL_LIMITS;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.MAX_VEL_LIMITS;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.SHOT_LATENCY;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.RANGE_BUFFER;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.INCH_TO_METERS;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class ShooterCalculator implements IShooterCalculator {
    private final double[] hoodCoeffs;

    public ShooterCalculator(double[] hoodCoeffs) {
        this.hoodCoeffs = hoodCoeffs.clone();
    }

    protected double calculateVerticalAngle(double distanceFromGoal) {
        double result = 0.0;
        for (int exponent = 0; exponent < hoodCoeffs.length; exponent++) {
            result += hoodCoeffs[exponent] * Math.pow(distanceFromGoal, hoodCoeffs.length - exponent - 1);
        }
        return result;
    }

    protected double shooterVelocity(double distance) {
        double result = 0.0;
        for (int exponent = 0; exponent < VELOCITY_INTERPOLATION.length; exponent++) {
            result += VELOCITY_INTERPOLATION[exponent] * Math.pow(distance, VELOCITY_INTERPOLATION.length - exponent - 1);
        }
        return result;
    }

    protected int velocityToRPM(double velocity) {
        double result = 0.0;
        for (int exponent = 0; exponent < RPM_INTERPOLATION.length; exponent++) {
            result += RPM_INTERPOLATION[exponent] * Math.pow(velocity, RPM_INTERPOLATION.length - exponent - 1);
        }
        return (int) result;
    }

    private double RPMToVelocity(int rpm) {
        if (RPM_INTERPOLATION.length >= 2) {
            return (rpm - RPM_INTERPOLATION[1]) / RPM_INTERPOLATION[0];
        }
        return 0.0;
    }

    /**
     * Checks if the given velocity is within the valid margin for the given distance.
     */
    private HoodSolution lookup(double distance, double velocity) {
        double verticalAngle = calculateVerticalAngle(distance);

        // Interpolate limits for the specific distance
        double minLimit = linearInterpolate(DISTANCE_LOOKUP, MIN_VEL_LIMITS, distance);
        double maxLimit = linearInterpolate(DISTANCE_LOOKUP, MAX_VEL_LIMITS, distance);

        // Check if velocity is within the safe "corridor"
        // We add a tiny epsilon to handle floating point errors if needed,
        // but generally direct comparison is fine here.
        boolean isValid = (velocity >= minLimit - RANGE_BUFFER) && (velocity <= maxLimit + RANGE_BUFFER);

        return new HoodSolution(verticalAngle, isValid);
    }

    /**
     * Helper to linearly interpolate values from the lookup tables.
     */
    private double linearInterpolate(double[] xTable, double[] yTable, double xVal) {
        if (xTable.length == 0) return 0.0;
        if (xVal <= xTable[0]) return yTable[0];
        if (xVal >= xTable[xTable.length - 1]) return yTable[xTable.length - 1];

        // Binary search to find the correct index (or linear scan since arrays are small)
        int i = 0;
        while (i < xTable.length - 1 && xTable[i + 1] < xVal) {
            i++;
        }

        double x1 = xTable[i];
        double x2 = xTable[i + 1];
        double y1 = yTable[i];
        double y2 = yTable[i + 1];

        // Linear interpolation formula
        return y1 + (xVal - x1) * (y2 - y1) / (x2 - x1);
    }

    public double calculateTurretAngle(Pose targetPose, Pose robotPose) {
        double heading = robotPose.getHeading();
        double turretX = robotPose.getX() + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = robotPose.getY() + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        return Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);
    }

    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Pose turretGoalPose, Vector robotVel, double angularVel, int flywheelVel) {
        Pose robotPoseMeters = robotPose.scale(INCH_TO_METERS);
        Vector robotVelMeters = robotVel.times(INCH_TO_METERS);
        Pose goalPoseMeters = goalPose.scale(INCH_TO_METERS);
        Pose turretGoalPoseMeters = turretGoalPose.scale(INCH_TO_METERS);

        double predX = robotPoseMeters.getX() + robotVelMeters.getXComponent() * SHOT_LATENCY;
        double predY = robotPoseMeters.getY() + robotVelMeters.getYComponent() * SHOT_LATENCY;
        double predHeading = robotPoseMeters.getHeading();
        Pose predictedPose = new Pose(predX, predY, predHeading);

        double distance = goalPoseMeters.distanceFrom(predictedPose);
        double horizontalAngleToGoal = calculateTurretAngle(turretGoalPoseMeters, predictedPose);

        Vector3D vRobot = new Vector3D(robotVelMeters.getXComponent(), robotVelMeters.getYComponent(), 0);

        // 1. Calculate Ideal Shot
        double idealSpeed = shooterVelocity(distance);
        HoodSolution idealHood = lookup(distance, idealSpeed); // Check ideal validity (usually true)

        Vector3D vLaunchIdeal = new Vector3D(
                idealSpeed * Math.cos(idealHood.getHoodAngle()) * Math.cos(horizontalAngleToGoal),
                idealSpeed * Math.cos(idealHood.getHoodAngle()) * Math.sin(horizontalAngleToGoal),
                idealSpeed * Math.sin(idealHood.getHoodAngle())
        );

        Vector3D vRequired = vLaunchIdeal.subtract(vRobot);
        int finalTargetRPM = velocityToRPM(vRequired.getNorm());

        // 2. Check Actual Shot (Current Flywheel Speed)
        double currentSpeed = RPMToVelocity(flywheelVel);
        HoodSolution currentHood = lookup(distance, currentSpeed); // This checks if current speed is safe

        Vector3D vLaunchActual = new Vector3D(
                currentSpeed * Math.cos(currentHood.getHoodAngle()) * Math.cos(horizontalAngleToGoal),
                currentSpeed * Math.cos(currentHood.getHoodAngle()) * Math.sin(horizontalAngleToGoal),
                currentSpeed * Math.sin(currentHood.getHoodAngle())
        );

        Vector3D vAim = vLaunchActual.subtract(vRobot);

        double aimHorizontalAngle = vAim.getAlpha() - predHeading;
        double aimHorizontalComp = Math.sqrt(vAim.getX() * vAim.getX() + vAim.getY() * vAim.getY());
        double aimVerticalAngle = Math.atan2(vAim.getZ(), aimHorizontalComp);

        return new ShootingSolution(
                MathFunctions.normalizeAngle(aimHorizontalAngle),
                aimVerticalAngle,
                finalTargetRPM,
                currentHood.isValid(), // Returns true only if we are within the safety tunnel
                vRequired.getNorm()
        );
    }
}