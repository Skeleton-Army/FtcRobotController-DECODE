package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_MAX;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_MIN;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.RPM_INTERPOLATION;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.VELOCITY_INTERPOLATION;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.MIN_VEL_COEFFS;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.MAX_VEL_COEFFS;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.SHOT_LATENCY;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.RANGE_BUFFER;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.INCH_TO_METERS;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import com.pedropathing.math.Vector;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class ShooterCalculator implements IShooterCalculator {
    private static final double RANGE_BUFFER = 0.05; // Buffer for velocity bounds
    private static final double INCH_TO_METERS = 0.0254;
    private static final double SHOT_LATENCY = 0.1; // Example latency
    private static final double TURRET_OFFSET_X = 0.0; // Example offsets
    private static final double TURRET_OFFSET_Y = 0.0;

    private final double[] hoodCoeffs;

    public ShooterCalculator(double[] hoodCoeffs) {
        this.hoodCoeffs = hoodCoeffs.clone();
    }

    /**
     * Generic helper to evaluate a polynomial of arbitrary degree.
     * Expects coefficients in descending order (highest degree first), matching MATLAB's polyfit.
     */
    private double evaluatePolynomial(double[] coeffs, double x) {
        double result = 0.0;
        for (int exponent = 0; exponent < coeffs.length; exponent++) {
            result += coeffs[exponent] * Math.pow(x, coeffs.length - exponent - 1);
        }
        return result;
    }

    protected double calculateVerticalAngle(double distanceFromGoal) {
        return evaluatePolynomial(hoodCoeffs, distanceFromGoal);
    }

    protected double shooterVelocity(double distance) {
        return evaluatePolynomial(VELOCITY_INTERPOLATION, distance);
    }

    protected int velocityToRPM(double velocity) {
        return (int) evaluatePolynomial(RPM_INTERPOLATION, velocity);
    }

    private double RPMToVelocity(int rpm) {
        if (RPM_INTERPOLATION.length >= 2) {
            return (rpm - RPM_INTERPOLATION[1]) / RPM_INTERPOLATION[0];
        }
        throw new IllegalStateException("RPM_INTERPOLATION has to be a linear polynomial with 2 coefficients for RPM to velocity conversion.");
    }

    /**
     * Checks if the given velocity is within the valid margin for the given distance.
     */
    private HoodSolution lookup(double distance, double velocity) {
        double verticalAngle = calculateVerticalAngle(distance);

        // Evaluate limits for the specific distance using polynomials
        double minLimit = evaluatePolynomial(MIN_VEL_COEFFS, distance);
        double maxLimit = evaluatePolynomial(MAX_VEL_COEFFS, distance);

        // Check if velocity is within the safe "corridor"
        boolean isValid = (velocity >= minLimit - RANGE_BUFFER) && (velocity <= maxLimit + RANGE_BUFFER);

        return new HoodSolution(verticalAngle, isValid);
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
        HoodSolution idealHood = lookup(distance, idealSpeed);

        Vector3D vLaunchIdeal = new Vector3D(
                idealSpeed * Math.cos(idealHood.getHoodAngle()) * Math.cos(horizontalAngleToGoal),
                idealSpeed * Math.cos(idealHood.getHoodAngle()) * Math.sin(horizontalAngleToGoal),
                idealSpeed * Math.sin(idealHood.getHoodAngle())
        );

        Vector3D vRequired = vLaunchIdeal.subtract(vRobot);
        int finalTargetRPM = velocityToRPM(vRequired.getNorm());

        // 2. Check Actual Shot (Current Flywheel Speed)
        double currentSpeed = RPMToVelocity(flywheelVel);
        HoodSolution currentHood = lookup(distance, currentSpeed);

        Vector3D vLaunchActual = new Vector3D(
                currentSpeed * Math.cos(currentHood.getHoodAngle()) * Math.cos(horizontalAngleToGoal),
                currentSpeed * Math.cos(currentHood.getHoodAngle()) * Math.sin(horizontalAngleToGoal),
                currentSpeed * Math.sin(currentHood.getHoodAngle())
        );

        Vector3D vAim = vLaunchActual.subtract(vRobot);

        double aimHorizontalAngle = vAim.getAlpha() - predHeading;
        double aimHorizontalComp = Math.sqrt(vAim.getX() * vAim.getX() + vAim.getY() * vAim.getY());
        double aimVerticalAngle = Math.atan2(vAim.getZ(), aimHorizontalComp);

        boolean isAngleValid = (aimVerticalAngle >= HOOD_MIN) && (aimVerticalAngle <= HOOD_MAX);
        boolean isSolutionPossible = isAngleValid & currentHood.isValid();

        return new ShootingSolution(
                MathFunctions.normalizeAngle(aimHorizontalAngle),
                aimVerticalAngle,
                finalTargetRPM,
                isSolutionPossible,
                vRequired.getNorm()
        );
    }
}