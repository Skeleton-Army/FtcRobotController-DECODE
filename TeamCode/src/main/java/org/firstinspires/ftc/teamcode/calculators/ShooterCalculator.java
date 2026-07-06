package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_USABLE_MAX;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_USABLE_MIN;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.LOWER_RANGE_BUFFER;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.SHOT_LATENCY;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.INCH_TO_METERS;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.SHOT_LATENCY_TURRET;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.UPPER_RANGE_BUFFER;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import com.pedropathing.math.Vector;
import com.skeletonarmy.marrow.OpModeManager;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;

public class ShooterCalculator implements IShooterCalculator {
    private static final double INCH_TO_METERS = 0.0254;

    private ShooterCoefficients coefficients;

    public ShooterCalculator(ShooterCoefficients coefficients) {
        this.coefficients = coefficients;
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
        return evaluatePolynomial(coefficients.HOOD_COEFFS, distanceFromGoal);
    }

    protected double shooterVelocity(double distance) {
        return evaluatePolynomial(coefficients.VELOCITY_INTERPOLATION, distance);
    }

    protected int velocityToRPM(double velocity) {
        return (int) evaluatePolynomial(coefficients.RPM_INTERPOLATION, velocity);
    }

    private double RPMToVelocity(int rpm) {
        if (coefficients.RPM_INTERPOLATION.length >= 2) {
            return (rpm - coefficients.RPM_INTERPOLATION[1]) / coefficients.RPM_INTERPOLATION[0];
        }
        throw new IllegalStateException("RPM_INTERPOLATION has to be a linear polynomial with 2 coefficients for RPM to velocity conversion.");
    }

    /**
     * Checks if the given velocity is within the valid margin for the given distance.
     */
    private HoodSolution lookup(double distance, double velocity, double referenceSpeed) {
        double verticalAngle = calculateVerticalAngle(distance);

        double minLimit = evaluatePolynomial(coefficients.MIN_VEL_COEFFS, distance);
        double maxLimit = evaluatePolynomial(coefficients.MAX_VEL_COEFFS, distance);

        // minLimit/maxLimit were calibrated as a margin band around the STATIONARY
        // ideal speed for this distance. When compensating for robot motion, the
        // mechanism's real target speed shifts to referenceSpeed. Shift the whole
        // band by that same delta so the tolerance width is preserved but it's
        // centered on the speed we're actually trying to hit.
        double idealSpeedAtDistance = shooterVelocity(distance);
        double compensationDelta = referenceSpeed - idealSpeedAtDistance;

        double shiftedMin = minLimit + compensationDelta;
        double shiftedMax = maxLimit + compensationDelta;

        boolean isValid = (velocity >= shiftedMin - LOWER_RANGE_BUFFER) && (velocity <= shiftedMax + UPPER_RANGE_BUFFER);

        return new HoodSolution(verticalAngle, isValid);
    }

    // Overload for the stationary case, so existing call sites don't need referenceSpeed
    private HoodSolution lookup(double distance, double velocity) {
        return lookup(distance, velocity, shooterVelocity(distance)); // delta = 0
    }

    public double calculateTurretAngle(Pose targetPose, Pose robotPose) {
        double heading = robotPose.getHeading();
        double turretX = robotPose.getX() + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = robotPose.getY() + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        return Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);
    }

    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector robotVel, double angularVel, int flywheelVel) {
        Pose robotPoseMeters = robotPose.scale(INCH_TO_METERS);
        Vector robotVelMeters = robotVel.times(INCH_TO_METERS);
        Pose goalPoseMeters = goalPose.scale(INCH_TO_METERS);

        double predX = robotPoseMeters.getX() + robotVelMeters.getXComponent() * SHOT_LATENCY;
        double predY = robotPoseMeters.getY() + robotVelMeters.getYComponent() * SHOT_LATENCY;
        double predHeading = robotPoseMeters.getHeading() + angularVel * SHOT_LATENCY_TURRET;
        Pose predictedPose = new Pose(predX, predY, predHeading);

        double distance = goalPoseMeters.distanceFrom(predictedPose);
        double horizontalAngleToGoal = calculateTurretAngle(goalPoseMeters, predictedPose);

        Vector3D vRobot = new Vector3D(robotVelMeters.getXComponent(), robotVelMeters.getYComponent(), 0);

        // 1. Calculate Ideal Shot (stationary baseline — delta is 0 by definition here)
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

        // Validate against the corridor shifted to be centered on vRequired's norm —
        // the speed the flywheel is actually being commanded toward while moving —
        // instead of the stale stationary idealSpeed for this distance.
        HoodSolution currentHood = lookup(distance, currentSpeed, vRequired.getNorm());

        Vector3D vLaunchActual = new Vector3D(
                currentSpeed * Math.cos(currentHood.getHoodAngle()) * Math.cos(horizontalAngleToGoal),
                currentSpeed * Math.cos(currentHood.getHoodAngle()) * Math.sin(horizontalAngleToGoal),
                currentSpeed * Math.sin(currentHood.getHoodAngle())
        );

        Vector3D vAim = vLaunchActual.subtract(vRobot);

        double aimHorizontalAngle = vAim.getAlpha() - predHeading;
        double aimHorizontalComp = Math.sqrt(vAim.getX() * vAim.getX() + vAim.getY() * vAim.getY());
        double aimVerticalAngle = Math.atan2(vAim.getZ(), aimHorizontalComp);

        boolean isAngleValid = (aimVerticalAngle >= HOOD_USABLE_MIN) && (aimVerticalAngle <= HOOD_USABLE_MAX);
        boolean isSolutionPossible = isAngleValid && currentHood.isValid();

        OpModeManager.getTelemetry().addData("isAngleValid", isAngleValid);
        OpModeManager.getTelemetry().addData("currentHood.isValid()", currentHood.isValid());

        return new ShootingSolution(
                MathFunctions.normalizeAngle(aimHorizontalAngle),
                aimVerticalAngle,
                finalTargetRPM,
                isSolutionPossible,
                vRequired.getNorm()
        );
    }
}