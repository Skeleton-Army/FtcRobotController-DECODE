package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.*;

public class TwoZonesCalculator extends OnTheMoveCalculator {
    private final double[] closeHoodCoeffs;
    private final double[] farHoodCoeffs;
    private final double[] velCoeffs;

    public TwoZonesCalculator(double[] closeHoodCoeffs, double[] farHoodCoeffs, double[] velCoeffs) {
        this.closeHoodCoeffs = closeHoodCoeffs.clone();
        this.farHoodCoeffs = farHoodCoeffs.clone();
        this.velCoeffs = velCoeffs;
    }

    protected double calculateVerticalAngle(double distanceFromGoal) {
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
    protected int velocityToRPM(double velocity) {
        double result = 0.0;
        for (int exponent = 0; exponent < velCoeffs.length; exponent++) {
            result += velCoeffs[exponent] * Math.pow(velocity, velCoeffs.length - exponent - 1);
        }
        return (int)result;
    }

    protected double shooterVelocity(double distance) {
        if (distance <= CLOSE_MAX_DISTANCE) {
            return CLOSE_SHOOTER_MIN_VELOCITY + (distance - CLOSE_MIN_DISTANCE)
                    * (CLOSE_SHOOTER_MAX_VELOCITY - CLOSE_SHOOTER_MIN_VELOCITY)
                    / (CLOSE_MAX_DISTANCE - CLOSE_MIN_DISTANCE);
        }
        return FAR_SHOOTER_MIN_VELOCITY + (distance - CLOSE_MAX_DISTANCE)
                * (FAR_SHOOTER_MAX_VELOCITY - FAR_SHOOTER_MIN_VELOCITY)
                / (FAR_MAX_DISTANCE - CLOSE_MAX_DISTANCE);
    }
}
