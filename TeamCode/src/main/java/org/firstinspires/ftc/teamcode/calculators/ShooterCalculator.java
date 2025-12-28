package org.firstinspires.ftc.teamcode.calculators;

import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.*;

public class ShooterCalculator extends OnTheMoveCalculator {
    private final double[] hoodCoeffs;
    private final double[] velCoeffs;

    public ShooterCalculator(double[] hoodCoeffs, double[] velCoeffs) {
        this.hoodCoeffs = hoodCoeffs.clone();
        this.velCoeffs = velCoeffs.clone();
    }

    protected double calculateVerticalAngle(double distanceFromGoal) {
        double result = 0.0;
        for (int exponent = 0; exponent < hoodCoeffs.length; exponent++) {
            result += hoodCoeffs[exponent] * Math.pow(distanceFromGoal, hoodCoeffs.length - exponent - 1);
        }
        return result;
    }

    protected double shooterVelocity(double distance) {
        return SHOOTER_MIN_VELOCITY + (distance - MIN_DISTANCE) * (SHOOTER_MAX_VELOCITY - SHOOTER_MIN_VELOCITY) / (MAX_DISTANCE - MIN_DISTANCE);
    }

    protected int velocityToRPM(double velocity) {
        double result = 0.0;
        for (int exponent = 0; exponent < velCoeffs.length; exponent++) {
            result += velCoeffs[exponent] * Math.pow(velocity, velCoeffs.length - exponent - 1);
        }
        return (int)result;
    }
}
